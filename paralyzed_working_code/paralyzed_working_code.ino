/**
 * @file   AutomaticTurret.ino
 * @author Connor Klies, Joshua Mcgregor
 * @date   12/3/2025
 * @brief  FreeRTOS dual-core radar-guided turret tracking system
 * 
 * This project implements an automated turret tracking system using the LD2450
 * radar sensor and a stepper motor. The system uses FreeRTOS to split processing
 * across two cores: Core 0 handles radar data parsing while Core 1 controls
 * stepper motor movement and trigger logic. When a target is detected and centered,
 * the system activates a trigger output pin.
 * 
 * @see LD2450 Radar Datasheet
 */

#include "FastAccelStepper.h"
#include <math.h>

/** @brief Stepper motor STEP pulse pin */
#define STEP_PIN 2

/** @brief Stepper motor direction control pin */
#define DIR_PIN 7

/** @brief Output pin for firing trigger (HIGH when target centered) */
#define TRIGGER_PIN 9

/** @brief UART receive pin for LD2450 radar */
#define RADAR_RX 6

/** @brief UART transmit pin for LD2450 radar */
#define RADAR_TX 5

/** @brief Stepper motor steps per degree of rotation (accounting for gear ratio) */
#define STEPS_PER_DEGREE 13.34

/** @brief Angle threshold (degrees) - target considered centered if |error| <= this */
#define CENTERED_THRESHOLD 5.0

/** @brief Minimum time (ms) trigger must stay HIGH before it can turn off */
#define MIN_HIGH_TIME 2000

/** @brief FastAccelStepper engine instance */
FastAccelStepperEngine engine = FastAccelStepperEngine();

/** @brief Pointer to stepper motor object */
FastAccelStepper *stepper = NULL;

/** @brief FreeRTOS mutex for protecting shared data between cores */
SemaphoreHandle_t dataMutex;

/** @brief Shared variable: angular error in degrees (positive = target right of center) */
volatile float shared_angle_error = 0;

/** @brief Shared variable: true if a valid target is currently detected */
volatile bool target_valid = false;

/** @brief Trigger state flag (true when trigger pin is active HIGH) */
bool trigger_active = false;

/** @brief Timestamp (ms) when trigger was activated */
unsigned long trigger_start = 0;

// LD2450 Frame Parser Variables
/** @brief LD2450 protocol frame header bytes */
const byte HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};

/** @brief Buffer for incoming LD2450 frame data (24 bytes payload) */
byte buffer[24];

/** @brief Current index in header matching or buffer filling */
int idx = 0;

/** @brief Flag indicating if currently receiving a frame payload */
bool receiving = false;

// FREERTOS TASK DEFINITIONS

/**
 * @brief FreeRTOS task for radar data parsing (runs on Core 0)
 * 
 * This task continuously monitors Serial2 for incoming LD2450 radar frames,
 * parses the data, and updates shared target information. It runs with higher
 * priority to ensure radar frames are processed without drops. The task uses
 * a state machine to detect frame headers and extract the 24-byte payload.
 * 
 * Frame structure: [4-byte header][24-byte payload][2-byte tail: 0x55 0xCC]
 * 
 * @param parameter Unused task parameter (required by FreeRTOS)
 * 
 * @note This task runs indefinitely and should never return
 * @note Updates shared data (@p shared_angle_error, @p target_valid) via mutex
 * @see parseLD2450Frame()
 */
void radarTask(void *parameter) {
  Serial.println("Radar task starting on core " + String(xPortGetCoreID()));
  
  while (true) {
    while (Serial2.available()) {
      byte b = Serial2.read();

      if (!receiving) {
        if (b == HEADER[idx]) {
          idx++;
          if (idx == 4) { receiving = true; idx = 0; }
        } else idx = 0;
      } else {
        if (idx < 24) buffer[idx++] = b;
        else if (idx == 24 && b == 0x55) idx = 25;
        else if (idx == 25 && b == 0xCC) {
          parseLD2450Frame();
          receiving = false;
          idx = 0;
        } else {
          receiving = false;
          idx = 0;
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // Minimal delay for radar processing
  }
}

/**
 * @brief FreeRTOS task for motion control and trigger logic (runs on Core 1)
 * 
 * This task handles all stepper motor movements and trigger pin control based
 * on target angle error. The control logic implements three states:
 * - <b>Moving</b>: When error > @p CENTERED_THRESHOLD, rotates turret toward target
 * - <b>Centered</b>: When error <= @p CENTERED_THRESHOLD, activates trigger pin
 * - <b>Lost</b>: After @p MIN_HIGH_TIME, deactivates trigger if target disappears
 * 
 * The task safely reads shared radar data using a mutex and executes motor
 * movements without blocking radar parsing on Core 0.
 * 
 * @param parameter Unused task parameter (required by FreeRTOS)
 * 
 * @note This task runs indefinitely with a 20ms control loop period
 * @warning Modifying @p CENTERED_THRESHOLD affects tracking stability
 * @see radarTask()
 */
void motionTask(void *parameter) {
  Serial.println("Motion task starting on core " + String(xPortGetCoreID()));
  
  while (true) {
    // Read shared data safely
    float angle_error;
    bool valid;
    
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      angle_error = shared_angle_error;
      valid = target_valid;
      xSemaphoreGive(dataMutex);
    }

    float abs_error = fabs(angle_error);

    if (valid && abs_error > CENTERED_THRESHOLD) {
      // Need to move toward target
      int32_t steps = round(-angle_error * STEPS_PER_DEGREE);
      Serial.printf("Moving %.1f° (%ld steps)\n", -angle_error, steps);

      stepper->move(steps);
      while (stepper->isRunning()) vTaskDelay(1 / portTICK_PERIOD_MS);

      // We moved → reset trigger
      trigger_active = false;
      digitalWrite(TRIGGER_PIN, LOW);

    } else if (valid && abs_error <= CENTERED_THRESHOLD) {
      // Facing target → Pin 9 HIGH
      digitalWrite(TRIGGER_PIN, HIGH);
      if (!trigger_active) {
        trigger_active = true;
        trigger_start = millis();
        Serial.println("TARGET CENTERED → PIN 9 HIGH");
      }
    }

    // Turn off only after min time if target disappears
    if (trigger_active && (millis() - trigger_start >= MIN_HIGH_TIME)) {
      if (!valid || abs_error > 30.0f) {
        trigger_active = false;
        digitalWrite(TRIGGER_PIN, LOW);
        Serial.println("Target lost → PIN 9 LOW");
      }
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Arduino setup function - initializes hardware and creates FreeRTOS tasks
 * 
 * Performs the following initialization sequence:
 * 1. Configures serial communication and GPIO pins
 * 2. Creates FreeRTOS mutex for thread-safe data sharing
 * 3. Initializes FastAccelStepper motor controller with speed/acceleration profiles
 * 4. Starts LD2450 radar communication at 256000 baud
 * 5. Creates two FreeRTOS tasks pinned to separate cores:
 *    - @p radarTask on Core 0 (priority 2)
 *    - @p motionTask on Core 1 (priority 1)
 * 
 * @note Setup runs once on boot before tasks take control
 * @warning System halts if mutex creation or stepper initialization fails
 * @see radarTask()
 * @see motionTask()
 */
void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  // Create mutex
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    Serial.println("Mutex creation FAILED!");
    while (1);
  }

  // Initialize FastAccelStepper
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(-1);
    stepper->setAutoEnable(false);
    stepper->setSpeedInHz(1200);
    stepper->setAcceleration(3000);
    Serial.println("Stepper ready");
  } else {
    Serial.println("Stepper FAILED!");
    while (1);
  }

  Serial2.begin(256000, SERIAL_8N1, RADAR_RX, RADAR_TX);
  delay(1000);
  Serial.println("=== LD2450 TURRET TRACKER (FreeRTOS) ===");

  // Create tasks on different cores
  xTaskCreatePinnedToCore(
    radarTask,       // Function
    "RadarTask",     // Name
    4096,            // Stack size
    NULL,            // Parameter
    2,               // Priority (higher for radar)
    NULL,            // Task handle
    0                // Core 0
  );

  xTaskCreatePinnedToCore(
    motionTask,      // Function
    "MotionTask",    // Name
    4096,            // Stack size
    NULL,            // Parameter
    1,               // Priority
    NULL,            // Task handle
    1                // Core 1
  );
}

/**
 * @brief Arduino main loop - idle function (all work done by FreeRTOS tasks)
 * 
 * This function is essentially empty in a FreeRTOS application. All real-time
 * processing occurs in the dedicated tasks (@p radarTask and @p motionTask).
 * The loop simply delays to avoid hogging CPU time from the FreeRTOS scheduler.
 * 
 * @note In FreeRTOS ESP32 applications, loop() runs as a low-priority task
 */
void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

/**
 * @brief Parses a complete LD2450 radar frame and updates target tracking data
 * 
 * This function processes the 24-byte payload containing up to 3 target reports.
 * Each target includes X/Y position (signed 16-bit mm), speed, and resolution gate.
 * The function selects the closest valid target within range and calculates the
 * angular error needed to center the turret on that target.
 * 
 * <b>Coordinate System:</b>
 * - X-axis: positive = right, negative = left
 * - Y-axis: positive = forward distance from radar
 * - Angle: atan2(x, y) gives degrees from center (+ = right, - = left)
 * 
 * <b>Target Selection Criteria:</b>
 * - Distance > 300mm (filters noise)
 * - Distance < 6000mm (6 meter max range)
 * - Closest valid target is selected
 * 
 * @note Updates @p shared_angle_error and @p target_valid via mutex
 * @note Coordinate parsing follows official LD2450 protocol (MSB = sign bit)
 * @warning Assumes @p buffer contains a valid 24-byte frame
 * @see radarTask()
 */
void parseLD2450Frame() {
  float best_x_mm = 0, best_y_mm = 0;
  float best_dist = 999999.0f;
  bool found = false;

  for (int i = 0; i < 3; i++) {
    byte* g = buffer + i * 8;

    uint16_t raw_x = g[0] | (g[1] << 8);
    uint16_t raw_y = g[2] | (g[3] << 8);
    uint16_t raw_speed = g[4] | (g[5] << 8);
    uint16_t gate = g[6] | (g[7] << 8);

    if (raw_x == 0 && raw_y == 0 && raw_speed == 0 && gate == 0) continue;

    int16_t x_mm;
    if (g[1] & 0x80) {
      x_mm = raw_x & 0x7FFF;
    } else {
      x_mm = -(raw_x & 0x7FFF);
    }

    int16_t y_mm;
    if (g[3] & 0x80) {
      y_mm = raw_y & 0x7FFF;
    } else {
      y_mm = -(raw_y & 0x7FFF);
    }

    float dist = sqrtf(x_mm * x_mm + y_mm * y_mm);
    if (dist > 300 && dist < best_dist) {
      best_dist = dist;
      best_x_mm = x_mm;
      best_y_mm = y_mm;
      found = true;
    }
  }

  // Update shared data safely
  if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
    if (found && best_dist < 6000) {
      float x_cm = best_x_mm / 10.0f;
      float y_cm = best_y_mm / 10.0f;
      shared_angle_error = atan2f(x_cm, y_cm) * (180.0f / PI);
      target_valid = true;
      Serial.printf("Target: X=%.1fcm Y=%.1fcm → Angle=%.2f°\n", x_cm, y_cm, shared_angle_error);
    } else {
      shared_angle_error = 0;
      target_valid = false;
      Serial.println("No valid target");
    }
    xSemaphoreGive(dataMutex);
  }
}