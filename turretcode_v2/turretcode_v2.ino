/**
 * ════════════════════════════════════════════════════════════════
 * COMPLETE TURRET TRACKING SYSTEM
 * ════════════════════════════════════════════════════════════════
 * Features:
 * - FastAccelStepper for smooth motor control
 * - FreeRTOS dual-core (Serial2 on Core 0, Motors on Core 1)
 * - Kalman filtering for noise reduction
 * - Ultrasonic fusion for precision when close
 * ════════════════════════════════════════════════════════════════
 */

#include "FastAccelStepper.h"
#include <math.h>

// ══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ══════════════════════════════════════════════════════════════════
#define STEP_PIN          2
#define DIR_PIN           7
#define TRIGGER_PIN       9
#define RADAR_RX          6
#define RADAR_TX          5
#define ULTRASONIC_TRIG   10
#define ULTRASONIC_ECHO   11

// ══════════════════════════════════════════════════════════════════
// MOTOR CONFIGURATION
// ══════════════════════════════════════════════════════════════════
const uint32_t STEPS_PER_TABLE_REV = 200UL * 256 * 3;  // 153600 steps/360°
const float    STEPS_PER_DEGREE    = STEPS_PER_TABLE_REV / 360.0f;  // ~426.67

// ══════════════════════════════════════════════════════════════════
// TRACKING SETTINGS
// ══════════════════════════════════════════════════════════════════
const float    CENTERED_THRESHOLD = 5.0f;
const unsigned long MIN_HIGH_TIME = 2000;

// ══════════════════════════════════════════════════════════════════
// FASTACCELSTEPPER OBJECTS
// ══════════════════════════════════════════════════════════════════
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// ══════════════════════════════════════════════════════════════════
// FREERTOS SHARED DATA (Protected by Mutex)
// ══════════════════════════════════════════════════════════════════
SemaphoreHandle_t dataMutex;
volatile float shared_angle_error = 0;
volatile bool target_valid = false;
volatile float shared_raw_x = 0;
volatile float shared_raw_y = 0;
volatile bool fusion_active = false;

// ══════════════════════════════════════════════════════════════════
// KALMAN FILTER STATE
// ══════════════════════════════════════════════════════════════════
float kalman_angle = 0;
float kalman_velocity = 0;
float kalman_error = 1.0;
const float PROCESS_NOISE = 0.01;
const float MEASUREMENT_NOISE = 3.0;
unsigned long last_kalman_time = 0;

// ══════════════════════════════════════════════════════════════════
// TRIGGER STATE
// ══════════════════════════════════════════════════════════════════
bool trigger_active = false;
unsigned long trigger_start = 0;

// ══════════════════════════════════════════════════════════════════
// LD2450 FRAME PARSER
// ══════════════════════════════════════════════════════════════════
const byte HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
byte buffer[24];
int idx = 0;
bool receiving = false;

// ══════════════════════════════════════════════════════════════════
// FUNCTION: Kalman Filter
// ══════════════════════════════════════════════════════════════════
float applyKalmanFilter(float measured_angle, float dt) {
  // Predict
  float predicted_angle = kalman_angle + (kalman_velocity * dt);
  float predicted_error = kalman_error + PROCESS_NOISE;
  
  // Update
  float kalman_gain = predicted_error / (predicted_error + MEASUREMENT_NOISE);
  kalman_angle = predicted_angle + kalman_gain * (measured_angle - predicted_angle);
  kalman_velocity = (kalman_angle - predicted_angle) / dt;
  kalman_error = (1 - kalman_gain) * predicted_error;
  
  return kalman_angle;
}

// ══════════════════════════════════════════════════════════════════
// FUNCTION: Ultrasonic Fusion
// ══════════════════════════════════════════════════════════════════
float applyUltrasonicFusion(float radar_x, float radar_y, float radar_angle) {
  // Only fuse when nearly centered
  if (fabs(radar_x) > 100) {
    fusion_active = false;
    return radar_angle;
  }
  
  // Fire ultrasonic pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Measure echo
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
  if (duration == 0) {
    fusion_active = false;
    return radar_angle;
  }
  
  // Calculate precise distance
  float ultrasonic_y_cm = (duration * 0.0343) / 2.0;
  float ultrasonic_y_mm = ultrasonic_y_cm * 10.0;
  
  // Validate range
  if (ultrasonic_y_cm < 20 || ultrasonic_y_cm > 400) {
    fusion_active = false;
    return radar_angle;
  }
  
  // Recalculate X using precise Y
  float fused_x_mm = ultrasonic_y_mm * tan(radar_angle * PI / 180.0);
  float fused_angle = atan2(fused_x_mm, ultrasonic_y_mm) * (180.0 / PI);
  
  fusion_active = true;
  Serial.printf("  🎯 FUSION: Y %.0f→%.0fmm | X %.0f→%.0fmm | Angle %.2f°→%.2f°\n",
                radar_y, ultrasonic_y_mm, radar_x, fused_x_mm, radar_angle, fused_angle);
  
  return fused_angle;
}

// ══════════════════════════════════════════════════════════════════
// FREERTOS TASK: Radar Parsing (Core 0)
// ══════════════════════════════════════════════════════════════════
void radarTask(void *parameter) {
  Serial.println("Radar task on Core " + String(xPortGetCoreID()));
  
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
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ══════════════════════════════════════════════════════════════════
// FUNCTION: Parse LD2450 Frame
// ══════════════════════════════════════════════════════════════════
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
    
    int16_t x_mm = (g[1] & 0x80) ? (raw_x & 0x7FFF) : -(raw_x & 0x7FFF);
    int16_t y_mm = (g[3] & 0x80) ? (raw_y & 0x7FFF) : -(raw_y & 0x7FFF);
    
    float dist = sqrtf(x_mm * x_mm + y_mm * y_mm);
    if (dist > 300 && dist < best_dist) {
      best_dist = dist;
      best_x_mm = x_mm;
      best_y_mm = y_mm;
      found = true;
    }
  }
  
  // Update shared data with Kalman + Fusion
  if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
    if (found && best_dist < 6000) {
      float x_cm = best_x_mm / 10.0f;
      float y_cm = best_y_mm / 10.0f;
      float raw_angle = atan2f(x_cm, y_cm) * (180.0f / PI);
      
      // STEP 1: Kalman filter
      unsigned long now = millis();
      float dt = (now - last_kalman_time) / 1000.0;
      float filtered_angle = (dt > 0 && dt < 1.0) ? applyKalmanFilter(raw_angle, dt) : raw_angle;
      if (dt >= 1.0) { kalman_angle = raw_angle; kalman_velocity = 0; }
      last_kalman_time = now;
      
      // STEP 2: Ultrasonic fusion
      float final_angle = applyUltrasonicFusion(best_x_mm, best_y_mm, filtered_angle);
      
      shared_angle_error = final_angle;
      shared_raw_x = best_x_mm;
      shared_raw_y = best_y_mm;
      target_valid = true;
      
      if (fusion_active) {
        Serial.printf("Target: X=%.1fcm Y=%.1fcm | Raw=%.2f° | Kalman=%.2f° | FUSED=%.2f° 🎯\n",
                      x_cm, y_cm, raw_angle, filtered_angle, final_angle);
      } else {
        Serial.printf("Target: X=%.1fcm Y=%.1fcm | Raw=%.2f° | Kalman=%.2f°\n",
                      x_cm, y_cm, raw_angle, filtered_angle);
      }
    } else {
      shared_angle_error = 0;
      target_valid = false;
      fusion_active = false;
    }
    xSemaphoreGive(dataMutex);
  }
}

// ══════════════════════════════════════════════════════════════════
// FREERTOS TASK: Motion Control (Core 1)
// ══════════════════════════════════════════════════════════════════
void motionTask(void *parameter) {
  Serial.println("Motion task on Core " + String(xPortGetCoreID()));
  
  while (true) {
    // Read shared data
    float angle_error;
    bool valid;
    
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      angle_error = shared_angle_error;
      valid = target_valid;
      xSemaphoreGive(dataMutex);
    }
    
    float abs_error = fabs(angle_error);
    
    if (valid && abs_error > CENTERED_THRESHOLD) {
      // Move toward target
      int32_t steps = round(-angle_error * STEPS_PER_DEGREE);
      Serial.printf(">>> MOVING %.1f° (%ld steps)\n", -angle_error, steps);
      
      stepper->move(steps);
      while (stepper->isRunning()) vTaskDelay(1 / portTICK_PERIOD_MS);
      
      trigger_active = false;
      digitalWrite(TRIGGER_PIN, LOW);
      
    } else if (valid && abs_error <= CENTERED_THRESHOLD) {
      // Centered - trigger ON
      digitalWrite(TRIGGER_PIN, HIGH);
      if (!trigger_active) {
        trigger_active = true;
        trigger_start = millis();
        Serial.println(">>> LOCKED - PIN 9 HIGH");
      }
    }
    
    // Turn off after timeout
    if (trigger_active && (millis() - trigger_start >= MIN_HIGH_TIME)) {
      if (!valid || abs_error > 30.0f) {
        trigger_active = false;
        digitalWrite(TRIGGER_PIN, LOW);
        Serial.println(">>> Target lost - PIN 9 LOW");
      }
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ══════════════════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Create mutex
  dataMutex = xSemaphoreCreateMutex();
  if (!dataMutex) {
    Serial.println("Mutex FAILED!");
    while(1);
  }
  
  // Initialize FastAccelStepper
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(-1);
    stepper->setAutoEnable(false);
    stepper->setSpeedInHz(4000);
    stepper->setAcceleration(10000);
    Serial.println("✓ FastAccelStepper ready");
  } else {
    Serial.println("✗ Stepper FAILED!");
    while(1);
  }
  
  // Start radar
  Serial2.begin(256000, SERIAL_8N1, RADAR_RX, RADAR_TX);
  delay(1000);
  
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println("  TURRET: FastAccel + Kalman + Ultrasonic Fusion");
  Serial.println("════════════════════════════════════════════════════════");
  
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(radarTask, "Radar", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(motionTask, "Motion", 4096, NULL, 1, NULL, 1);
}

// ══════════════════════════════════════════════════════════════════
// LOOP (Idle)
// ══════════════════════════════════════════════════════════════════
void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}