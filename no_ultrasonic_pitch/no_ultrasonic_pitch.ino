/**
 * CLEANED TURRET TRACKING SYSTEM — ULTRASONIC FOR TARGETING ONLY
 * - Startup pitch 35° preserved
 * - Ultrasonic used ONLY for X/Y fusion (no ballistic pitch)
 * - Fires instantly when yaw locked
 * - Kalman + FastAccelStepper + FreeRTOS preserved
 */

#include "FastAccelStepper.h"
#include <math.h>

// ══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ══════════════════════════════════════════════════════════════════
#define YAW_STEP_PIN      16
#define YAW_DIR_PIN       15
#define PITCH_STEP_PIN    7
#define PITCH_DIR_PIN     6
#define TRIGGER_PIN       1
#define RADAR_RX          5
#define RADAR_TX          4
#define ULTRASONIC_TRIG   10
#define ULTRASONIC_ECHO   11

// ══════════════════════════════════════════════════════════════════
// MOTOR CONFIGURATION
// ══════════════════════════════════════════════════════════════════
const uint16_t MICROSTEPPING          = 8;
const uint16_t STEPS_PER_REV          = 200;
const float    YAW_GEAR_RATIO         = 3.0f;
const float    PITCH_GEAR_RATIO       = 64.0f / 21.0f;
const uint32_t STEPS_PER_MOTOR_REV    = STEPS_PER_REV * MICROSTEPPING;
const uint32_t YAW_STEPS_PER_TABLE_REV   = STEPS_PER_MOTOR_REV * YAW_GEAR_RATIO;
const uint32_t PITCH_STEPS_PER_TABLE_REV = STEPS_PER_MOTOR_REV * PITCH_GEAR_RATIO;
const float    YAW_STEPS_PER_DEGREE   = YAW_STEPS_PER_TABLE_REV / 360.0f;
const float    PITCH_STEPS_PER_DEGREE = PITCH_STEPS_PER_TABLE_REV / 360.0f;

const float    STARTUP_PITCH_ANGLE = 35.0f;
const int32_t  STARTUP_PITCH_STEPS = round(STARTUP_PITCH_ANGLE * PITCH_STEPS_PER_DEGREE);

// ══════════════════════════════════════════════════════════════════
// TRACKING SETTINGS
// ══════════════════════════════════════════════════════════════════
const float    CENTERED_THRESHOLD = 5.0f;           // degrees
const unsigned long TARGET_TIMEOUT_MS = 3000;
const unsigned long MIN_HIGH_TIME = 2000;

// ══════════════════════════════════════════════════════════════════
// FASTACCELSTEPPER
// ══════════════════════════════════════════════════════════════════
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *yaw_stepper = NULL;
FastAccelStepper *pitch_stepper = NULL;

// ══════════════════════════════════════════════════════════════════
// POSITION TRACKING
// ══════════════════════════════════════════════════════════════════
volatile int32_t current_yaw_position = 0;
volatile int32_t current_pitch_position = 0;

// ══════════════════════════════════════════════════════════════════
// SHARED DATA (FreeRTOS)
// ══════════════════════════════════════════════════════════════════
SemaphoreHandle_t dataMutex;
volatile float shared_yaw_error = 0.0f;
volatile bool target_valid = false;
volatile float shared_raw_x_mm = 0;
volatile float shared_raw_y_mm = 0;
volatile bool fusion_active = false;

// ══════════════════════════════════════════════════════════════════
// KALMAN FILTER (Yaw)
// ══════════════════════════════════════════════════════════════════
float kalman_yaw = 0;
float kalman_yaw_velocity = 0;
float kalman_yaw_error = 1.0f;
const float PROCESS_NOISE = 0.01f;
const float MEASUREMENT_NOISE = 3.0f;
unsigned long last_kalman_time = 0;

// ══════════════════════════════════════════════════════════════════
// TRIGGER STATE
// ══════════════════════════════════════════════════════════════════
bool trigger_active = false;
unsigned long trigger_start = 0;
unsigned long last_target_seen = 0;

// ══════════════════════════════════════════════════════════════════
// LD2450 PARSER
// ══════════════════════════════════════════════════════════════════
const byte HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
byte buffer[24];
int idx = 0;
bool receiving = false;

// ══════════════════════════════════════════════════════════════════
// ULTRASONIC FUSION (ONLY FOR X/Y PRECISION)
// ══════════════════════════════════════════════════════════════════════════════════
float applyUltrasonicFusion(float radar_x_mm, float radar_y_mm, float radar_angle_deg) {
  if (fabs(radar_x_mm) > 100) {  // Not centered enough → skip fusion
    fusion_active = false;
    return radar_angle_deg;
  }

  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
  if (duration == 0) {
    fusion_active = false;
    return radar_angle_deg;
  }

  float ultra_y_cm = (duration * 0.0343f) / 2.0f;
  float ultra_y_mm = ultra_y_cm * 10.0f;

  if (ultra_y_cm < 20 || ultra_y_cm > 450) {
    fusion_active = false;
    return radar_angle_deg;
  }

  float fused_x_mm = ultra_y_mm * tan(radar_angle_deg * PI / 180.0f);
  float fused_angle = atan2(fused_x_mm, ultra_y_mm) * (180.0f / PI);

  fusion_active = true;
  Serial.printf("FUSION ACTIVE: Y %.0f→%.0fmm | X %.0f→%.0fmm | Angle %.2f°→%.2f°\n",
                radar_y_mm, ultra_y_mm, radar_x_mm, fused_x_mm, radar_angle_deg, fused_angle);

  return fused_angle;
}

// ══════════════════════════════════════════════════════════════════
// FUNCTION: Kalman Filter (YAW)
// ══════════════════════════════════════════════════════════════════
float applyKalmanFilter(float measured_angle, float dt) {
  // Predict step
  float predicted_angle = kalman_yaw + (kalman_yaw_velocity * dt);
  float predicted_error = kalman_yaw_error + PROCESS_NOISE;
  
  // Update step
  float kalman_gain = predicted_error / (predicted_error + MEASUREMENT_NOISE);
  kalman_yaw = predicted_angle + kalman_gain * (measured_angle - predicted_angle);
  kalman_yaw_velocity = (kalman_yaw - predicted_angle) / dt;
  kalman_yaw_error = (1 - kalman_gain) * predicted_error;
  
  return kalman_yaw;
}

// ══════════════════════════════════════════════════════════════════
// RADAR TASK (Core 0)
// ══════════════════════════════════════════════════════════════════
void radarTask(void *parameter) {
  Serial.println("Radar task running on core " + String(xPortGetCoreID()));

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
    vTaskDelay(1);
  }
}

void parseLD2450Frame() {
  float best_x_mm = 0, best_y_mm = 0;
  float best_dist = 999999.0f;
  bool found = false;

  for (int i = 0; i < 3; i++) {
    byte* g = buffer + i * 8;

        uint16_t raw_x = g[0] | ((g[1] & 0x7F) << 8);
    uint16_t raw_y = g[2] | (g[3] << 8);

    // CORRECTED X SIGN HANDLING
    int16_t x_mm = (g[1] & 0x80) ? -((int16_t)(raw_x & 0x7FFF)) : (raw_x & 0x7FFF);
    int16_t y_mm = (int16_t)(raw_y) - 0x8000;  // this line was already correct

    x_mm *= 10;  // convert 0.1 cm → mm
    y_mm *= 10;

    float dist = sqrtf(x_mm*x_mm + y_mm*y_mm);
    if (dist < best_dist && dist > 300) {
      best_dist = dist;
      best_x_mm = x_mm;
      best_y_mm = y_mm;
      found = true;
    }
  }

  if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
    if (found && best_dist < 6000) {
      float raw_angle = atan2(best_x_mm/10.0f, best_y_mm/10.0f) * (180.0f / PI);

      unsigned long now = millis();
      float dt = (now - last_kalman_time) / 1000.0f;
      if (dt < 0 || dt > 1.0f) dt = 0.1f;
      float kalman_yaw = applyKalmanFilter(raw_angle, dt);
      last_kalman_time = now;

      float final_angle = applyUltrasonicFusion(best_x_mm, best_y_mm, kalman_yaw);

      shared_yaw_error = final_angle;
      shared_raw_x_mm = best_x_mm;
      shared_raw_y_mm = best_y_mm;
      target_valid = true;
      last_target_seen = millis();

      Serial.printf("Target: X=%.1fcm Y=%.1fcm | Angle=%.2f° %s\n",
                    best_x_mm/10.0f, best_y_mm/10.0f, final_angle,
                    fusion_active ? "(FUSED)" : "");
    } else {
      shared_yaw_error = 0;
      target_valid = false;
      fusion_active = false;
    }
    xSemaphoreGive(dataMutex);
  }
}

// ══════════════════════════════════════════════════════════════════
// MOTION TASK (Core 1) — NO PITCH ADJUSTMENT ANYMORE
// ══════════════════════════════════════════════════════════════════
void motionTask(void *parameter) {
  Serial.println("Motion task running on core " + String(xPortGetCoreID()));
  bool at_default = true;

  while (true) {
    float yaw_error = 0;
    bool valid = false;
    unsigned long last_seen = 0;

    if (xSemaphoreTake(dataMutex, 10)) {
      yaw_error = shared_yaw_error;
      valid = target_valid;
      last_seen = last_target_seen;
      xSemaphoreGive(dataMutex);
    }

    // Return to default if no target
    if (!valid || (millis() - last_seen > TARGET_TIMEOUT_MS)) {
      if (!at_default) {
        yaw_stepper->move(-current_yaw_position);
        while (yaw_stepper->isRunning()) vTaskDelay(1);
        current_yaw_position = 0;
        at_default = true;
        trigger_active = false;
        digitalWrite(TRIGGER_PIN, LOW);
      }
      vTaskDelay(100);
      continue;
    }

    float abs_error = fabs(yaw_error);

        // Track with yaw
    if (abs_error > CENTERED_THRESHOLD) {
      // CORRECTED LINE — removed the wrong minus
      int32_t steps = round(yaw_error * YAW_STEPS_PER_DEGREE);

      Serial.printf("Tracking → %.2f° error (%ld steps)\n", yaw_error, steps);

      yaw_stepper->move(steps);
      current_yaw_position += steps;
      at_default = false;

      while (yaw_stepper->isRunning()) vTaskDelay(1);

      trigger_active = false;
      digitalWrite(TRIGGER_PIN, LOW);
    }
    // Yaw locked → FIRE immediately (no pitch change)
    else {
      if (!trigger_active) {
        trigger_active = true;
        trigger_start = millis();
        digitalWrite(TRIGGER_PIN, HIGH);
        Serial.println("YAW LOCKED → FIRING!");
      }
    }

    // Keep trigger high for MIN_HIGH_TIME
    if (trigger_active && (millis() - trigger_start >= MIN_HIGH_TIME)) {
      if (abs_error > 30.0f) {  // lost lock
        trigger_active = false;
        digitalWrite(TRIGGER_PIN, LOW);
      }
    }

    vTaskDelay(10);
  }
}

// ══════════════════════════════════════════════════════════════════
// SETUP & LOOP
// ══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  dataMutex = xSemaphoreCreateMutex();

  engine.init();
  yaw_stepper = engine.stepperConnectToPin(YAW_STEP_PIN);
  yaw_stepper->setDirectionPin(YAW_DIR_PIN);
  yaw_stepper->setSpeedInHz(4000);
  yaw_stepper->setAcceleration(5000);

  pitch_stepper = engine.stepperConnectToPin(PITCH_STEP_PIN);
  pitch_stepper->setDirectionPin(PITCH_DIR_PIN);
  pitch_stepper->setSpeedInHz(2000);
  pitch_stepper->setAcceleration(2500);

  // Startup pitch up 35°
  Serial.printf("Raising pitch %.1f° (%ld steps)...\n", STARTUP_PITCH_ANGLE, STARTUP_PITCH_STEPS);
  pitch_stepper->move(STARTUP_PITCH_STEPS);
  while (pitch_stepper->isRunning()) delay(10);
  current_pitch_position = STARTUP_PITCH_STEPS;
  Serial.println("Startup pitch complete");

  // Serial2.begin(256000, SERIAL_8N1, RADAR_RX, RADAR_TX);
    Serial2.begin(256000, SERIAL_8N1, RADAR_RX, RADAR_TX);
  Serial2.setRxBufferSize(512);
  Serial2.setTimeout(5);  // timeout 5 ms instead of 1000 ms
  delay(200);

  // Force 40 Hz mode (makes radar ~3× faster)
  const byte highSpeed[] = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0x62,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
  Serial2.write(highSpeed, sizeof(highSpeed));
  delay(50);
  Serial.println("LD2450 forced to 40 Hz mode");
  delay(1000);

  Serial.println("TURRET READY — Ultrasonic fusion active, pitch fixed at startup angle");
  Serial.println("════════════════════════════════════════════════════════");

  xTaskCreatePinnedToCore(radarTask, "Radar", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(motionTask, "Motion", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(1000);
}