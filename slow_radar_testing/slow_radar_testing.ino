#include "FastAccelStepper.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
// ==================== 1. Pin Configuration ====================
#define PIN_SERIAL_TX   4
#define PIN_SERIAL_RX   5
#define YAW_STEP_PIN    16
#define YAW_DIR_PIN     15
#define MPU_SDA_PIN     17
#define MPU_SCL_PIN     18
#define MPU_ADDR        0x68
// ==================== 2. RD03D Radar Constants ====================
const uint8_t RADAR_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t RADAR_FOOTER[] = {0x55, 0xCC};
const uint8_t MULTI_TARGET_MODE[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};
// ==================== 3. Motor Constants ====================
const uint16_t MICROSTEPPING = 8;
const uint16_t STEPS_PER_REV = 200;
const float    YAW_GEAR_RATIO = 3.0;
const uint32_t SPEED_HZ      = 1500;  // Reduced for less overshoot
const uint32_t ACCELERATION  = 4000;  // Reduced for damping
const float STEPS_PER_YAW_DEGREE = (STEPS_PER_REV * MICROSTEPPING * YAW_GEAR_RATIO) / 360.0;
// ==================== 4. Correction & Tracking Constants ====================
#define RADAR_OFFSET_MM            145.0f
#define VELOCITY_THRESHOLD_CM_S    35.0f  // Increased to filter residuals
#define INDUCED_VELOCITY_SIGN      -1.0f
#define DEBUG_VELOCITY_CORRECTION  false
// ANTI-OSCILLATION SETTINGS
#define SMOOTHING_ALPHA            0.2f   // Lowered for more damping
#define PREDICTION_TIME_MS         100.0f // Reduced to minimize overshoot
#define MIN_MOVE_THRESHOLD_DEG     3.0f   // Increased to ignore small errors
#define TARGET_LOST_TIMEOUT_MS     1000   // Increased to prevent quick stops
#define MAX_RESOLUTION_MM          300    // NEW: Ignore strong reflectors (metal)
#define ANGULAR_VEL_ALPHA          0.5f   // NEW: Smoothing for angular vel
// ==================== 5. Stepper Engine ====================
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *yaw_stepper = NULL;
// ==================== 6. Shared Data Between Tasks ====================
struct SharedData {
  int32_t targetYawSteps;
  float   targetAngleDeg;
  float   targetAngularVel;
  bool    hasNewTarget;
  uint32_t lastTargetTime;
  float   gyroYawRate;
  SemaphoreHandle_t mutex;
} sharedData;
// ==================== 7. Radar Buffer and Targets ====================
uint8_t radarBuffer[64];
uint8_t bufferIndex = 0;
bool    frameStarted = false;
struct Target {
  int16_t x;
  int16_t y;
  int16_t speed;
  uint16_t resolution;
  bool active;
};
// ==================== 8. Tracking State ====================
float smoothedAngleDeg = 0.0f;
float smoothedAngularVel = 0.0f;  // NEW
float previousTargetAngle = 0.0f;
uint32_t previousTargetTime = 0;
// Helpers
inline int32_t angleToSteps(float angleDeg) {
  return (int32_t)round(angleDeg * STEPS_PER_YAW_DEGREE);
}
inline float stepsToDegrees(int32_t steps) {
  return (float)steps / STEPS_PER_YAW_DEGREE;
}
// ==================== MPU Functions ====================
bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  if (error != 0) return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);
  return true;
}
float gyroZ_offset = 0.0f;
void calibrateMPU() {
  Serial.println("[MPU] Calibrating gyro - keep still...");
  const int samples = 500;
  long sumZ = 0;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    Wire.read(); Wire.read();
    Wire.read(); Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();
    sumZ += gz;
    delay(2);
  }
  gyroZ_offset = (sumZ / samples) / 131.0f;
  Serial.print("[MPU] Offset: ");
  Serial.print(gyroZ_offset, 2);
  Serial.println(" deg/s");
}
void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  Wire.read(); Wire.read();
  Wire.read(); Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();
  float gyroZ_dps = (gz / 131.0f) - gyroZ_offset;
  // NOTE: If sign is wrong, add: gyroZ_dps *= -1; (test with manual rotation)
  if (xSemaphoreTake(sharedData.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
    sharedData.gyroYawRate = gyroZ_dps;
    xSemaphoreGive(sharedData.mutex);
  }
}
// ==================== TASK 3: MPU Sensor (Core 0) ====================
void mpuTask(void *parameter) {
  Serial.println("[MPU Task] Started on Core 0");
  while (true) {
    readMPU6050();
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}
// ==================== TASK 1: Radar Processing (Core 0) ====================
void radarTask(void *parameter) {
  Serial.println("[Radar Task] Started on Core 0");
  static uint32_t last_debug = 0;
  while (true) {
    while (Serial2.available()) {
      uint8_t byte = Serial2.read();
      if (!frameStarted) {
        radarBuffer[bufferIndex++] = byte;
        if (bufferIndex >= 4) {
          if (radarBuffer[bufferIndex-4] == RADAR_HEADER[0] &&
              radarBuffer[bufferIndex-3] == RADAR_HEADER[1] &&
              radarBuffer[bufferIndex-2] == RADAR_HEADER[2] &&
              radarBuffer[bufferIndex-1] == RADAR_HEADER[3]) {
            frameStarted = true;
            bufferIndex = 4;
          } else {
            memmove(radarBuffer, radarBuffer + 1, 3);
            bufferIndex = 3;
          }
        }
      } else {
        radarBuffer[bufferIndex++] = byte;
        if (bufferIndex >= 6 &&
            radarBuffer[bufferIndex-2] == RADAR_FOOTER[0] &&
            radarBuffer[bufferIndex-1] == RADAR_FOOTER[1]) {
          Target targets[3];
          for (int i = 0; i < 3; i++) {
            uint8_t* t = &radarBuffer[4 + (i * 8)];
            uint16_t x_raw = t[0] | (t[1] << 8);
            uint16_t y_raw = t[2] | (t[3] << 8);
            uint16_t speed_raw = t[4] | (t[5] << 8);
            targets[i].resolution = t[6] | (t[7] << 8);
            targets[i].x = -(int16_t)x_raw;
            targets[i].y = (int16_t)y_raw - 32768;
            targets[i].speed = -(int16_t)speed_raw;
            targets[i].active = (x_raw != 0 || y_raw != 32768 || speed_raw != 0 || targets[i].resolution != 0);
          }
          float gyroYawRate = 0.0f;
          if (xSemaphoreTake(sharedData.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
            gyroYawRate = sharedData.gyroYawRate;
            xSemaphoreGive(sharedData.mutex);
          }
          float omega = gyroYawRate * (M_PI / 180.0f);
          if (DEBUG_VELOCITY_CORRECTION && (millis() - last_debug > 500)) {
            Serial.print("[Status] Gyro: ");
            Serial.print(gyroYawRate, 2);
            Serial.println(" deg/s");
            last_debug = millis();
          }
          Target* closest = nullptr;
          float minDistance = 999999.0f;
          for (int i = 0; i < 3; i++) {
            Target& tgt = targets[i];
            if (!tgt.active) continue;
            float x_cm = (float)tgt.x / 10.0f;
            float y_cm = (float)tgt.y / 10.0f;
            float r = sqrt(x_cm * x_cm + y_cm * y_cm);
            if (r <= 0) continue;
            float v_radial_induced = INDUCED_VELOCITY_SIGN * omega * x_cm;
            float corrected_speed = (float)tgt.speed - v_radial_induced;
            if (DEBUG_VELOCITY_CORRECTION) {
              Serial.print("[T");
              Serial.print(i);
              Serial.print("] Raw=");
              Serial.print(tgt.speed);
              Serial.print(" Corr=");
              Serial.print(corrected_speed, 1);
              Serial.println();
            }
            // NEW: Filter strong reflectors (metal)
            if (tgt.resolution > MAX_RESOLUTION_MM || fabs(corrected_speed) < VELOCITY_THRESHOLD_CM_S) {
              continue;
            }
            float dist = y_cm;
            if (dist < minDistance && dist > 0) {
              minDistance = dist;
              closest = &tgt;
            }
          }
          if (closest != nullptr) {
            uint32_t currentTime = millis();
            float raw_angle_deg = atan2((float)closest->x, (float)closest->y) * 180.0f / M_PI;
            // Calculate angular velocity
            float angular_velocity = 0.0f;
            if (previousTargetTime > 0) {
              float dt = (currentTime - previousTargetTime) / 1000.0f;
              if (dt > 0.001f && dt < 0.5f) {
                float angle_diff = raw_angle_deg - previousTargetAngle;
                if (angle_diff > 180.0f) angle_diff -= 360.0f;
                if (angle_diff < -180.0f) angle_diff += 360.0f;
                angular_velocity = angle_diff / dt;
              }
            }
            // NEW: Smooth angular velocity
            smoothedAngularVel = (angular_velocity * ANGULAR_VEL_ALPHA) + (smoothedAngularVel * (1.0f - ANGULAR_VEL_ALPHA));
            // Predict
            float prediction_seconds = PREDICTION_TIME_MS / 1000.0f;
            float predicted_angle = raw_angle_deg + (smoothedAngularVel * prediction_seconds);
            previousTargetAngle = raw_angle_deg;
            previousTargetTime = currentTime;
            if (xSemaphoreTake(sharedData.mutex, portMAX_DELAY) == pdTRUE) {
              sharedData.targetAngleDeg = predicted_angle;
              sharedData.targetAngularVel = smoothedAngularVel;
              sharedData.hasNewTarget = true;
              sharedData.lastTargetTime = currentTime;
              xSemaphoreGive(sharedData.mutex);
            }
            Serial.print("[Radar] Target at ");
            Serial.print(raw_angle_deg, 1);
            Serial.print("° (vel: ");
            Serial.print(smoothedAngularVel, 1);
            Serial.print(" deg/s, predicted: ");
            Serial.print(predicted_angle, 1);
            Serial.print("°, dist: ");
            Serial.print(minDistance, 0);
            Serial.println(" cm)");
          } else {
            if (DEBUG_VELOCITY_CORRECTION) {
              Serial.println("[Radar] No moving targets");
            }
          }
          bufferIndex = 0;
          frameStarted = false;
        }
        if (bufferIndex >= sizeof(radarBuffer)) {
          bufferIndex = 0;
          frameStarted = false;
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
// ==================== TASK 2: Stepper Control (Core 1) ====================
void stepperTask(void *parameter) {
  Serial.println("[Stepper Task] Started on Core 1");
  while (true) {
    bool hasTarget = false;
    float targetAngle = 0.0f;
    uint32_t lastTargetTime = 0;
    if (xSemaphoreTake(sharedData.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
      if (sharedData.hasNewTarget) {
        targetAngle = sharedData.targetAngleDeg;
        lastTargetTime = sharedData.lastTargetTime;
        sharedData.hasNewTarget = false;
        hasTarget = true;
      }
      xSemaphoreGive(sharedData.mutex);
    }
    if (hasTarget && (millis() - lastTargetTime) < TARGET_LOST_TIMEOUT_MS) {
      if (smoothedAngleDeg == 0.0f && previousTargetTime == 0) {
        smoothedAngleDeg = targetAngle;
      } else {
        float angle_diff = targetAngle - smoothedAngleDeg;
        if (angle_diff > 180.0f) angle_diff -= 360.0f;
        if (angle_diff < -180.0f) angle_diff += 360.0f;
        smoothedAngleDeg += angle_diff * SMOOTHING_ALPHA;
        if (smoothedAngleDeg > 180.0f) smoothedAngleDeg -= 360.0f;
        if (smoothedAngleDeg < -180.0f) smoothedAngleDeg += 360.0f;
      }
      int32_t currentPos = yaw_stepper->getCurrentPosition();
      float currentAngle = stepsToDegrees(currentPos);
      float angle_error = smoothedAngleDeg - currentAngle;
      if (angle_error > 180.0f) angle_error -= 360.0f;
      if (angle_error < -180.0f) angle_error += 360.0f;
      // NEW: Damping using gyro feedback
      float gyroRate = 0.0f;
      if (xSemaphoreTake(sharedData.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
        gyroRate = sharedData.gyroYawRate;
        xSemaphoreGive(sharedData.mutex);
      }
      float damped_error = angle_error - (gyroRate * 0.1f);  // Simple derivative damping
      if (fabs(damped_error) > MIN_MOVE_THRESHOLD_DEG) {
        int32_t targetSteps = angleToSteps(smoothedAngleDeg);
        yaw_stepper->moveTo(targetSteps);
        Serial.print("[Stepper] Tracking → ");
        Serial.print(smoothedAngleDeg, 1);
        Serial.print("° (error: ");
        Serial.print(damped_error, 1);
        Serial.println("°)");
      }
    } else if (millis() - lastTargetTime > TARGET_LOST_TIMEOUT_MS && lastTargetTime > 0) {
      Serial.println("[Stepper] Target lost - stopping");
      lastTargetTime = 0;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
// ==================== setup() ====================
void setup() {
  Serial.begin(115200);
  Serial2.begin(256000, SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX);
  Serial.println("\n=== Smooth Predictive Turret Tracking ===");
  Serial2.write(MULTI_TARGET_MODE, sizeof(MULTI_TARGET_MODE));
  Serial.println("[Setup] Radar configured");
  delay(200);
  Serial2.flush();
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000);
  if (!initMPU6050()) {
    Serial.println("[ERROR] MPU init failed!");
  } else {
    calibrateMPU();
    Serial.println("[Setup] MPU ready");
  }
  engine.init();
  yaw_stepper = engine.stepperConnectToPin(YAW_STEP_PIN);
  if (yaw_stepper) {
    yaw_stepper->setDirectionPin(YAW_DIR_PIN);
    yaw_stepper->setSpeedInHz(SPEED_HZ);
    yaw_stepper->setAcceleration(ACCELERATION);
    yaw_stepper->setCurrentPosition(0);
    Serial.println("[Setup] Stepper ready");
  } else {
    Serial.println("[ERROR] Stepper init failed!");
    while (true) delay(1000);
  }
  sharedData.mutex = xSemaphoreCreateMutex();
  sharedData.hasNewTarget = false;
  sharedData.targetYawSteps = 0;
  sharedData.targetAngleDeg = 0.0f;
  sharedData.targetAngularVel = 0.0f;
  sharedData.lastTargetTime = 0;
  sharedData.gyroYawRate = 0.0f;
  xTaskCreatePinnedToCore(mpuTask,    "MPUTask",    4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(radarTask,  "RadarTask",  8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(stepperTask,"StepperTask",4096, NULL, 2, NULL, 1);
  Serial.println("\n=== Updated Tuning ===");
  Serial.println("SMOOTHING_ALPHA: 0.2");
  Serial.println("PREDICTION_TIME_MS: 100");
  Serial.println("MIN_MOVE_THRESHOLD_DEG: 3.0");
  Serial.println("VELOCITY_THRESHOLD_CM_S: 35.0");
  Serial.println("MAX_RESOLUTION_MM: 300 (for metal filter)");
  Serial.println("=========================\n");
}
void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}