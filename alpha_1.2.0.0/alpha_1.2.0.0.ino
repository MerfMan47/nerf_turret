#include "FastAccelStepper.h"
#include <Wire.h>

#define PIN_SERIAL_TX 4
#define PIN_SERIAL_RX 5

// Pin Configuration
#define YAW_STEP_PIN        16
#define YAW_DIR_PIN         15
#define PITCH_STEP_PIN      7
#define PITCH_DIR_PIN       6

// MPU-6050 Configuration
#define MPU_SDA_PIN         17
#define MPU_SCL_PIN         18
#define MPU_ADDR            0x68

// Radar configuration
const byte HEADER[] = { 0xAA, 0xFF, 0x03, 0x00 };
const byte FOOTER[] = { 0x55, 0xCC };
const size_t DATA_LENGTH = 24;
const size_t GROUP_LENGTH = 8;

// Motor and Gear Configuration
const uint16_t MICROSTEPPING = 8;
const uint16_t STEPS_PER_REV = 200;
const float YAW_GEAR_RATIO = 3;
const float PITCH_GEAR_RATIO = 64.0 / 21.0;
const uint32_t SPEED_HZ = 5000;
const uint32_t ACCELERATION = 13000;

// Calculate steps for movement
const uint32_t STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPPING;
const uint32_t YAW_STEPS_PER_TABLE_REV = STEPS_PER_MOTOR_REV * YAW_GEAR_RATIO;
const uint32_t PITCH_STEPS_PER_GUN_REV = STEPS_PER_MOTOR_REV * PITCH_GEAR_RATIO;

const float STEPS_PER_YAW_DEGREE = YAW_STEPS_PER_TABLE_REV / 360.0;
const float STEPS_PER_PITCH_DEGREE = PITCH_STEPS_PER_GUN_REV / 360.0;

// Deadband filter configuration
const int32_t DEADBAND_STEPS = 120;

// Update rate configuration
const unsigned long RADAR_UPDATE_INTERVAL_MS = 1000;
const unsigned long GYRO_UPDATE_INTERVAL_MS = 20; // 50Hz gyro updates

// Auto-level configuration
const float LEVEL_TOLERANCE = 1.5; // Degrees - stop when within this range

// Helper function
inline int32_t angleToSteps(float angleDegrees, float stepsPerDegree) {
  return (int32_t)round(angleDegrees * stepsPerDegree);
}

// FastAccelStepper objects
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *yaw_stepper = NULL;
FastAccelStepper *pitch_stepper = NULL;

// MPU-6050 data
struct MPUData {
  float pitch;
  float roll;
  float yaw;
  float gyroX_offset;
  float gyroY_offset;
  float gyroZ_offset;
  float gyroYaw_rate;    // Current yaw rotation rate
  float gyroPitch_rate;  // Current pitch rotation rate
  unsigned long lastUpdate;
  bool initialized;
  bool isLeveled;
} mpuData = {0, 0, 0, 0, 0, 0, 0, 0, 0, false, false};

// Target struct
struct Target {
  int16_t x;
  int16_t y;
  int16_t speed;
  int16_t resolution;
  bool active;
};

// Simple 1D Kalman Filter
class KalmanFilter {
private:
  float x_est;
  float v_est;
  float P_pos;
  float P_vel;
  float Q_pos;
  float Q_vel;
  float R;
  float dt;
  bool initialized;

public:
  KalmanFilter() : x_est(0), v_est(0), P_pos(1000), P_vel(1000), 
                   Q_pos(10), Q_vel(10), R(50), dt(0.1), initialized(false) {}
  
  void reset() {
    initialized = false;
    P_pos = 1000;
    P_vel = 1000;
  }
  
  float update(float measurement) {
    if (!initialized) {
      x_est = measurement;
      v_est = 0;
      initialized = true;
      return x_est;
    }
    
    float x_pred = x_est + v_est * dt;
    float v_pred = v_est;
    
    float P_pos_pred = P_pos + P_vel * dt * dt + Q_pos;
    float P_vel_pred = P_vel + Q_vel;
    
    float innovation = measurement - x_pred;
    float S = P_pos_pred + R;
    float K_pos = P_pos_pred / S;
    float K_vel = (P_vel * dt) / S;
    
    x_est = x_pred + K_pos * innovation;
    v_est = v_pred + K_vel * innovation;
    
    P_pos = (1 - K_pos) * P_pos_pred;
    P_vel = P_vel_pred - K_vel * P_vel * dt;
    
    return x_est;
  }
  
  float getPosition() { return x_est; }
  float getVelocity() { return v_est; }
};

KalmanFilter kalman_x;
KalmanFilter kalman_y;

// Shared data structure between cores
struct SharedTargetData {
  int32_t radarYawSteps;
  int32_t radarPitchSteps;
  bool hasNewRadarTarget;
  float gimbalPitch;
  float gyroYawRate;
  float gyroPitchRate;
  bool levelingComplete;
  SemaphoreHandle_t mutex;
} sharedData;

struct RadarState {
  byte buffer[DATA_LENGTH];
  byte currentIndex;
  bool frameStarted;
  Target targets[3];
} radarState = { {}, 0, false, {} };

// Function prototypes
void radarProcessingTask(void* parameter);
void stepperControlTask(void* parameter);
void mpuTask(void* parameter);
void read_radar_data_frame();
void processFrame(byte* data, size_t length);
void unpackTarget(byte* data, Target& target);
void selectAndTrackTarget();
bool initMPU6050();
void calibrateMPU();
void readMPU6050();
void autoLevelOnStartup();

void setup() {
  Serial.begin(9600);
  Serial2.begin(256000, SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX);

  Serial.println("\n=== Turret Control System ===");

  // Initialize I2C for MPU-6050
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000);

  // Initialize MPU-6050
  if (!initMPU6050()) {
    Serial.println("ERROR: MPU-6050 init failed!");
    Serial.println("System will continue without gyro features");
  } else {
    Serial.println("MPU-6050 initialized successfully");
    calibrateMPU();
    mpuData.initialized = true;
  }

  // Initialize shared data mutex
  sharedData.mutex = xSemaphoreCreateMutex();
  sharedData.hasNewRadarTarget = false;
  sharedData.radarYawSteps = 0;
  sharedData.radarPitchSteps = 0;
  sharedData.gimbalPitch = 0;
  sharedData.gyroYawRate = 0;
  sharedData.gyroPitchRate = 0;
  sharedData.levelingComplete = false;

  // Initialize FastAccelStepper
  engine.init();
  yaw_stepper = engine.stepperConnectToPin(YAW_STEP_PIN);
  pitch_stepper = engine.stepperConnectToPin(PITCH_STEP_PIN);

  if (yaw_stepper) {
    yaw_stepper->setDirectionPin(YAW_DIR_PIN);
    yaw_stepper->setSpeedInHz(SPEED_HZ);
    yaw_stepper->setAcceleration(ACCELERATION);
    Serial.println("Yaw stepper initialized");
  } else {
    Serial.println("ERROR: Yaw stepper init failed!");
    while (1) delay(1000);
  }

  if (pitch_stepper) {
    pitch_stepper->setDirectionPin(PITCH_DIR_PIN);
    pitch_stepper->setSpeedInHz(SPEED_HZ / 2);
    pitch_stepper->setAcceleration(ACCELERATION / 2);
    Serial.println("Pitch stepper initialized");
  } else {
    Serial.println("ERROR: Pitch stepper init failed!");
    while (1) delay(1000);
  }

  // Create tasks on different cores FIRST
  xTaskCreatePinnedToCore(radarProcessingTask, "RadarTask", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(stepperControlTask, "StepperTask", 4096, NULL, 1, NULL, 1);
  
  if (mpuData.initialized) {
    xTaskCreatePinnedToCore(mpuTask, "MPUTask", 4096, NULL, 2, NULL, 0);
    
    // Wait longer for MPU task to start and stabilize
    delay(1000);
    
    // Auto-level on startup
    autoLevelOnStartup();
  } else {
    // No MPU, skip leveling
    sharedData.levelingComplete = true;
  }

  Serial.println("\n=== Startup Complete ===");
  Serial.println("Radar tracking with gyro-enhanced smoothing enabled");
  Serial.println("All tasks running");
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// ==================== MPU-6050 FUNCTIONS ====================

bool initMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  delay(100);
  return true;
}

void calibrateMPU() {
  Serial.println("\nCalibrating gyroscope...");
  Serial.println("Keep turret STILL for 3 seconds!");
  delay(3000);
  
  const int samples = 500;
  long sumX = 0, sumY = 0, sumZ = 0;
  
  for(int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();
    
    sumX += gx;
    sumY += gy;
    sumZ += gz;
    delay(10);
    
    if(i % 100 == 0) Serial.print(".");
  }
  
  mpuData.gyroX_offset = (sumX / samples) / 131.0;
  mpuData.gyroY_offset = (sumY / samples) / 131.0;
  mpuData.gyroZ_offset = (sumZ / samples) / 131.0;
  
  Serial.println(" Done!");
  Serial.println("Gyro calibration complete");
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  int16_t accelX = Wire.read() << 8 | Wire.read();
  int16_t accelY = Wire.read() << 8 | Wire.read();
  int16_t accelZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();
  
  // Convert to g and deg/s
  float accelX_g = accelX / 16384.0;
  float accelY_g = accelY / 16384.0;
  float accelZ_g = accelZ / 16384.0;
  
  float gyroX_dps = (gyroX / 131.0) - mpuData.gyroX_offset;
  float gyroY_dps = (gyroY / 131.0) - mpuData.gyroY_offset;
  float gyroZ_dps = (gyroZ / 131.0) - mpuData.gyroZ_offset;
  
  // Calculate time delta
  unsigned long currentTime = millis();
  float dt = (currentTime - mpuData.lastUpdate) / 1000.0;
  mpuData.lastUpdate = currentTime;
  
  if (dt > 0.1) dt = 0.02; // Prevent large jumps on first read
  
  // Calculate angles from accelerometer
  float accel_pitch = atan2(accelY_g, sqrt(accelX_g*accelX_g + accelZ_g*accelZ_g)) * 180.0/PI;
  float accel_roll = atan2(-accelX_g, accelZ_g) * 180.0/PI;
  
  // Integrate gyroscope
  mpuData.pitch += gyroX_dps * dt;
  mpuData.roll += gyroY_dps * dt;
  mpuData.yaw += gyroZ_dps * dt;
  
  // Complementary filter
  const float alpha = 0.98;
  mpuData.pitch = alpha * mpuData.pitch + (1 - alpha) * accel_pitch;
  mpuData.roll = alpha * mpuData.roll + (1 - alpha) * accel_roll;
  
  // Store current rotation rates for enhanced tracking
  // Remap based on your sensor orientation: gimbal pitch = sensor yaw
  mpuData.gyroPitch_rate = gyroZ_dps;  // Your pitch = sensor's yaw axis
  mpuData.gyroYaw_rate = gyroY_dps;    // Your yaw = sensor's roll axis
}

void autoLevelOnStartup() {
  Serial.println("\n=== Auto-Leveling Pitch ===");
  
  // Wait for MPU task to populate data
  delay(500);
  
  // Get current pitch from shared data (updated by MPU task)
  float current_pitch = 0;
  if (xSemaphoreTake(sharedData.mutex, portMAX_DELAY) == pdTRUE) {
    current_pitch = sharedData.gimbalPitch;
    xSemaphoreGive(sharedData.mutex);
  }
  
  Serial.print("Current pitch: ");
  Serial.print(current_pitch, 1);
  Serial.println("°");
  
  if (abs(current_pitch) < LEVEL_TOLERANCE) {
    Serial.println("Already level!");
    mpuData.isLeveled = true;
    sharedData.levelingComplete = true;
    return;
  }
  
  // Calculate correction needed
  float correction_angle = -current_pitch;
  int32_t correction_steps = angleToSteps(correction_angle, STEPS_PER_PITCH_DEGREE);
  
  Serial.print("Leveling: ");
  Serial.print(correction_angle, 1);
  Serial.print("° (");
  Serial.print(correction_steps);
  Serial.println(" steps)");
  
  // Move to level
  pitch_stepper->moveTo(correction_steps);
  
  // Wait for movement to complete
  while(pitch_stepper->isRunning()) {
    delay(50);
  }
  
  // Verify level
  delay(500);
  float final_pitch = 0;
  if (xSemaphoreTake(sharedData.mutex, portMAX_DELAY) == pdTRUE) {
    final_pitch = sharedData.gimbalPitch;
    xSemaphoreGive(sharedData.mutex);
  }
  Serial.print("Final pitch: ");
  Serial.print(final_pitch, 1);
  Serial.println("°");
  
  if (abs(final_pitch) < LEVEL_TOLERANCE) {
    Serial.println("✓ Level achieved!");
    mpuData.isLeveled = true;
  } else {
    Serial.println("⚠ Close enough, continuing...");
    mpuData.isLeveled = true;
  }
  
  // Reset position to zero for relative tracking
  pitch_stepper->setCurrentPosition(0);
  
  sharedData.levelingComplete = true;
}

void mpuTask(void* parameter) {
  Serial.println("MPU task started on Core 0");
  mpuData.lastUpdate = millis();
  
  while (true) {
    readMPU6050();
    
    // Update shared data
    if (xSemaphoreTake(sharedData.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
      sharedData.gimbalPitch = mpuData.yaw;        // Remapped
      sharedData.gyroYawRate = mpuData.gyroYaw_rate;
      sharedData.gyroPitchRate = mpuData.gyroPitch_rate;
      xSemaphoreGive(sharedData.mutex);
    }
    
    vTaskDelay(GYRO_UPDATE_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

// ==================== CORE 0: RADAR PROCESSING ====================

void radarProcessingTask(void* parameter) {
  Serial.println("Radar processing task started on Core 0");
  
  while (true) {
    read_radar_data_frame();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void read_radar_data_frame() {
  while (Serial2.available()) {
    byte inputByte = Serial2.read();

    if (!radarState.frameStarted) {
      if (inputByte == HEADER[radarState.currentIndex]) {
        radarState.currentIndex++;
        if (radarState.currentIndex == sizeof(HEADER)) {
          radarState.frameStarted = true;
          radarState.currentIndex = 0;
        }
      } else {
        radarState.currentIndex = 0;
      }
    } else {
      if (radarState.currentIndex < DATA_LENGTH) {
        radarState.buffer[radarState.currentIndex++] = inputByte;
      } else if (radarState.currentIndex == DATA_LENGTH) {
        if (inputByte == FOOTER[0]) {
          radarState.currentIndex++;
        } else {
          radarState.frameStarted = false;
          radarState.currentIndex = 0;
        }
      } else if (radarState.currentIndex == DATA_LENGTH + 1) {
        if (inputByte == FOOTER[1]) {
          processFrame(radarState.buffer, DATA_LENGTH);
        }
        radarState.frameStarted = false;
        radarState.currentIndex = 0;
      }
    }
  }
}

void processFrame(byte* data, size_t length) {
  if (length != DATA_LENGTH) return;

  for (int i = 0; i < 3; i++) {
    byte group[GROUP_LENGTH];
    memcpy(group, &data[i * GROUP_LENGTH], GROUP_LENGTH);
    unpackTarget(group, radarState.targets[i]);
  }

  selectAndTrackTarget();
}

void unpackTarget(byte* data, Target& target) {
  target.x = 0 - (data[0] + (data[1] << 8));
  target.y = (data[2] + (data[3] << 8)) - 32768;
  target.speed = 0 - (data[4] + (data[5] << 8));
  target.resolution = data[6] + (data[7] << 8);

  target.active = !(target.x == 0 && target.y == 0 && target.speed == 0);
}

void selectAndTrackTarget() {
  static int32_t lastTargetSteps = 0;
  static unsigned long lastUpdateTime = 0;
  static int lastTargetIndex = -1;
  
  // Wait for leveling to complete
  if (!sharedData.levelingComplete) {
    return;
  }
  
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime < RADAR_UPDATE_INTERVAL_MS) {
    return;
  }
  
  Target* closestTarget = nullptr;
  float minDistance = 999999;
  int closestIndex = -1;

  for (int i = 0; i < 3; i++) {
    if (radarState.targets[i].active) {
      float distance = radarState.targets[i].y;
      if (distance < minDistance && distance > 0) {
        minDistance = distance;
        closestTarget = &radarState.targets[i];
        closestIndex = i;
      }
    }
  }

  if (closestTarget != nullptr) {
    if (closestIndex != lastTargetIndex) {
      kalman_x.reset();
      kalman_y.reset();
      lastTargetIndex = closestIndex;
      Serial.println("Target switched");
    }
    
    float filtered_x = kalman_x.update(closestTarget->x);
    float filtered_y = kalman_y.update(closestTarget->y);
    
    float yawAngle = atan2(filtered_x, filtered_y) * 180.0 / PI;
    int32_t yawSteps = angleToSteps(yawAngle, STEPS_PER_YAW_DEGREE);

    int32_t pitchSteps = 0;

    if (abs(yawSteps - lastTargetSteps) > DEADBAND_STEPS) {
      lastTargetSteps = yawSteps;
      lastUpdateTime = currentTime;
      
      if (xSemaphoreTake(sharedData.mutex, portMAX_DELAY) == pdTRUE) {
        sharedData.radarYawSteps = yawSteps;
        sharedData.radarPitchSteps = pitchSteps;
        sharedData.hasNewRadarTarget = true;
        xSemaphoreGive(sharedData.mutex);
      }

      Serial.print("Tracking T");
      Serial.print(closestIndex);
      Serial.print(" -> ");
      Serial.print(yawAngle, 1);
      Serial.println("°");
    } else {
      lastUpdateTime = currentTime;
    }
  } else {
    if (lastTargetIndex != -1) {
      kalman_x.reset();
      kalman_y.reset();
      lastTargetIndex = -1;
      Serial.println("Target lost");
    }
    lastUpdateTime = currentTime;
  }
}

// ==================== CORE 1: STEPPER CONTROL WITH GYRO ENHANCEMENT ====================

void stepperControlTask(void* parameter) {
  Serial.println("Stepper control task started on Core 1");
  
  int32_t lastRadarYawSteps = 0;
  unsigned long lastGyroUpdateTime = millis();
  
  while (true) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastGyroUpdateTime) / 1000.0;
    lastGyroUpdateTime = currentTime;
    
    if (xSemaphoreTake(sharedData.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
      
      // Handle new radar target
      if (sharedData.hasNewRadarTarget) {
        lastRadarYawSteps = sharedData.radarYawSteps;
        
        yaw_stepper->moveTo(sharedData.radarYawSteps);
        pitch_stepper->moveTo(sharedData.radarPitchSteps);
        
        sharedData.hasNewRadarTarget = false;
        
        Serial.print("Moving to: ");
        Serial.print(sharedData.radarYawSteps);
        Serial.println(" steps");
      }
      // Gyro-enhanced tracking: smooth movement between radar updates
      else if (sharedData.levelingComplete && !yaw_stepper->isRunning()) {
        // Use gyro data to predict where target might be moving
        // This helps smooth out the stepwise radar updates
        float gyroYawRate = sharedData.gyroYawRate;
        
        // If gimbal is rotating (following target), add small predictive adjustment
        if (abs(gyroYawRate) > 1.0) { // Ignore tiny noise
          // Small correction based on rotation rate
          float predictive_angle = gyroYawRate * dt * 0.3; // 30% prediction factor
          int32_t predictive_steps = angleToSteps(predictive_angle, STEPS_PER_YAW_DEGREE);
          
          if (abs(predictive_steps) > 5) { // Only if meaningful
            int32_t newTarget = yaw_stepper->getCurrentPosition() + predictive_steps;
            yaw_stepper->moveTo(newTarget);
          }
        }
      }
      
      xSemaphoreGive(sharedData.mutex);
    }
    
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}