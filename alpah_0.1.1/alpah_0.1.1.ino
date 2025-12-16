#include "FastAccelStepper.h"

#define PIN_SERIAL_TX 4
#define PIN_SERIAL_RX 5

// Pin Configuration
#define YAW_STEP_PIN        16
#define YAW_DIR_PIN         15
#define PITCH_STEP_PIN      7
#define PITCH_DIR_PIN       6

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
const uint32_t ACCELERATION = 15000; // was 6000

// Calculate steps for movement
const uint32_t STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPPING;
const uint32_t YAW_STEPS_PER_TABLE_REV = STEPS_PER_MOTOR_REV * YAW_GEAR_RATIO;
const uint32_t PITCH_STEPS_PER_GUN_REV = STEPS_PER_MOTOR_REV * PITCH_GEAR_RATIO;

const float STEPS_PER_YAW_DEGREE = YAW_STEPS_PER_TABLE_REV / 360.0;
const float STEPS_PER_PITCH_DEGREE = PITCH_STEPS_PER_GUN_REV / 360.0;

// Deadband filter configuration
const int32_t DEADBAND_STEPS = 120; // Only update if change is > 50 steps (~0.4°)

// Helper function
inline int32_t angleToSteps(float angleDegrees, float stepsPerDegree) {
  return (int32_t)round(angleDegrees * stepsPerDegree);
}

// FastAccelStepper objects (used only by Core 1)
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *yaw_stepper = NULL;
FastAccelStepper *pitch_stepper = NULL;

// Target struct
struct Target {
  int16_t x;
  int16_t y;
  int16_t speed;
  int16_t resolution;
  bool active;
};

// Shared data structure between cores
struct SharedTargetData {
  int32_t targetYawSteps;
  int32_t targetPitchSteps;
  bool hasNewTarget;
  SemaphoreHandle_t mutex;
} sharedData;

// Core 0 data (radar processing)
struct RadarState {
  byte buffer[DATA_LENGTH];
  byte currentIndex;
  bool frameStarted;
  Target targets[3];
} radarState = { {}, 0, false, {} };

// Function prototypes
void radarProcessingTask(void* parameter);
void stepperControlTask(void* parameter);
void read_radar_data_frame();
void processFrame(byte* data, size_t length);
void unpackTarget(byte* data, Target& target);
void selectAndTrackTarget();

void setup() {
  Serial.begin(9600);
  Serial2.begin(256000, SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX);

  // Initialize shared data mutex
  sharedData.mutex = xSemaphoreCreateMutex();
  sharedData.hasNewTarget = false;
  sharedData.targetYawSteps = 0;
  sharedData.targetPitchSteps = 0;

  // Initialize FastAccelStepper on Core 1
  engine.init();
  yaw_stepper = engine.stepperConnectToPin(YAW_STEP_PIN);
  pitch_stepper = engine.stepperConnectToPin(PITCH_STEP_PIN);

  if (yaw_stepper) {
    yaw_stepper->setDirectionPin(YAW_DIR_PIN);
    yaw_stepper->setSpeedInHz(SPEED_HZ);
    yaw_stepper->setAcceleration(ACCELERATION);
    Serial.println("Yaw stepper motor engine init complete");
  } else {
    Serial.println("ERROR: Yaw stepper init failed!");
    while (1) delay(1000);
  }

  if (pitch_stepper) {
    pitch_stepper->setDirectionPin(PITCH_DIR_PIN);
    pitch_stepper->setSpeedInHz(SPEED_HZ / 2);
    pitch_stepper->setAcceleration(ACCELERATION / 2);
    Serial.println("Pitch stepper motor engine init complete");
  } else {
    Serial.println("ERROR: Pitch stepper init failed!");
    while (1) delay(1000);
  }

  Serial.println("Startup sequence Complete");
  delay(1000);

  // Create tasks on different cores
  xTaskCreatePinnedToCore(
    radarProcessingTask,
    "RadarTask",
    8192,
    NULL,
    2,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    stepperControlTask,
    "StepperTask",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  Serial.println("Tasks created on separate cores");
}

void loop() {
  // Empty - all work done in tasks
  vTaskDelay(1000 / portTICK_PERIOD_MS);
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
  static int32_t lastTargetSteps = 0; // Remember last commanded position
  
  Target* closestTarget = nullptr;
  float minDistance = 999999;

  // Find closest active target
  for (int i = 0; i < 3; i++) {
    if (radarState.targets[i].active) {
      float distance = radarState.targets[i].y;
      if (distance < minDistance && distance > 0) {
        minDistance = distance;
        closestTarget = &radarState.targets[i];
      }
    }
  }

  if (closestTarget != nullptr) {
    // Calculate angles
    float yawAngle = atan2(closestTarget->x, closestTarget->y) * 180.0 / PI;
    int32_t yawSteps = angleToSteps(yawAngle, STEPS_PER_YAW_DEGREE);

    int32_t pitchSteps = 0;

    // Deadband filter: only update if change is significant
    if (abs(yawSteps - lastTargetSteps) > DEADBAND_STEPS) {
      lastTargetSteps = yawSteps;
      
      // Thread-safe update of shared data
      if (xSemaphoreTake(sharedData.mutex, portMAX_DELAY) == pdTRUE) {
        sharedData.targetYawSteps = yawSteps;
        sharedData.targetPitchSteps = pitchSteps;
        sharedData.hasNewTarget = true;
        xSemaphoreGive(sharedData.mutex);
      }

      // Debug output
      Serial.print("Target: x=");
      Serial.print(closestTarget->x);
      Serial.print(" y=");
      Serial.print(closestTarget->y);
      Serial.print(" -> Yaw: ");
      Serial.print(yawAngle);
      Serial.print("° (");
      Serial.print(yawSteps);
      Serial.print(" steps) | Change: ");
      Serial.println(abs(yawSteps - lastTargetSteps));
    }
  }
}

// ==================== CORE 1: STEPPER CONTROL ====================

void stepperControlTask(void* parameter) {
  Serial.println("Stepper control task started on Core 1");
  
  while (true) {
    if (xSemaphoreTake(sharedData.mutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
      if (sharedData.hasNewTarget) {
        yaw_stepper->moveTo(sharedData.targetYawSteps);
        pitch_stepper->moveTo(sharedData.targetPitchSteps);
        
        sharedData.hasNewTarget = false;
        
        Serial.print("Core 1: Moving to yaw steps: ");
        Serial.print(sharedData.targetYawSteps);
        Serial.print(" | Current: ");
        Serial.println(yaw_stepper->getCurrentPosition());
      }
      xSemaphoreGive(sharedData.mutex);
    }
    
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}