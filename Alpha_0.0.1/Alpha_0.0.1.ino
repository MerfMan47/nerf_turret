#include "FastAccelStepper.h"

#define PIN_SERIAL_TX 4
#define PIN_SERIAL_RX 5

// Pin Configuration
#define YAW_STEP_PIN        16
#define YAW_DIR_PIN         15

#define PITCH_STEP_PIN        7
#define PITCH_DIR_PIN         6

// radar configuration
const byte HEADER[] = { 0xAA, 0xFF, 0x03, 0x00 };  // Frame header
const byte FOOTER[] = { 0x55, 0xCC };              // Frame footer
const size_t DATA_LENGTH = 24;                     // Data from radar
const size_t GROUP_LENGTH = 8;                     // individual target length
byte buffer[DATA_LENGTH];
byte currentIndex = 0;
bool frameStarted = false;                         // To track if header has been found

// Motor and Gear Configuration
const uint16_t MICROSTEPPING = 8;            // Mircosteps per step
const uint16_t STEPS_PER_REV = 200;          // Motor native steps
const float YAW_GEAR_RATIO = 3;              // 180/60 = 3:1 reduction
const float PITCH_GEAR_RATIO = 64.0 / 21.0;  // 64/21 = 3.048 reduction
const uint32_t SPEED_HZ = 4000;
const uint32_t ACCELERATION = 5000;

// Calculate steps for movement
const uint32_t STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPPING;               // 51200 steps per motor revolution
const uint32_t YAW_STEPS_PER_TABLE_REV = STEPS_PER_MOTOR_REV * YAW_GEAR_RATIO;    // 153600 steps per turntable revolution
const uint32_t PITCH_STEPS_PER_GUN_REV = STEPS_PER_MOTOR_REV * PITCH_GEAR_RATIO;  // 15603 steps per turntable revolution

const float STEPS_PER_YAW_DEGREE = YAW_STEPS_PER_TABLE_REV / 360.0;
const float STEPS_PER_PITCH_DEGREE = PITCH_STEPS_PER_GUN_REV / 360.0;

// Helper function to keep precision
inline int32_t angleToSteps(float angleDegrees, float stepsPerDegree) {
  return (int32_t)round(angleDegrees * stepsPerDegree);
}

// current rotation target
const int32_t destination = 0;

// Setup FastAccelStepper
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *yaw_stepper = NULL;
FastAccelStepper *pitch_stepper = NULL;

// target struct
struct Target {
  int16_t x;            // mm
  int16_t y;            // mm
  int16_t speed;        // cm/s
  int16_t resolution;   // mm distance resolution
  bool active;          // is target valid
};

Target targets[3];

// Function Prototypes
void unpackTarget(byte* data, Target& target);
void processFrame(byte* data, size_t length);
void selectAndTrackTarget();

void setup() {
  Serial.begin(9600);

  // Serial2 for sensor communication on specified pins
  // Syntax: SerialX.begin(baud, config, rxPin, txPin);
  // The third and fourth parameters assign the hardware pins.
  Serial2.begin(256000, SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX);

  engine.init();
  yaw_stepper = engine.stepperConnectToPin(YAW_STEP_PIN);
  pitch_stepper = engine.stepperConnectToPin(PITCH_STEP_PIN);

  // initalize yaw stepper
  if (yaw_stepper) {
    yaw_stepper->setDirectionPin(YAW_DIR_PIN);

    yaw_stepper->setSpeedInHz(SPEED_HZ);
    yaw_stepper->setAcceleration(ACCELERATION);
    Serial.println("Yaw stepper motor engine init complete");
  } else {
    Serial.println("ERROR: Yaw stepper init failed!");
    while (1) delay(1000);  // stall in case of failure
  }

  // initalize pitch stepper
  if (pitch_stepper) {
    pitch_stepper->setDirectionPin(PITCH_DIR_PIN);

    pitch_stepper->setSpeedInHz(SPEED_HZ / 2);
    pitch_stepper->setAcceleration(ACCELERATION / 2);

    Serial.println("Pitch stepper motor engine init complete");
  } else {
    Serial.println("ERROR: Pitch stepper init failed!");
    while (1) delay(1000);  // stall in case of failure
  }

  Serial.println("Startup sequence Complete");
  delay(1000);
}


// --- Main Loop ---
void loop() {
  // get data

  // move to face target
  read_radar_data_frame();
}

// --- Parses the data recieved from radar and sends chunks of data to be processed ---
void read_radar_data_frame() {
  while (Serial2.available()) {  // Check if Serial2 has data
    byte inputByte = Serial2.read();

    // Check if the current index is at the start of a new frame
    if (!frameStarted) {
      if (inputByte == HEADER[currentIndex]) {
        currentIndex++;
        if (currentIndex == sizeof(HEADER)) {
          frameStarted = true; // Header is found
          Serial.print("Header Found");
          currentIndex = 0;
        } else {               // Header not found invalid frame
          currentIndex = 0; 
        }
      }
    } else { 
      // Read data
      if (currentIndex < DATA_LENGTH) {
        buffer[currentIndex++] = inputByte; // put data into buffer
      } else if (currentIndex == DATA_LENGTH) {
        if (inputByte == FOOTER[0]) { // check if footer has been reached
          currentIndex++;
        } else { // footer not found invalid frame
          frameStarted = false;
          currentIndex = 0;
        }
      } else if (currentIndex == DATA_LENGTH + 1) {
        if (inputByte == FOOTER[1]) {
          // frame completed and verified process frame data
          Serial.print("Frame completed");
          processFrame(buffer, DATA_LENGTH);
        }
        // reset for next frame
        frameStarted = false;
        currentIndex = 0;
      }
    }
  }
}

// --- processes frame data and preps data to be unpacked ---
void processFrame(byte* data, size_t length) {
  if (length != DATA_LENGTH) return; // validation check

  for (int i = 0; i < 3; i++) {
    byte group[GROUP_LENGTH];
    memcpy(group, &data[i * GROUP_LENGTH], GROUP_LENGTH);
    unpackTarget(group, targets[i]);
  }
  Serial.print("Processing Finished, calling tracking function");
  selectAndTrackTarget();
}

// --- sorts data directly into target struct array ---
void unpackTarget (byte* data, Target& target) {
  target.x = 0 - (data[0] + (data[1] << 8));
  target.y = (data[2] + (data[3] << 8)) - 32768;
  target.speed = 0 - (data[4] + (data[5] << 8));
  target.resolution = data[6] + (data[7] << 8);

  target.active = !(target.x == 0 && target.y == 0 && target.speed == 0);

  // Debug output
  if (target.active) {
    Serial.print("Target: x=");
    Serial.print(target.x);
    Serial.print(" y=");
    Serial.print(target.y);
    Serial.print(" speed=");
    Serial.print(target.speed);
    Serial.print(" res=");
    Serial.println(target.resolution);
  }
}

// --- finds closest active target and tracks ---
void selectAndTrackTarget() {
  Target* closestTarget = nullptr;
  float minDistance = 999999;

  // finds closest target
  for (int i = 0; i < 3; i++) {
    if (targets[i].active) {
      float distance = targets[i].y;
      if (distance < minDistance && distance > 0) {
        minDistance = distance;
        closestTarget = &targets[i];
      }
    }
  }

  if (closestTarget != nullptr) {
    // float yawAngle = atan2(closestTarget->x, closestTarget->y) * 180.0 / PI; // calculate the angle to rotate

    // int32_t yawSteps = angleToSteps(yawAngle, STEPS_PER_YAW_DEGREE); // convert angle into steps
    // yaw_stepper->moveTo(yawSteps); // rotate

    Serial.print("Tracking target at x=");
    Serial.print(closestTarget->x);
    Serial.print(" y=");
    Serial.println(closestTarget->y);
    
    float yawAngle = atan2(closestTarget->x, closestTarget->y) * 180.0 / PI;
    Serial.print("Yaw angle: ");
    Serial.print(yawAngle);
    Serial.println(" degrees");
    
    int32_t yawSteps = angleToSteps(yawAngle, STEPS_PER_YAW_DEGREE);
    Serial.print("Moving to steps: ");
    Serial.println(yawSteps);
    
    yaw_stepper->moveTo(yawSteps);
    
    Serial.print("Current position: ");
    Serial.println(yaw_stepper->getCurrentPosition());
  } else {
    Serial.println("No active target found");
  }
}















































