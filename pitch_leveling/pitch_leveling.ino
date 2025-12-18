#include "FastAccelStepper.h"
#include <Wire.h>

// Stepper pins
#define PITCH_STEP_PIN      7
#define PITCH_DIR_PIN       6

// MPU-6050 Configuration
#define MPU_SDA_PIN         17
#define MPU_SCL_PIN         18
#define MPU_ADDR            0x68

// Pitch settings
#define TARGET_PITCH_DEG 90.0
#define ANGLE_TOLERANCE 0.8

// Motor and Gear Configuration
const uint16_t MICROSTEPPING = 8;
const uint16_t STEPS_PER_REV = 200;
const float PITCH_GEAR_RATIO = 64.0 / 21.0;
const uint32_t SPEED_HZ = 2500;
const uint32_t ACCELERATION = 6500;

// Calculate steps per degree
const uint32_t STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPPING;
const uint32_t PITCH_STEPS_PER_GUN_REV = STEPS_PER_MOTOR_REV * PITCH_GEAR_RATIO;
const float STEPS_PER_PITCH_DEGREE = PITCH_STEPS_PER_GUN_REV / 360.0;

// Leveling tolerance
const float LEVEL_TOLERANCE = 1.5; // degrees

// FastAccelStepper
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *pitch_stepper = NULL;

// MPU data
float pitch = 0;
float roll = 0;
float yaw = 0;
float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 5000);
  delay(1000);
  
  Serial.println("\n=== Auto-Level System ===");
  
  // Initialize I2C
  Wire.begin(MPU_SDA_PIN, MPU_SCL_PIN);
  Wire.setClock(400000);
  
  // Initialize MPU-6050
  if (!initMPU6050()) {
    Serial.println("ERROR: MPU-6050 not found!");
    while(1) delay(1000);
  }
  
  Serial.println("MPU-6050 found!");
  
  
  // Calibrate gyroscope
  // calibrateGyro();
  
  // Initialize stepper
  engine.init();
  pitch_stepper = engine.stepperConnectToPin(PITCH_STEP_PIN);
  
  if (pitch_stepper) {
    pitch_stepper->setDirectionPin(PITCH_DIR_PIN);
    pitch_stepper->setSpeedInHz(SPEED_HZ);
    pitch_stepper->setAcceleration(ACCELERATION);
    Serial.println("Pitch stepper initialized");
  } else {
    Serial.println("ERROR: Stepper init failed!");
    while(1) delay(1000);
  }
  
  // Auto-level
  // autoLevel();

  //Calibrate gyroscope
  // calibrateGyro();
  
  Serial.println("\n=== Ready ===");
  Serial.println("Type 'L' to re-level");
  Serial.println("Type 'P' to print current pitch");
}

void loop() {
  // Update MPU readings
  // readMPU6050();
  
  // Check for commands
  // if (Serial.available()) {
  //   char cmd = Serial.read();
    
  //   if (cmd == 'L' || cmd == 'l') {
  //     autoLevel();
  //   }
  //   else if (cmd == 'P' || cmd == 'p') {
  //     Serial.print("Current pitch: ");
  //     Serial.print(yaw, 2);  // Your pitch = sensor's yaw (Z axis)
  //     Serial.println("°");
  //   }
  // }

  goTo90Degrees();
  
  delay(50);
}

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

// void calibrateGyro() {
//   Serial.println("\nCalibrating gyroscope...");
//   Serial.println("Keep sensor STILL for 3 seconds!");
//   delay(3000);
  
//   const int samples = 500;
//   long sumX = 0, sumY = 0, sumZ = 0;
  
//   for(int i = 0; i < samples; i++) {
//     Wire.beginTransmission(MPU_ADDR);
//     Wire.write(0x43);
//     Wire.endTransmission(false);
//     Wire.requestFrom(MPU_ADDR, 6, true);
    
//     int16_t gx = Wire.read() << 8 | Wire.read();
//     int16_t gy = Wire.read() << 8 | Wire.read();
//     int16_t gz = Wire.read() << 8 | Wire.read();
    
//     sumX += gx;
//     sumY += gy;
//     sumZ += gz;
//     delay(10);
    
//     if(i % 100 == 0) Serial.print(".");
//   }
  
//   gyroX_offset = (sumX / samples) / 131.0;
//   gyroY_offset = (sumY / samples) / 131.0;
//   gyroZ_offset = (sumZ / samples) / 131.0;
  
//   Serial.println(" Done!");
//   Serial.println("Calibration complete");
// }

// void readMPU6050() {
//   Wire.beginTransmission(MPU_ADDR);
//   Wire.write(0x3B);
//   Wire.endTransmission(false);
//   Wire.requestFrom(MPU_ADDR, 14, true);
  
//   int16_t accelX = Wire.read() << 8 | Wire.read();
//   int16_t accelY = Wire.read() << 8 | Wire.read();
//   int16_t accelZ = Wire.read() << 8 | Wire.read();
//   Wire.read(); Wire.read(); // Skip temperature
//   int16_t gyroX = Wire.read() << 8 | Wire.read();
//   int16_t gyroY = Wire.read() << 8 | Wire.read();
//   int16_t gyroZ = Wire.read() << 8 | Wire.read();
  
//   // Convert to g and deg/s
//   float accelX_g = accelX / 16384.0;
//   float accelY_g = accelY / 16384.0;
//   float accelZ_g = accelZ / 16384.0;
  
//   float gyroX_dps = (gyroX / 131.0) - gyroX_offset;
//   float gyroY_dps = (gyroY / 131.0) - gyroY_offset;
//   float gyroZ_dps = (gyroZ / 131.0) - gyroZ_offset;
  
//   // Calculate time delta
//   unsigned long currentTime = millis();
//   float dt = (currentTime - lastTime) / 1000.0;
//   lastTime = currentTime;
  
//   if (dt > 0.1) dt = 0.02;
  
//   // Calculate angles from accelerometer
//   float accel_pitch = atan2(accelY_g, sqrt(accelX_g*accelX_g + accelZ_g*accelZ_g)) * 180.0/PI;
//   float accel_roll = atan2(-accelX_g, accelZ_g) * 180.0/PI;
  
//   // Integrate gyroscope
//   pitch += gyroX_dps * dt;
//   roll += gyroY_dps * dt;
//   yaw += gyroZ_dps * dt;
  
//   // Complementary filter
//   const float alpha = 0.98;
//   pitch = alpha * pitch + (1 - alpha) * accel_pitch;
//   roll = alpha * roll + (1 - alpha) * accel_roll;
// }

// void autoLevel() {
//   Serial.println("\n=== Auto-Leveling ===");
  
//   // Read current pitch multiple times to get stable reading
//   for(int i = 0; i < 20; i++) {
//     readMPU6050();
//     delay(20);
//   }
  
//   // Your gimbal pitch = sensor's yaw axis
//   float current_pitch = yaw;
  
//   Serial.print("Current pitch: ");
//   Serial.print(current_pitch, 1);
//   Serial.println("°");
  
//   if (abs(current_pitch) < LEVEL_TOLERANCE) {
//     Serial.println("Already level!");
//     return;
//   }
  
//   // Calculate correction needed
//   float correction_angle = -current_pitch;
//   int32_t correction_steps = round(correction_angle * STEPS_PER_PITCH_DEGREE);
  
//   Serial.print("Correcting: ");
//   Serial.print(correction_angle, 1);
//   Serial.print("° (");
//   Serial.print(correction_steps);
//   Serial.println(" steps)");
  
//   // Move to level
//   int32_t target = pitch_stepper->getCurrentPosition() + correction_steps;
//   pitch_stepper->moveTo(target);
  
//   // Wait for movement to complete
//   while(pitch_stepper->isRunning()) {
//     delay(50);
//   }
  
//   // Verify level
//   Serial.println("Verifying...");
//   delay(500);
  
//   for(int i = 0; i < 20; i++) {
//     readMPU6050();
//     delay(20);
//   }
  
//   float final_pitch = yaw;
//   Serial.print("Final pitch: ");
//   Serial.print(final_pitch, 1);
//   Serial.println("°");
  
//   if (abs(final_pitch) < LEVEL_TOLERANCE) {
//     Serial.println("✓ Level achieved!");
//   } else {
//     Serial.println("⚠ Close enough");
//   }
// }

float readPitchOnly() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  return atan2(ay_g, az_g) * 180.0 / PI;
}


// void autoLevel() {
//   Serial.println("\n=== Auto-Leveling ===");

//   // Let readings settle
//   float pitch = 0;
//   for (int i = 0; i < 20; i++) {
//     pitch = readPitchOnly();
//     delay(20);
//   }

//   while (true) {
//     pitch = readPitchOnly();

//     Serial.print("Pitch: ");
//     Serial.println(pitch, 2);

//     if (abs(pitch) < LEVEL_TOLERANCE) {
//       pitch_stepper->stopMove();
//       Serial.println("✓ Level achieved!");
//       break;
//     }

//     float correction_deg = pitch;

//     int32_t steps = correction_deg * STEPS_PER_PITCH_DEGREE * 0.6;
//     steps = constrain(steps, -300, 300);

//     pitch_stepper->move(steps);

//     while (pitch_stepper->isRunning()) {
//       delay(5);
//     }

//     delay(80); // allow mechanical settling
//   }
// }

void goTo90Degrees() {
  Serial.println("\n=== Moving to 90° ===");

  uint32_t start = millis();

  while (millis() - start < 5000) {
    float pitch = readPitchOnly();   // verified correct
    float error = TARGET_PITCH_DEG - pitch;

    Serial.print("Pitch: ");
    Serial.print(pitch, 2);
    Serial.print("  Error: ");
    Serial.println(error, 2);

    if (abs(error) < ANGLE_TOLERANCE) {
      pitch_stepper->stopMove();
      Serial.println("✓ Target reached!");
      return;
    }

    float absErr = abs(error);

    // Scale gain down as we approach target
    float gain;
    if (absErr > 20)       gain = 0.6;
    else if (absErr > 10)  gain = 0.4;
    else if (absErr > 5)   gain = 0.25;
    else if (absErr > 2)   gain = 0.15;
    else                   gain = 0.08;
    int32_t steps = error * STEPS_PER_PITCH_DEGREE * gain;

    steps = constrain(steps, -300, 300);

    pitch_stepper->move(steps);

    while (pitch_stepper->isRunning()) delay(5);
    delay(80);
  }

  pitch_stepper->stopMove();
  Serial.println("⚠ Timeout");
}

