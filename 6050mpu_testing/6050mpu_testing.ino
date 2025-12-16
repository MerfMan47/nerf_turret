#include <Wire.h>

// MPU-6050 I2C address and pins
const int MPU_ADDR = 0x68;
const int SDA_PIN = 41;
const int SCL_PIN = 42;

// MPU-6050 registers
const int PWR_MGMT_1 = 0x6B;
const int ACCEL_XOUT_H = 0x3B;

// Sensor data
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;

// Calibration offsets
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

// Angle tracking variables
float pitch = 0, roll = 0, yaw = 0;
unsigned long lastTime = 0;

// Complementary filter constant (0.98 = 98% gyro, 2% accel)
const float alpha = 0.98;

void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 5000);
  delay(1000);
  
  Serial.println("\n=== MPU-6050 Gimbal Tracker ===");
  
  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  
  // Check connection
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.println("MPU-6050 not found! Check wiring.");
    while (1) delay(1000);
  }
  
  Serial.println("MPU-6050 found!");
  
  // Wake up MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
  
  delay(100);
  
  // Let sensor warm up and stabilize
  Serial.println("\nWarming up sensor...");
  delay(3000);
  
  // Calibrate gyroscope
  Serial.println("Calibrating gyroscope...");
  Serial.println("Keep the sensor PERFECTLY STILL!");
  delay(2000);
  
  calibrateGyro();
  
  Serial.println("Calibration complete!");
  Serial.println("\nTracking gimbal angles...");
  Serial.println("Format: Pitch | Roll | Yaw\n");
  
  lastTime = millis();
  delay(500);
}

void loop() {
  // Calculate time since last reading
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;
  
  // Read sensor data
  readMPU6050();
  
  // Convert to meaningful units
  float accelX_g = accelX / 16384.0;
  float accelY_g = accelY / 16384.0;
  float accelZ_g = accelZ / 16384.0;
  
  float gyroX_dps = (gyroX / 131.0) - gyroX_offset;
  float gyroY_dps = (gyroY / 131.0) - gyroY_offset;
  float gyroZ_dps = (gyroZ / 131.0) - gyroZ_offset;
  
  // Calculate angles from accelerometer (for pitch and roll only)
  float accel_pitch = atan2(accelY_g, sqrt(accelX_g*accelX_g + accelZ_g*accelZ_g)) * 180.0/PI;
  float accel_roll = atan2(-accelX_g, accelZ_g) * 180.0/PI;
  
  // Integrate gyroscope data
  pitch += gyroX_dps * dt;
  roll += gyroY_dps * dt;
  yaw += gyroZ_dps * dt;
  
  // Apply complementary filter (combine gyro and accel for pitch and roll)
  pitch = alpha * pitch + (1 - alpha) * accel_pitch;
  roll = alpha * roll + (1 - alpha) * accel_roll;
  // Yaw uses only gyro (accel can't measure yaw)
  
  // Remap to gimbal axes (adjust these based on your mounting)
  float gimbal_pitch = pitch;  // Change to roll or yaw if needed
  float gimbal_yaw = yaw;      // Change to pitch or roll if needed
  float gimbal_roll = roll;    // If your gimbal has 3 axes
  
  // Print angles
  Serial.print("Pitch: ");
  Serial.print(gimbal_pitch, 1);
  Serial.print("° | Yaw: ");
  Serial.print(gimbal_yaw, 1);
  Serial.print("° | Roll: ");
  Serial.print(gimbal_roll, 1);
  Serial.println("°");
  
  delay(20); // 50Hz update rate
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void calibrateGyro() {
  const int samples = 1000;  // Increased from 200
  long sumX = 0, sumY = 0, sumZ = 0;
  
  for(int i = 0; i < samples; i++) {
    readMPU6050();
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    delay(10);
    
    if(i % 40 == 0) Serial.print(".");
  }
  
  gyroX_offset = (sumX / samples) / 131.0;
  gyroY_offset = (sumY / samples) / 131.0;
  gyroZ_offset = (sumZ / samples) / 131.0;
  
  Serial.println();
  Serial.print("Offsets - X: ");
  Serial.print(gyroX_offset);
  Serial.print(" Y: ");
  Serial.print(gyroY_offset);
  Serial.print(" Z: ");
  Serial.println(gyroZ_offset);
}