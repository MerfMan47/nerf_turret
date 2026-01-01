#include <HardwareSerial.h>

HardwareSerial radarSerial(1);  // Use UART1

// Frame delimiters
const uint8_t FRAME_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t FRAME_FOOTER[] = {0x55, 0xCC};

struct RadarTarget {
  int16_t x;
  int16_t y;
  int16_t speed;
  uint16_t resolution;
  float distance;
  float angle;
};

uint8_t rxBuffer[64];
uint8_t rxIndex = 0;
bool frameStarted = false;

void setup() {
  Serial.begin(115200);
  radarSerial.begin(256000, SERIAL_8N1, 5, 4);  // RX=5, TX=4
  radarSerial.setRxBufferSize(256);
  
  Serial.println("RD03D Radar Tracker");
  Serial.println("Waiting for data...");
}

void loop() {
  while (radarSerial.available()) {
    uint8_t byte = radarSerial.read();
    
    // Look for frame header
    if (!frameStarted) {
      rxBuffer[rxIndex++] = byte;
      
      // Check if we have enough bytes for header
      if (rxIndex >= 4) {
        // Check for header pattern
        if (rxBuffer[rxIndex-4] == FRAME_HEADER[0] &&
            rxBuffer[rxIndex-3] == FRAME_HEADER[1] &&
            rxBuffer[rxIndex-2] == FRAME_HEADER[2] &&
            rxBuffer[rxIndex-1] == FRAME_HEADER[3]) {
          
          // Found header!
          frameStarted = true;
          rxBuffer[0] = FRAME_HEADER[0];
          rxBuffer[1] = FRAME_HEADER[1];
          rxBuffer[2] = FRAME_HEADER[2];
          rxBuffer[3] = FRAME_HEADER[3];
          rxIndex = 4;
        } else {
          // Shift buffer left
          for (int i = 0; i < 3; i++) {
            rxBuffer[i] = rxBuffer[i + 1];
          }
          rxIndex = 3;
        }
      }
    } else {
      // We're collecting a frame
      rxBuffer[rxIndex++] = byte;
      
      // Prevent overflow
      if (rxIndex >= 64) {
        Serial.println("[ERROR] Buffer overflow, resetting");
        rxIndex = 0;
        frameStarted = false;
      }
      
      // Check for footer (55 CC)
      if (rxIndex >= 6 && 
          rxBuffer[rxIndex-2] == FRAME_FOOTER[0] && 
          rxBuffer[rxIndex-1] == FRAME_FOOTER[1]) {
        
        // Frame complete!
        parseFrame(rxBuffer, rxIndex);
        
        // Reset for next frame
        rxIndex = 0;
        frameStarted = false;
      }
    }
  }
}

void parseFrame(uint8_t* frame, int length) {
  // Expected frame: AA FF 03 00 [data bytes] 55 CC
  // Data format from your example:

  // Byte 4-5: Target 1 X
  // Byte 6-7: Target 1 Y
  // Byte 8-9: Target 1 Speed
  // Byte 10-11: Target 1 Resolution
  // Then Target 2 (8 bytes), Target 3 (8 bytes)
  
  if (length < 30) {  // Minimum expected length
    Serial.println("[WARNING] Frame too short");
    return;
  }
  
  // Parse Target 1 (starts at byte 4, after header)
  uint16_t x_raw = frame[4] | (frame[5] << 8);
  uint16_t y_raw = frame[6] | (frame[7] << 8);
  uint16_t speed_raw = frame[8] | (frame[9] << 8);
  uint16_t resolution = frame[10] | (frame[11] << 8);
  
  // Apply offsets according to documentation
  // X: 0 - raw_value (if raw > 0, result is negative)
  int16_t x = 0 - (int16_t)x_raw;
  
  // Y: raw_value - 32768 (2^15)
  int16_t y = (int16_t)y_raw - 32768;
  
  // Speed: 0 - raw_value
  int16_t speed = 0 - (int16_t)speed_raw;
  
  // Calculate distance and angle
  float distance = sqrt(pow(x, 2) + pow(y, 2));
  float angle = atan2(y, x) * 180.0 / PI;
  
  // Check if target exists (all zeros means no target)
  if (x_raw == 0 && y_raw == 0 && speed_raw == 0 && resolution == 0) {
    Serial.println("No target detected");
    return;
  }
  
  Serial.println("\n--- Radar Data ---");
  Serial.print("X: "); Serial.print(x); Serial.println(" mm");
  Serial.print("Y: "); Serial.print(y); Serial.println(" mm");
  Serial.print("Speed: "); Serial.print(speed); Serial.println(" cm/s");
  Serial.print("Resolution: "); Serial.print(resolution); Serial.println(" mm");
  Serial.print("Distance: "); Serial.print(distance / 10.0, 2); Serial.println(" cm");
  Serial.print("Angle: "); Serial.print(angle, 2); Serial.println("°");
  Serial.println("-------------------------");
}