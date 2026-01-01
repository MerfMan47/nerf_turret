#define PIN_SERIAL_TX (4)
#define PIN_SERIAL_RX (5)

// ----- ADD THESE TWO LINES -----
#define TRIG_PIN 10
#define ECHO_PIN 11
// --------------------------------

const byte HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
const byte FOOTER[] = {0x55, 0xCC};
const size_t DATA_LENGTH = 24;
const size_t GROUP_LENGTH = 8;

byte buffer[DATA_LENGTH];
byte currentIndex = 0;
bool frameStarted = false;

void setup() {
  Serial.begin(9600);

  // Radar on custom pins
  Serial2.begin(256000, SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX);

  // ----- ADD THESE 3 LINES FOR ULTRASONIC -----
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  // -------------------------------------------

  Serial.println("LD2450 + HC-SR04 started");
}

void loop() {
  // Your original radar parsing (unchanged)
  while (Serial2.available()) {
    byte inputByte = Serial2.read();

    if (!frameStarted) {
      if (inputByte == HEADER[currentIndex]) {
        currentIndex++;
        if (currentIndex == sizeof(HEADER)) {
          frameStarted = true;
          currentIndex = 0;
        }
      } else {
        currentIndex = 0;
      }
    } else {
      if (currentIndex < DATA_LENGTH) {
        buffer[currentIndex++] = inputByte;
      } else if (currentIndex == DATA_LENGTH) {
        if (inputByte == FOOTER[0]) currentIndex++;
        else { frameStarted = false; currentIndex = 0; }
      } else if (currentIndex == DATA_LENGTH + 1) {
        if (inputByte == FOOTER[1]) {
          processFrame(buffer, DATA_LENGTH);  // This prints your nice radar table
          
          // ----- ULTRASONIC MEASUREMENT RIGHT AFTER RADAR FRAME -----
          long duration = 0;
          digitalWrite(TRIG_PIN, HIGH);
          delayMicroseconds(10);
          digitalWrite(TRIG_PIN, LOW);
          duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms ≈ 5m
          float ultra_cm = duration * 0.0343 / 2.0;
          
          Serial.print("| Ultrasonic distance: ");
          if (duration == 0) Serial.print("---");
          else Serial.print(ultra_cm, 1);
          Serial.println(" cm                                   |");
          // -----------------------------------------------------------
        }
        frameStarted = false;
        currentIndex = 0;
      }
    }
  }
}

// === ALL YOUR ORIGINAL FUNCTIONS BELOW (100 % unchanged) ===
void processFrameRaw(byte* data, size_t length) {
  Serial.println("Frame received:");
  for (size_t i = 0; i < length; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void processFrame(byte* data, size_t length) {
  if (length != DATA_LENGTH) return;
  for (int i = 0; i < 3; i++) {
    byte group[GROUP_LENGTH];
    memcpy(group, &data[i * GROUP_LENGTH], GROUP_LENGTH);
    unpackSensorData(group);
  }
}

String repeatSpaces(int count) {
  String spaces = "";
  for (int i = 0; i < count; i++) spaces += " ";
  return spaces;
}

void unpackSensorData(byte* group) {
  uint16_t int1 = ((group[1] & 0x7F) << 8) | group[0];
  uint16_t int2 = ((group[3] & 0xFF) << 8) | group[2];
  uint16_t int3 = ((group[5] & 0x7F) << 8) | group[4];
  uint16_t uintInt = (group[7] << 8) | group[6];

  if (int1 != 0 || int2 != 0 || int3 != 0 || uintInt != 0) {
    float xdist = (int1) / 10.0;
    if ((group[1] & 0x80) == 0) xdist = -xdist;

    float ydist = (int2);
    ydist = ydist - 0x8000;
    ydist /= 10.0;

    float speed = (int3);
    if ((group[5] & 0x80) == 0) speed = -speed;

    const int WIDTH_X = 8;
    const int WIDTH_Y = 8;
    const int WIDTH_SPD = 8;
    const int WIDTH_GATE = 5;

    Serial.print("| X: ");
    Serial.print(repeatSpaces(WIDTH_X - String(xdist).length()));
    Serial.print(xdist);
    Serial.print("cm | Y:");
    Serial.print(repeatSpaces(WIDTH_Y - String(ydist).length()));
    Serial.print(ydist);
    Serial.print("cm | Spd:");
    Serial.print(repeatSpaces(WIDTH_SPD - String(speed).length()));
    Serial.print(speed);
    Serial.print("cm/sec | Gate: ");
    Serial.print(repeatSpaces(WIDTH_GATE - String(uintInt).length()));
    Serial.print(uintInt);
    Serial.println("mm |");
  }
}