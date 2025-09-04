#include <Stepper.h>
#include <CAN.h>

// ---------- Stepper Config ----------
#define STEPS_PER_REV 200
#define MM_PER_REV 8.5             // adjust for your mechanics
#define STEPS_PER_MM (STEPS_PER_REV / MM_PER_REV)
#define STEP_SIZE 10                // micro steps per iteration

Stepper stepper1(STEPS_PER_REV, 2, 3, 4, 5);
Stepper stepper2(STEPS_PER_REV, 6, 7, 8, 9);

float actPosMM = 0.0;               // current absolute pos [mm]
int targetPosMM = 0.0;            // target absolute pos [mm]

// ---------- CAN Config ----------
unsigned char canMessage[8];

// CAN IDs
const int CAN_ID_REC_TARGET = 0x2E5;
const int CAN_ID_SEND_POS   = 0x2E4;
const int CAN_ID_HOME       = 0x2E8;

// ---------- Periodic send timer ----------
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 300;  // ms

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Stepper + CAN control");

  stepper1.setSpeed(60);
  stepper2.setSpeed(60);

  if (!CAN.begin(1000000)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Home();
}

void loop() {
  // ----- Handle CAN Receive -----
  int packetSize = CAN.parsePacket();
  if (packetSize >= 0) {
    int canId = CAN.packetId();

    if (canId == CAN_ID_REC_TARGET && packetSize >= 4) {
      int receivedTarget = 0;
      for (int i = 0; i < packetSize && i < 8; i++) {
        canMessage[i] = CAN.read();
        Serial.println(canMessage[i]);
      }
      memcpy(&receivedTarget, canMessage, sizeof(int));  // decode 4-byte int
      targetPosMM = receivedTarget / 5/2; // % to mm
      Serial.print("New target received: ");
      Serial.println(targetPosMM);
      GoToAbsMM(targetPosMM);
    }

    else if (canId == CAN_ID_HOME) {
      Serial.println("Homing command received via CAN");
      Home();
    }
  }

  // ----- Periodic position send -----
    // unsigned long now = millis();
    // if (now - lastSendTime >= SEND_INTERVAL) {
    //   SendPosition();
    //   lastSendTime = now;
    // }
}

// ---------- Stepper Control ----------
void Home() {
  Serial.println("Homing...");
  stepper1.step(STEPS_PER_REV);
  stepper2.step(STEPS_PER_REV);
  actPosMM = 12.0;
}

void GoToAbsMM(float posMM) {
  float delta = posMM - actPosMM;
  int steps = delta * STEPS_PER_MM;
  MoveSteps(steps);
  actPosMM = posMM;
  Serial.print("actPosMM: ");
  Serial.println(actPosMM);
}

void MoveSteps(int steps) {
  Serial.print("Moving steps: ");
  Serial.println(steps);

  int loops = abs(steps) / STEP_SIZE;
  int dir = (steps > 0) ? STEP_SIZE : -STEP_SIZE;

  for (int i = 0; i < loops; i++) {
    stepper1.step(dir);
    stepper2.step(dir);
  }
}

// ---------- CAN Send ----------
void SendPosition() {
  memcpy(canMessage, &actPosMM, sizeof(float));

  if (CAN.beginPacket(CAN_ID_SEND_POS)) {
    for (int i = 0; i < 8; i++) {
      CAN.write(canMessage[i]);
    }
    CAN.endPacket();
    // Serial.print("Sent current position over CAN: ");
    // Serial.println(actPosMM, 3);
  }
}
