#include <Encoder.h>

#include "LIDARLite.h"

const int PWM_A = 3;
const int DIR_A = 12;
const int BRK_A = 9;

const int ENC_A = 18;
const int ENC_B = 19;
const int CALIBRATION_PIN = 2;

//uint16_t currentStep = 0;

volatile bool markDetected = false;
bool scanRunning = false;

Encoder encoder(ENC_A, ENC_B);
LIDARLite lidar = LIDARLite();

void detectMark() {
  markDetected = true;

  encoder.write(0);
}

void setup() {
  pinMode(CALIBRATION_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(CALIBRATION_PIN), detectMark, RISING);

  Serial.begin(115200);

  lidar.init();

  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(BRK_A, OUTPUT);

//  runCalibration();
}

//void runCalibration() {
//  markDetected = false;
//  
//  long calibrationTime = millis();
//
//  encoder.write(0);
//  drive(false, 60);
//
//  while (!markDetected || millis() - calibrationTime < 1000) {
//    if (markDetected && millis() - calibrationTime < 1000) markDetected = false;
//  }
//
//  stopMotor();
//
//  encoder.write(0);
//
//  markDetected = false;
//  currentStep = 0;
//}
//
//int scanMeasurements[500][2] = {};
//
//void runScan() {
//  markDetected = false;
//  currentStep = 0;
//
//  long startTime = millis();
//
//  encoder.write(0);
//  drive(false, 60);
//
//  int currentDistance;
//
//  while (!markDetected || millis() - startTime < 1000) {
//    if (markDetected && millis() - startTime < 1000) markDetected = false;
//
//    currentDistance = lidar.measureDistance();
//    
//    scanMeasurements[currentStep][0] = encoder.read();
//    scanMeasurements[currentStep][1] = currentDistance;
//
//    currentStep++;
//  }
//
//  stopMotor();
//
//  for (int i = 0; i < currentStep; i++) {
//    Serial.print(i);
//    Serial.print("|");
//    Serial.print(scanMeasurements[i][0]);
//    Serial.print("|");
//    Serial.print(scanMeasurements[i][1]);
//    Serial.print(",");
//  }
//
//  Serial.println("");
//
//  encoder.write(0);
//
//  markDetected = false;
//  currentStep = 0;
//
//  memset(scanMeasurements, 0, sizeof(scanMeasurements));
//}

void startScan() {
  markDetected = false;
  scanRunning = true;
  
  drive(false, 60);

  // Drive until start mark
  while (!markDetected) {
    continue;
  }

  processScan();
}

void processScan() {
  Serial.print(encoder.read());
  Serial.print("|");
  Serial.println(lidar.measureDistance());
}

void stopScan() {
  scanRunning = false;
  
  stopMotor();
}

const char COMMAND_SCAN = 0x53;
const char COMMAND_SCAN_STOP = 0x2e;
const char COMMAND_CALIBRATE = 0x43;

void loop() {
  if (scanRunning) processScan();
  
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == COMMAND_SCAN) {
      startScan();
    } else if (command == COMMAND_SCAN_STOP) {
      stopScan();
    } else if (command == COMMAND_CALIBRATE) {
      //runCalibration();
    }
  }
}

void drive(bool dir, int power) {
  // Disable breaks
  digitalWrite(BRK_A, LOW);

  // Direction forward
  digitalWrite(DIR_A, dir ? HIGH : LOW);

  // Energize!
  analogWrite(PWM_A, power);
}

void stopMotor() {
  // Full break
  digitalWrite(BRK_A, HIGH);
  analogWrite(PWM_A, 0);
}
