#include <EnableInterrupt.h>
#include <PID_v1.h>

#include "LIDARFastWire.h"

const int DIR_C = 10;
const int BRK_C = 11;
const int PWM_C = 12;

const int CALIBRATION_PIN = A8;
const int ENC_A = A9;
const int ENC_B = A10;

volatile bool markDetected = false;
volatile long markDetectTime = 0;
volatile int motorPosition = 0;
volatile long totalMotorPosition = 0;
long lastMotorPosition = 0;
bool scanRunning = false;

LIDARFastWire lidar = LIDARFastWire();

double PID_setpoint = 10.0;
double PID_output = 0.0;
double PID_input = 0.0;
const double Kp = 5.0, Ki = 0.01, Kd = 0.1;
const int PID_SAMPLE_TIME = 100;

PID PIDController(&PID_input, &PID_output, &PID_setpoint, Kp, Ki, Kd, DIRECT);

void detectMark() {
  if (millis() - markDetectTime < 400) {
    return;
  }
  
  markDetected = true;
  motorPosition = 0;
  markDetectTime = millis();
}

void detectEncoder() {
  motorPosition++;
  totalMotorPosition++;
}

void setup() {
  pinMode(CALIBRATION_PIN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  
  enableInterrupt(CALIBRATION_PIN, detectMark, RISING);
  enableInterrupt(ENC_A, detectEncoder, CHANGE);

  pinMode(DIR_C, OUTPUT);
  pinMode(BRK_C, OUTPUT);
  pinMode(PWM_C, OUTPUT);

  PIDController.SetOutputLimits(30, 60);
  PIDController.SetSampleTime(PID_SAMPLE_TIME);
  PIDController.SetMode(AUTOMATIC);

  Serial.begin(115200);
  Serial2.begin(57600);
  
  Serial.println(F("Intitializing LIDAR..."));
  Fastwire::setup(200, true);
  lidar.init();
  Serial.println(F("LIDAR ready"));

  delay(5000);

  Serial.println(F("Rover online"));

  startScan();
}

void startScan() {
  markDetected = false;
  scanRunning = true;
  
  digitalWrite(DIR_C, HIGH);
  digitalWrite(BRK_C, LOW);
  analogWrite(PWM_C, 120);

  delay(1000);

  // Drive until start mark
//  while (!markDetected) {
//    continue;
//  }

  analogWrite(PWM_C, 40);

  processScan();
}

byte outputBuffer[4] = {0, 0, 0, 0};
int lastDistance = 0;

void processScan() {
  lastDistance = lidar.measureDistance();
  
  outputBuffer[0] = highByte(motorPosition);
  outputBuffer[1] = lowByte(motorPosition);
  outputBuffer[2] = highByte(lastDistance);
  outputBuffer[3] = lowByte(lastDistance);

  Serial2.write(outputBuffer, sizeof(outputBuffer));
  Serial.println(lastDistance);
  PID_input = (double)totalMotorPosition - (double)lastMotorPosition;
  lastMotorPosition = totalMotorPosition;

  PIDController.Compute();

  analogWrite(PWM_C, PID_output);
}

void stopScan() {
  scanRunning = false;
  
  digitalWrite(DIR_C, LOW);
  digitalWrite(BRK_C, LOW);
  analogWrite(PWM_C, 0);
}

const char COMMAND_SCAN = 0x53;
const char COMMAND_SCAN_STOP = 0x2e;

void loop() {
  if (scanRunning) processScan();

  while (Serial2.available()) {
    char command = Serial2.read();

    if (command == COMMAND_SCAN) {
      startScan();
    } else if (command == COMMAND_SCAN_STOP) {
      stopScan();
    }
  }
}
