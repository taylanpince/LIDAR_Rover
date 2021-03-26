#include <EnableInterrupt.h>
#include <QuadratureEncoder.h>
#include <PID_v1.h>

#include "LIDAR.h"
#include "MPU6050_6Axis_MotionApps20.h"

const char COMMAND_LEFTMOTOR = 0x4C;
const char COMMAND_RIGHTMOTOR = 0x52;
const char COMMAND_FORWARDS = 0x46;
const char COMMAND_BACKWARDS = 0x42;
const char COMMAND_SCAN = 0x53;
const char COMMAND_SCAN_STOP = 0x58;
const char COMMAND_END = 0x2E;

const int PWM_A = 4;
const int PWM_B = 5;
const int DIR_A = 6;
const int BRK_A = 9;
const int DIR_B = 7;
const int BRK_B = 8;
const int ENC_D = 19;
const int ENC_C = 18;
const int ENC_B = 2;
const int ENC_A = 3;
const int DIR_C = 10;
const int BRK_C = 11;
const int PWM_C = 12;
const int ENC_LIDAR = A9;
const int LIDAR_CALIBRATION_PIN = A8;
const int MPU_INT_PIN = A15;

double PID_setpoint = 10.0;
double PID_output = 0.0;
double PID_input = 0.0;
const double Kp = 1.0, Ki = 1.2, Kd = 0.1;
const int PID_SAMPLE_TIME = 100;

// MPU control/status vars
bool dmpReady = false;     // set true if DMP init was successful
uint8_t mpuIntStatus;      // holds actual interrupt status byte from MPU
uint8_t mpuDeviceStatus;   // return status after each device operation (0 = success, !0 = error)
uint16_t mpuPacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t mpuFIFOCount;     // count of all bytes currently in FIFO
uint8_t mpuFIFOBuffer[64]; // FIFO storage buffer

volatile bool mpuInterrupt = false;
volatile bool markDetected = false;
volatile long markDetectTime = 0;
volatile int motorPosition = 0;
volatile long totalMotorPosition = 0;
long lastMotorPosition = 0;
long lastMPUMessageTime = 0;
bool scanRunning = false;

const int IMU_PUBLISH_RATE = 1000 / 10; // 10Hz
const int MOTOR_PUBLISH_RATE = 1000 / 10; // 10Hz

enum MOTOR {
  LEFT,
  RIGHT,
};

enum DIRECTION {
  FORWARDS,
  BACKWARDS,
};

DIRECTION leftMotorDirection = DIRECTION::FORWARDS;
DIRECTION rightMotorDirection = DIRECTION::FORWARDS;

MPU6050 mpu;
LIDARLite lidar = LIDARLite();
Encoders leftEncoder(ENC_A, ENC_B);
Encoders rightEncoder(ENC_C, ENC_D);
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

void detectMPU() {
  mpuInterrupt = true;
}

void setMotorDirection(MOTOR motor, DIRECTION dir, bool forceUpdate = false) {
  if (motor == MOTOR::LEFT) {
    if (leftMotorDirection == dir && !forceUpdate) return;
    leftMotorDirection = dir;
    digitalWrite(BRK_A, (dir == DIRECTION::FORWARDS) ? LOW : HIGH);
    digitalWrite(DIR_A, (dir == DIRECTION::FORWARDS) ? HIGH : LOW);
  } else {
    if (rightMotorDirection == dir && !forceUpdate) return;
    rightMotorDirection = dir;
    digitalWrite(BRK_B, (dir == DIRECTION::FORWARDS) ? LOW : HIGH);
    digitalWrite(DIR_B, (dir == DIRECTION::FORWARDS) ? HIGH : LOW);
  }
}

void setMotorSpeed(MOTOR motor, int power) {
  if (motor == MOTOR::LEFT) {
    analogWrite(PWM_A, power);
  } else {
    analogWrite(PWM_B, power);
  }
}

void setup() {
  pinMode(LIDAR_CALIBRATION_PIN, INPUT_PULLUP);
  pinMode(ENC_LIDAR, INPUT_PULLUP);
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(ENC_C, INPUT_PULLUP);
  pinMode(ENC_D, INPUT_PULLUP);
  
  enableInterrupt(LIDAR_CALIBRATION_PIN, detectMark, RISING);
  enableInterrupt(MPU_INT_PIN, detectMPU, RISING);
  enableInterrupt(ENC_LIDAR, detectEncoder, CHANGE);

  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(BRK_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(BRK_B, OUTPUT);
  pinMode(DIR_C, OUTPUT);
  pinMode(BRK_C, OUTPUT);
  pinMode(PWM_C, OUTPUT);

  Fastwire::setup(200, true);

  PIDController.SetOutputLimits(30, 60);
  PIDController.SetSampleTime(PID_SAMPLE_TIME);
  PIDController.SetMode(AUTOMATIC);

  Serial.begin(115200);
  Serial2.begin(57600);
  
  lidar.init();
  mpu.initialize();
  
  delay(2000);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Rover online"));

  setMotorDirection(MOTOR::LEFT, DIRECTION::FORWARDS, true);
  setMotorDirection(MOTOR::RIGHT, DIRECTION::FORWARDS, true);

  calibrateMPU();
}

void calibrateMPU() {
  Serial.println(F("Initializing DMP..."));
  mpuDeviceStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (mpuDeviceStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    mpuPacketSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.println(mpuDeviceStatus);
  }
}

void startScan() {
  if (scanRunning) return;

  markDetected = false;
  scanRunning = true;
  
  digitalWrite(DIR_C, HIGH);
  digitalWrite(BRK_C, LOW);
  analogWrite(PWM_C, 120);

  delay(1000);

  // Drive until start mark
  while (!markDetected) {
    continue;
  }

  analogWrite(PWM_C, 40);

  processScan();
}

byte DATA_TYPE_SCAN = 100;
byte DATA_TYPE_MOTOR_ENC = 101;
byte DATA_TYPE_QUATERNION = 102;
byte DATA_TYPE_GYRO = 103;
byte DATA_TYPE_ACCELEROMETER = 104;

byte OUTPUT_BUFFER_BEGIN = 24;
byte OUTPUT_BUFFER_END = 23;

byte outputBuffer[12];
int lastDistance = 0;

int16_t quaternionValue[4];
int16_t gyroValue[3];
int16_t accelerationValue[3];
int16_t *quaternion = quaternionValue;
int16_t *gyro = gyroValue;
int16_t *acceleration = accelerationValue;

void writeOutputBuffer() {
  outputBuffer[0] = OUTPUT_BUFFER_BEGIN;
  outputBuffer[11] = OUTPUT_BUFFER_END;

  Serial2.write(outputBuffer, sizeof(outputBuffer));
  memset(outputBuffer, 0, sizeof(outputBuffer));
}

void processMPU() {
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  mpuFIFOCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || mpuFIFOCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (mpuFIFOCount < mpuPacketSize) {
      mpuFIFOCount = mpu.getFIFOCount();
    }

    // read a packet from FIFO
    mpu.getFIFOBytes(mpuFIFOBuffer, mpuPacketSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    mpuFIFOCount -= mpuPacketSize;

    if (millis() - lastMPUMessageTime < IMU_PUBLISH_RATE) {
      return;
    }
    
    lastMPUMessageTime = millis();

    mpu.dmpGetQuaternion(quaternion, mpuFIFOBuffer);
    mpu.dmpGetAccel(acceleration, mpuFIFOBuffer);
    mpu.dmpGetGyro(gyro, mpuFIFOBuffer);
    
    // Quaternion values
    outputBuffer[1] = DATA_TYPE_QUATERNION;
    outputBuffer[2] = highByte(quaternionValue[0]);
    outputBuffer[3] = lowByte(quaternionValue[0]);
    outputBuffer[4] = highByte(quaternionValue[1]);
    outputBuffer[5] = lowByte(quaternionValue[1]);
    outputBuffer[6] = highByte(quaternionValue[2]);
    outputBuffer[7] = lowByte(quaternionValue[2]);
    outputBuffer[8] = highByte(quaternionValue[3]);
    outputBuffer[9] = lowByte(quaternionValue[3]);

    writeOutputBuffer();
    
    // Gyro values
    outputBuffer[1] = DATA_TYPE_GYRO;
    outputBuffer[2] = highByte(gyroValue[0]);
    outputBuffer[3] = lowByte(gyroValue[0]);
    outputBuffer[4] = highByte(gyroValue[1]);
    outputBuffer[5] = lowByte(gyroValue[1]);
    outputBuffer[6] = highByte(gyroValue[2]);
    outputBuffer[7] = lowByte(gyroValue[0]);

    writeOutputBuffer();

    // Accelerometer values
    outputBuffer[1] = DATA_TYPE_ACCELEROMETER;
    outputBuffer[2] = highByte(accelerationValue[0]);
    outputBuffer[3] = lowByte(accelerationValue[0]);
    outputBuffer[4] = highByte(accelerationValue[1]);
    outputBuffer[5] = lowByte(accelerationValue[1]);
    outputBuffer[6] = highByte(accelerationValue[2]);
    outputBuffer[7] = lowByte(accelerationValue[2]);

    writeOutputBuffer();
  }
}

void processScan() {
  lastDistance = lidar.measureDistance();

  outputBuffer[1] = DATA_TYPE_SCAN;
  outputBuffer[2] = highByte(motorPosition);
  outputBuffer[3] = lowByte(motorPosition);
  outputBuffer[4] = highByte(lastDistance);
  outputBuffer[5] = lowByte(lastDistance);

  writeOutputBuffer();

  PID_input = (double)totalMotorPosition - (double)lastMotorPosition;
  lastMotorPosition = totalMotorPosition;

  PIDController.Compute();

  Serial.print(PID_input);
  Serial.print("\t");
  Serial.print(PID_output);
  Serial.println();

  analogWrite(PWM_C, PID_output);
}

void stopScan() {
  if (!scanRunning) return;
  
  scanRunning = false;
  
  digitalWrite(DIR_C, LOW);
  digitalWrite(BRK_C, LOW);
  analogWrite(PWM_C, 0);
}

uint16_t leftEncoderCount = 0;
uint16_t rightEncoderCount = 0;
long lastMotorProcessingTime = 0;

void processMotorPositions() {
  if (millis() - lastMotorProcessingTime < MOTOR_PUBLISH_RATE) {
    return;
  }

  leftEncoderCount = leftEncoder.getEncoderCount();
  rightEncoderCount = rightEncoder.getEncoderCount();

  outputBuffer[1] = DATA_TYPE_MOTOR_ENC;
  outputBuffer[2] = highByte(leftEncoderCount);
  outputBuffer[3] = lowByte(leftEncoderCount);
  outputBuffer[4] = highByte(rightEncoderCount);
  outputBuffer[5] = lowByte(rightEncoderCount);

  writeOutputBuffer();

  lastMotorProcessingTime = millis();
}

char activeCommand = 0x00;
char activeDirection = 0x00;
char activePayload[5] = {};
uint8_t payloadIndex = 0;
long lastMotorCommandTime = 0;

void loop() {
  if (scanRunning) processScan();
  if (mpuInterrupt) processMPU();

  processMotorPositions();

  Serial2.flush();

  long nowTime = millis();

  while (Serial2.available()) {
    char value = Serial2.read();
    
    if (value == COMMAND_END) {
      if (activeCommand == COMMAND_SCAN) {
        startScan();
      } else if (activeCommand == COMMAND_SCAN_STOP) {
        stopScan();
      } else {
        int power = atoi((char *)activePayload);
  
        MOTOR motor;
        DIRECTION dir;

        Serial.print(activeCommand);
        Serial.print(F("\t"));
        Serial.println(power);
        
        if (activeCommand == COMMAND_LEFTMOTOR) {
          motor = MOTOR::LEFT;
        } else if (activeCommand == COMMAND_RIGHTMOTOR) {
          motor = MOTOR::RIGHT;
        }
    
        if (activeDirection == COMMAND_FORWARDS) {
          dir = DIRECTION::FORWARDS;
        } else if (activeDirection == COMMAND_BACKWARDS) {
          dir = DIRECTION::BACKWARDS;
        }
  
        setMotorDirection(motor, dir);
        setMotorSpeed(motor, power);

        lastMotorCommandTime = nowTime;
      }

      payloadIndex = 0;
      activeCommand = 0x00;
      activeDirection = 0x00;

      memset(activePayload, 0, sizeof(activePayload));
    } else if (payloadIndex == 0) {
      activeCommand = value;
      payloadIndex++;
    } else if (payloadIndex == 1) {
      activeDirection = value;
      payloadIndex++;
    } else {
      activePayload[payloadIndex - 2] = value;
      payloadIndex++;
    }
  }

  // Too much time since last command, stop
  if (nowTime - lastMotorCommandTime > 1000) {
    setMotorSpeed(MOTOR::LEFT, 0);
    setMotorSpeed(MOTOR::RIGHT, 0);
  }
}
