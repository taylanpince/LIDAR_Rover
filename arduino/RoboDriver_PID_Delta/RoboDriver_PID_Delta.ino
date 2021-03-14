#include <AltSoftSerial.h>
#include <DeadReckoner.h>
#include <PID_v1.h>

AltSoftSerial XBee;

const char COMMAND_FORWARDS = 0x46;
const char COMMAND_BACKWARDS = 0x42;
const char COMMAND_ROTATE = 0x52;
const char COMMAND_END = 0x0D;

const int PWM_A = 5;
const int PWM_B = 6;
const int DIR_A = A0;
const int DIR_B = A2;
const int BRK_A = A1;
const int BRK_B = A3;
const int ENC_L = 3;
const int ENC_R = 2;
const double POWER_OFFSET = 130.0;         // Motors don't respond below this value
const double POWER_RANGE = 100.0;
const int SAMPLE_TIME = 100;          // PID sampling in ms
const double WHEEL_D = 64.0;          // Wheel diameter (mm)
const double SHAFT_LENGTH = 140.0;    // Shaft length, wheel to wheel (mm)
const double WHEEL_C = PI * WHEEL_D;  // Wheel circumference (mm)
const int CLICKS_PER_REV = 20;        // Encoder clicks for one full turn

volatile unsigned int leftTurns, rightTurns = 0;

bool leftMotorDirection = true;
bool rightMotorDirection = true;

DeadReckoner deadReckoner(&leftTurns, &rightTurns, CLICKS_PER_REV, WHEEL_D / 2, SHAFT_LENGTH);

double PID_setpoint = 0.0;
double PID_output = 0.0;
double PID_input = 0.0;
const double Kp = 5.0, Ki = 0.01, Kd = 0.1;
long nowTime;

PID PIDController(&PID_input, &PID_output, &PID_setpoint, Kp, Ki, Kd, DIRECT);

void countLeft() {
  if (leftMotorDirection) {
    leftTurns++;
  } else {
    leftTurns--;
  }
}

void countRight() {
  if (rightMotorDirection) {
    rightTurns++;
  } else {
    rightTurns--;
  }
}

void setMotorDirection(bool motorLeft, bool dir, bool forceUpdate = false) {
  if (motorLeft) {
    if (leftMotorDirection == dir && !forceUpdate) return;
    leftMotorDirection = dir;
    digitalWrite(BRK_A, dir ? LOW : HIGH);
    digitalWrite(DIR_A, dir ? HIGH : LOW);
  } else {
    if (rightMotorDirection == dir && !forceUpdate) return;
    rightMotorDirection = dir;
    digitalWrite(BRK_B, dir ? HIGH : LOW);
    digitalWrite(DIR_B, dir ? LOW : HIGH);
  }
}

void driveStraight(int dist, bool dir = true) {
  Serial.print(F("Driving: "));
  Serial.print(dist);
  Serial.println(F("cm"));
  
  leftTurns = 0;
  rightTurns = 0;

  // Calculate target point in encoder clicks
  double targetDistance = (double)dist * 10;
  double targetRevolutions = targetDistance / WHEEL_C;
  unsigned long targetClicks = targetRevolutions * CLICKS_PER_REV;

  Serial.print(F("Target clicks: "));
  Serial.println(targetClicks);

  setMotorDirection(true, dir, true);
  setMotorDirection(false, dir, true);

  long lastReportTime = 0;
  double totalDistance = 0.0;
  double outputLeft = POWER_OFFSET / 2;
  double outputRight = POWER_OFFSET / 2;
  double leftSpeed = 0.0;
  double rightSpeed = 0.0;
  long lastCheckTime = 0;
  long lastLeftTurns = 0;
  long lastRightTurns = 0;
  uint32_t leftDelta, rightDelta , timeDelta = 0;

  lastCheckTime = millis();

  // Drive until the target encoder clicks are reached by either motor
  while (totalDistance < targetDistance) {
    analogWrite(PWM_B, outputLeft + POWER_OFFSET);
    analogWrite(PWM_A, outputRight + POWER_OFFSET);
    
    delay(SAMPLE_TIME);

    nowTime = millis();
    leftDelta = leftTurns - lastLeftTurns;
    rightDelta = rightTurns - lastRightTurns;
    timeDelta = nowTime - lastCheckTime;
    lastCheckTime = nowTime;
    lastLeftTurns = leftTurns;
    lastRightTurns = rightTurns;

    leftSpeed = (double)leftDelta / (double)timeDelta * 1000.0;
    rightSpeed = (double)rightDelta / (double)timeDelta * 1000.0;

    PID_input = (double)leftTurns - (double)rightTurns;

    PIDController.Compute();

//    deadReckoner.computePosition();

    // Cartesian coordinate of latest location estimate.
    // Length unit correspond to the one specified under MEASUREMENTS.
//    double x = deadReckoner.getX();
//    double y = deadReckoner.getY();

    // Left and right angular velocities.
//    double wl = deadReckoner.getWl();
//    double wr = deadReckoner.getWr();

    // getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
    // This angle is set initially at zero before the robot starts moving.
//    double theta = deadReckoner.getTheta() * RAD_TO_DEG;
//
//    if (theta > 0.0) {
//      outputLeft -= constrain(map(abs(theta), 0, 90, 0, POWER_OFFSET), 0, POWER_OFFSET);
//    } else if (theta < 0.0) {
//      outputRight -= constrain(map(abs(theta), 0, 90, 0, POWER_OFFSET), 0, POWER_OFFSET);
//    }

    // Total distance robot has travelled.
//    totalDistance = sqrt(x * x + y * y);

//    constrain(PID_output, -1.0, 1.0);

    if (PID_output < 0.0) {
      setMotorDirection(true, (PID_output > -50.0));
      setMotorDirection(false, true);
      
      outputLeft = (POWER_OFFSET / 2) + (PID_output / 2);
      outputRight = (POWER_OFFSET / 2) - (PID_output / 2);
    } else if (PID_output > 0.0) {
      setMotorDirection(true, true);
      setMotorDirection(false, (PID_output < 50.0));
      
      outputRight = (POWER_OFFSET / 2) - (PID_output / 2);
      outputLeft = (POWER_OFFSET / 2) + (PID_output / 2);
    }

    if (nowTime - lastReportTime >= 1000) {
      Serial.print(nowTime);
      Serial.print("\t");
      Serial.print(leftTurns);
      Serial.print("\t");
      Serial.print(rightTurns);
      Serial.print("\t");
      Serial.print(PID_input);
      Serial.print("\t");
      Serial.print(PID_output);
      Serial.print("\t");
      Serial.print(outputLeft);
      Serial.print("\t");
      Serial.print(outputRight);
//      Serial.print("\t");
//      Serial.print(theta);
//      Serial.print("\t");
//      Serial.print(totalDistance);
      Serial.println();

      lastReportTime = nowTime;
    }
  }

  brake();
}

void brake() {
  digitalWrite(BRK_A, LOW);
  digitalWrite(BRK_B, LOW);
  digitalWrite(DIR_A, LOW);
  digitalWrite(DIR_B, LOW);
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
}

void setup() {
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), countRight, RISING);

  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(BRK_A, OUTPUT);
  pinMode(BRK_B, OUTPUT);

  // PID configutarion
  PIDController.SetOutputLimits(-POWER_RANGE, POWER_RANGE);
  PIDController.SetSampleTime(SAMPLE_TIME);
  PIDController.SetMode(AUTOMATIC);

  Serial.begin(115200);
  XBee.begin(9600);

  delay(5000);

  driveStraight(1000);
}

char command = 0x00;
char payload[5] = {};
uint8_t payload_index = 0;

void loop() {
  while (XBee.available()) {
    char value = XBee.read();

    if (value == COMMAND_END) {
      int value = atoi((char *)payload);

      Serial.println(command);
      Serial.println(value);
  
      if (command == COMMAND_FORWARDS) {
        driveStraight(value);
      } else if (command == COMMAND_BACKWARDS) {
        driveStraight(value, false);
      } else if (command == COMMAND_ROTATE) {
        
      }

      payload_index = 0;
      command = 0x00;

      memset(payload, 0, sizeof(payload));
    } else if (payload_index == 0) {
      command = value;
      payload_index++;
    } else {
      payload[payload_index - 1] = value;
      payload_index++;
    }
  }
}
