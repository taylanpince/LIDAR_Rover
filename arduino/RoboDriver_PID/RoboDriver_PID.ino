#include <AltSoftSerial.h>
#include <DeadReckoner.h>
#include <PID_v1.h>

AltSoftSerial XBee;

const char COMMAND_FORWARDS = 0x46;
const char COMMAND_BACKWARDS = 0x42;
const char COMMAND_ROTATE = 0x52;
const char COMMAND_END = 0x0D;

const int PWM_A = 5;
const int PWM_B = A2;
const int DIR_A = A1;
const int DIR_B = A0;
const int BRK_A = A3;
const int BRK_B = 6;
const int ENC_L = 3;
const int ENC_R = 2;
const int POWER_OFFSET = 120;         // Motors don't respond below this value
const int SAMPLE_TIME = 50;           // PID sampling in ms
const double WHEEL_D = 64.0;          // Wheel diameter (mm)
const double SHAFT_LENGTH = 140.0;    // Shaft length, wheel to wheel (mm)
const double WHEEL_C = PI * WHEEL_D;  // Wheel circumference (mm)
const int CLICKS_PER_REV = 20;        // Encoder clicks for one full turn
const double TIME_PER_REV = CLICKS_PER_REV * 1000;

volatile unsigned int leftTurns, rightTurns = 0;
volatile unsigned int totalLeftTurns, totalRightTurns = 0;

DeadReckoner deadReckoner(&totalLeftTurns, &totalRightTurns, 1, WHEEL_D / 2, SHAFT_LENGTH);

double setpointLeft = 150;
double setpointRight = 150;
volatile double inputLeft, inputRight;
double outputLeft, outputRight;
const double Kp = 0.2, Ki = 0.2, Kd = 0.4;
volatile unsigned long startTimeA, startTimeB, nowTime;

PID PIDControllerLeft(&inputLeft, &outputLeft, &setpointLeft, Kp, Ki, Kd, DIRECT);
PID PIDControllerRight(&inputRight, &outputRight, &setpointRight, Kp, Ki, Kd, DIRECT);

void countLeft() {
  leftTurns++;

  if (leftTurns == CLICKS_PER_REV) {
    inputLeft = TIME_PER_REV / (double)(nowTime - startTimeA);
    startTimeA = nowTime;
    totalLeftTurns++;
    leftTurns = 0;
  }
}

void countRight() {
  rightTurns++;

  if (rightTurns == CLICKS_PER_REV) {
    inputRight = TIME_PER_REV / (double)(nowTime - startTimeB);
    startTimeB = nowTime;
    totalRightTurns++;
    rightTurns = 0;
  }
}

void driveStraight(int dist, bool direction = true) {
  Serial.print(F("Driving: "));
  Serial.print(dist);
  Serial.println(F("cm"));
  
  leftTurns = 0;
  rightTurns = 0;
  totalLeftTurns = 0;
  totalRightTurns = 0;
  inputLeft = 0;
  inputRight = 0;
  outputLeft = 0;
  outputRight = 0;
  startTimeA, startTimeB = 0;

  // Calculate target point in encoder clicks
  double targetDistance = (double)dist * 10;
  double targetRevolutions = targetDistance / WHEEL_C;
  unsigned long targetClicks = targetRevolutions * CLICKS_PER_REV;
  unsigned long avgClicks = 0;

  Serial.print(F("Target clicks: "));
  Serial.println(targetClicks);

  // Release breaks
  digitalWrite(BRK_A, LOW);
  digitalWrite(BRK_B, LOW);

  // Set direction
  digitalWrite(DIR_A, direction ? LOW : HIGH);
  digitalWrite(DIR_B, direction ? LOW : HIGH);

  double totalDistance = 0.0;
  long lastReportTime = 0;
  
  // Drive until the target encoder clicks are reached by either motor
  while (avgClicks < targetClicks) {
    nowTime = millis();
    avgClicks = (totalLeftTurns + totalRightTurns) / 2;

    PIDControllerLeft.Compute();
    PIDControllerRight.Compute();

    deadReckoner.computePosition();

    // Cartesian coordinate of latest location estimate.
    // Length unit correspond to the one specified under MEASUREMENTS.
//    double x = deadReckoner.getX();
//    double y = deadReckoner.getY();

    // Left and right angular velocities.
//    double wl = deadReckoner.getWl();
//    double wr = deadReckoner.getWr();

    // getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
    // This angle is set initially at zero before the robot starts moving.
    double theta = deadReckoner.getTheta() * RAD_TO_DEG;
//    double mappedOutput = map(abs(theta), 0, 90, 0, POWER_OFFSET);
//
//    if (theta > 0.0) {
//      outputRight -= constrain(mappedOutput, 0, POWER_OFFSET);
//    } else if (theta < 0.0) {
//      outputLeft -= constrain(mappedOutput, 0, POWER_OFFSET);
//    }

    // Total distance robot has travelled.
//    totalDistance = sqrt(x * x + y * y);

    // Set motor speeds
    analogWrite(PWM_B, outputLeft + POWER_OFFSET);
    analogWrite(PWM_A, outputRight + POWER_OFFSET);

    if (nowTime - lastReportTime >= 1000) {
      Serial.print(nowTime);
      Serial.print("\t");
      Serial.print(totalLeftTurns);
      Serial.print("\t");
      Serial.print(totalRightTurns);
      Serial.print("\t");
      Serial.print(outputLeft);
      Serial.print("\t");
      Serial.print(outputRight);
      Serial.print("\t");
      Serial.print(theta);
      Serial.print("\t");
      Serial.println(avgClicks);

      lastReportTime = nowTime;
    }

    delay(50);
  }

  brake();
}

void brake() {
  digitalWrite(BRK_A, HIGH);
  digitalWrite(BRK_B, HIGH);  
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
  PIDControllerLeft.SetOutputLimits(0, POWER_OFFSET);
  PIDControllerRight.SetOutputLimits(0, POWER_OFFSET);
  PIDControllerLeft.SetSampleTime(SAMPLE_TIME);
  PIDControllerRight.SetSampleTime(SAMPLE_TIME);
  PIDControllerLeft.SetMode(AUTOMATIC);
  PIDControllerRight.SetMode(AUTOMATIC);

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
