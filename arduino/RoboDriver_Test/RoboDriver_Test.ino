#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <AltSoftSerial.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
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
const int POWER_OFFSET = 125; // Due to incoming voltage 3V motors ignore PWM below this value

const int motor_offset = 5;       // Diff. when driving straight
const int wheel_d = 64;           // Wheel diameter (mm)
const float wheel_c = PI * wheel_d; // Wheel circumference (mm)
const int counts_per_rev = 72;   // (4 pairs N-S) * (48:1 gearbox) * (2 falling/rising edges) = 384

volatile unsigned long leftTurns = 0;
volatile unsigned long rightTurns = 0;

void countLeft() {
  leftTurns++;
}

void countRight() {
  rightTurns++;
}

void setup() {
//  pinMode(ENC_L, INPUT_PULLUP);
//  pinMode(ENC_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), countRight, RISING);

  pinMode(PWM_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(BRK_A, OUTPUT);
  pinMode(BRK_B, OUTPUT);

  Serial.begin(115200);
  XBee.begin(9600);

//  delay(5000);

//  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
//    Serial.println(F("SSD1306 allocation failed"));
//    for(;;); // Don't proceed, loop forever
//  } else {
//    Serial.println(F("SSD1306 found"));
//  }

//  display.display();
//  display.clearDisplay();

//  delay(1000);

//  display.setTextSize(2);
//  display.setTextColor(SSD1306_WHITE);
//  display.setCursor(0, 0);
//  display.println(F("Online"));
//  display.println(F("Waiting..."));
//  display.display();

  runDriveTest();

//  driveStraight(100, 200);
}

void driveStraight(int dist, int power) {

  unsigned long num_ticks_l;
  unsigned long num_ticks_r;

  // Set initial motor power
  int power_l = power;
  int power_r = power;

  // Used to determine which way to turn to adjust
  unsigned long diff_l;
  unsigned long diff_r;

  // Reset encoder counts
  leftTurns = 0;
  rightTurns = 0;

  // Remember previous encoder counts
  unsigned long enc_l_prev = leftTurns;
  unsigned long enc_r_prev = rightTurns;

  // Calculate target number of ticks
  float num_rev = ((float)dist * 10) / wheel_c;  // Convert to mm
  unsigned long target_count = num_rev * counts_per_rev;
  
  // Debug
  Serial.print("Driving for ");
  Serial.print(dist);
  Serial.print(" cm (");
  Serial.print(target_count);
  Serial.print(" ticks) at ");
  Serial.print(power);
  Serial.println(" motor power");

  // Release breaks
  digitalWrite(BRK_A, LOW);
  digitalWrite(BRK_B, LOW);  

  // Drive until one of the encoders reaches desired count
  while ( (leftTurns < target_count) && (rightTurns < target_count) ) {

    // Sample number of encoder ticks
    num_ticks_l = leftTurns;
    num_ticks_r = rightTurns;

    // Print out current number of ticks
    Serial.print(num_ticks_l);
    Serial.print("\t");
    Serial.println(num_ticks_r);

    // Drive
    drive(power_l, power_r);

    // Number of ticks counted since last time
    diff_l = num_ticks_l - enc_l_prev;
    diff_r = num_ticks_r - enc_r_prev;

    // Store current tick counter for next time
    enc_l_prev = num_ticks_l;
    enc_r_prev = num_ticks_r;

    // If left is faster, slow it down and speed up right
    if ( diff_l > diff_r ) {
      power_l -= motor_offset;
      power_r += motor_offset;
    }

    // If right is faster, slow it down and speed up left
    if ( diff_l < diff_r ) {
      power_l += motor_offset;
      power_r -= motor_offset;
    }

    // Brief pause to let motors respond
    delay(20);
  }

  // Brake
  brake();
}

//void enableMotors(boolean en) {
//  if ( en ) {
//    digitalWrite(stby_pin, HIGH);
//  } else {
//    digitalWrite(stby_pin, LOW);
//  }
//}

void drive(int power_a, int power_b) {
  // Constrain power to between -125 and 125
  power_a = constrain(power_a, -POWER_OFFSET, POWER_OFFSET);
  power_b = constrain(power_b, -POWER_OFFSET, POWER_OFFSET);

  // Left motor direction
  if ( power_a < 0 ) {
    digitalWrite(DIR_A, HIGH);
  } else {
    digitalWrite(DIR_A, LOW);
  }

  // Right motor direction
  if ( power_b < 0 ) {
    digitalWrite(DIR_B, HIGH);
  } else {
    digitalWrite(DIR_B, LOW);
  }

  // Set speed
  analogWrite(PWM_A, abs(power_a) + POWER_OFFSET);
  analogWrite(PWM_B, abs(power_b) + POWER_OFFSET);
}

void brake() {
  digitalWrite(BRK_A, HIGH);
  digitalWrite(BRK_B, HIGH);  
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);
}

void runDriveTest() {
  Serial.println("Begin test...");

  // Direction forward
  digitalWrite(BRK_A, LOW);
  digitalWrite(BRK_B, HIGH);
  digitalWrite(DIR_A, HIGH);
  digitalWrite(DIR_B, LOW);

  int motorPower = 130;
  int motorBias = 0;
  double leftSpeed, rightSpeed = 0.0;
  long lastCheckTime, nowTime = 0;
  long lastLeftTurns, lastRightTurns = 0;
  uint32_t leftDelta, rightDelta , timeDelta = 0;

  lastCheckTime = millis();

  while (motorPower <= 255 - motorBias) {
    analogWrite(PWM_A, motorPower - motorBias);
    analogWrite(PWM_B, motorPower + motorBias);

    delay(500);

    nowTime = millis();
    leftDelta = leftTurns - lastLeftTurns;
    rightDelta = rightTurns - lastRightTurns;
    timeDelta = nowTime - lastCheckTime;

    leftSpeed = (double)leftDelta / (double)timeDelta * 1000.0;
    rightSpeed = (double)rightDelta / (double)timeDelta * 1000.0;

    lastCheckTime = nowTime;
    lastLeftTurns = leftTurns;
    lastRightTurns = rightTurns;

    Serial.print(motorPower);
    Serial.print(F("\t"));
    Serial.print(leftSpeed);
    Serial.print(F("\t"));
    Serial.print(rightSpeed);
    Serial.println();

    motorPower += 1;

//    resetDisplay();
//    display.print(motorPower);
//    display.display();
  }

  // Full break
  digitalWrite(BRK_A, HIGH);
  digitalWrite(BRK_B, HIGH);  
  analogWrite(PWM_A, 0);
  analogWrite(PWM_B, 0);

  Serial.println("Stop");
  Serial.print(leftTurns);
  Serial.print("\t");
  Serial.println(rightTurns);

//  resetDisplay();
//  display.print(leftTurns);
//  display.print(" | ");
//  display.println(rightTurns);
//  display.display();
}

void resetDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
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
      
      resetDisplay();
  
      if (command == COMMAND_FORWARDS) {
        display.println(F("Forwards:"));
        display.println(value);
        display.display();

        driveStraight(value, 200);
      } else if (command == COMMAND_BACKWARDS) {
        display.println(F("Backwards:"));
        display.println(value);
        display.display();
      } else if (command == COMMAND_ROTATE) {
        display.println(F("Rotate:"));
        display.println(value);
        display.display();
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
