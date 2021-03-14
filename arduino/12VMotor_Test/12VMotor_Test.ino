#include <Encoder.h>

const int PWM_A = 11;
const int PWM_B = 10;
const int DIR_A = 5;
const int BRK_A = 4;
const int DIR_B = 7;
const int BRK_B = 6;
const int ENC_B = 2;
const int ENC_A = 3;

Encoder encoder(ENC_A, ENC_B);

void setup() {
  pinMode(PWM_A, OUTPUT);
  pinMode(DIR_A, OUTPUT);
  pinMode(BRK_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(BRK_B, OUTPUT);

  Serial.begin(115200);

  delay(2000);

  runDriveTest();
}

void runDriveTest() {
  Serial.println("Begin test...");

  digitalWrite(BRK_A, LOW);
  digitalWrite(DIR_A, HIGH);
  digitalWrite(BRK_B, LOW);
  digitalWrite(DIR_B, HIGH);

  int motorPower = 1;

  while (motorPower <= 255) {
    analogWrite(PWM_A, motorPower);
    analogWrite(PWM_B, motorPower);

    Serial.print(motorPower);
    Serial.print(F("\t"));
    Serial.print(encoder.read());
    Serial.println();

    motorPower += 1;

    delay(200);
  }
}

void loop() {
  
}
