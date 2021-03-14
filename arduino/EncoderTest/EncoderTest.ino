const int ENC_L = 3;
const int ENC_R = 2;

volatile unsigned long leftTurns = 0;
volatile unsigned long rightTurns = 0;

void countLeft() {
  leftTurns++;
}

void countRight() {
  rightTurns++;
}

void setup() {
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L), countLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), countRight, RISING);

  Serial.begin(115200);
}

void loop() {
  Serial.print(leftTurns);
  Serial.print("\t");
  Serial.println(rightTurns);
  delay(50);
}
