#define Kp                900L
#define Ki                1800L
#define Kd                0L
#define SYSTEM_BIAS       150L
#define MAX_ADJUSTMENT    100

const int PWM_A = 5;
const int PWM_B = A2;
const int DIR_A = A1;
const int DIR_B = A0;
const int BRK_A = A3;
const int BRK_B = 6;
const int ENC_L = 3;
const int ENC_R = 2;

const int POWER_OFFSET = 125;         // Motors don't respond below this value
const int SAMPLE_TIME = 200;          // PID sampling in ms

int turnSpeed           = 60;     // 0..125
int moveSpeed           = 60;     // 0..125

volatile int16_t en_lft_ticks = 0;
volatile int16_t en_rht_ticks = 0;
volatile bool en_error = false;
long en_lastCall = 0;

int16_t pid_lastErr;
int16_t pid_sumErrs;
uint16_t pid_time;
int16_t pid_total_lticks;
int16_t pid_total_rticks;

// PID - use this make the left motor more
// powerful or less pwerful than the right motor.
// Value is as a percentage, i.e. 110 means left
// motor will be 10% faster than the right. 90
// would mean the left motor is 10% slower
int LMotorGain    = 100;
int adjustLMotor  = 0;
int adjustRMotor  = 0;
int LMotorSpeed   = 0;
int RMotorSpeed   = 0;
bool LMotorDirection = true;
bool RMotorDirection = true;

void countLeft() {
  en_lft_ticks++;
}

void countRight() {
  en_rht_ticks++;
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

  Serial.begin(115200);

  delay(5000);

  pid_move(10000);
}

void clear_ticks()
{
  cli();
  en_lft_ticks = en_rht_ticks = 0;
  en_error = false;
  en_lastCall = millis();
  sei();
}

bool get_ticks_since_last( int16_t *lft, int16_t *rht, uint16_t *ms ) {
  cli();
  *lft = en_lft_ticks;
  *rht = en_rht_ticks;
  long now = millis();
  *ms = (uint16_t)(now - en_lastCall);
  en_lastCall = now;
  en_lft_ticks = en_rht_ticks = 0;
  char error = en_error;
  en_error = false;
  sei();

  return !error;
}

void resetPID()
{
  pid_total_lticks = 0;
  pid_total_rticks = 0;
  pid_lastErr = 0;
  pid_sumErrs = 0;
  adjustLMotor = adjustRMotor = 0;
  updateMotors();
  clear_ticks();
  pid_time = 0;

  Serial.println("PID Reset");
//  Serial.println(SYSTEM_BIAS);
//  Serial.println("Time, Interval, Left Ticks, Right Ticks, Error, Sum Erros, Adjust, Left, Adjust Right");
}

void driveStraight()
{
  static int16_t lticks = 0, rticks = 0;
  static uint16_t ms = 0;
  int16_t dlticks, drticks, diff;
  int32_t delta;
  uint16_t dms;
  
  get_ticks_since_last( &dlticks, &drticks, &dms);

  lticks += dlticks;
  rticks += drticks;
  pid_total_lticks += dlticks;
  pid_total_rticks += drticks;
  ms += dms;
  pid_time += dms;

  if ( ms > 200 )
  {
    int16_t rdir = rticks < 0 ? -1 : 1;
    int16_t ldir = lticks < 0 ? -1 : 1;

    // make the values positive
    lticks *= ldir;
    rticks *= rdir;

    int16_t bias = (rticks * SYSTEM_BIAS) / 10000L;
    diff = ((lticks - rticks + bias ) * 100L)/ms;

    // we want the difference to be 0

    // track the integral 
    pid_sumErrs += diff;

    // get the differential
    delta = (int32_t) (diff - pid_lastErr);

    int16_t P = (int16_t) ((Kp*((int32_t)diff) + Ki*((int32_t)pid_sumErrs) + (Kd*delta))/1000L);

    pid_lastErr = diff;

    // a positive error means the left motor is 
    // turning more than the right so adjust 
    // each motor accordingly
    int16_t adjust = (P>>1);
    adjustLMotor -= adjust*ldir;
    adjustRMotor += adjust*rdir;

    // Put a limit on the total adjustment in case PID gets out of control
    constrain( adjustLMotor, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);
    constrain( adjustRMotor, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);

    Serial.print(pid_time); Serial.print("\t");
    Serial.print(ms); Serial.print("\t");
    Serial.print(lticks); Serial.print("\t");
    Serial.print(rticks); Serial.print("\t");
    Serial.print(diff); Serial.print("\t");
    Serial.print(pid_sumErrs); Serial.print("\t");
    Serial.print(adjustLMotor); Serial.print("\t");
    Serial.print(adjustRMotor);
    Serial.println();

    updateMotors();
    lticks = 0;
    rticks = 0;
    ms = 0;
  }
}

#define WHEEL_DIAM    64.0f   // mm
#define TRACK_WIDTH   140.0f   // mm
#define TICKS_PER_REV 20.0f
#define MM_PER_TICK (((WHEEL_DIAM*PI)/TICKS_PER_REV))
#define MM_TO_TICKS(A) ((float)(A))/MM_PER_TICK
#define MM_PER_DEGREES(D) ((TRACK_WIDTH*PI*(D))/360.0f)

int16_t pid_target_ticks;
int16_t pid_lmotor_speed = 0;
int16_t pid_rmotor_speed = 0;
int16_t pid_motor_step = 20;
uint32_t pid_last_motor_update;

void releaseBreaks() {
  digitalWrite(BRK_A, LOW);
  digitalWrite(BRK_B, LOW);
}

void pid_move( int16_t mm)
{
  resetPID();
  releaseBreaks();
  pid_target_ticks = (int16_t) MM_TO_TICKS(mm);
  pid_lmotor_speed = pid_rmotor_speed = moveSpeed;
  pid_last_motor_update = millis();
  Serial.print("Moving: ");
  Serial.print(mm);
  Serial.print(" mm - ("); 
  Serial.print(pid_target_ticks);
  Serial.println(" ticks)");
}

void pid_turn( int16_t degrees)
{
  resetPID();
  pid_target_ticks = (int16_t) MM_TO_TICKS( MM_PER_DEGREES(degrees));

  int16_t speed = degrees < 0 ? turnSpeed : -turnSpeed;

  pid_lmotor_speed = speed;
  pid_rmotor_speed = -speed;
  pid_last_motor_update = millis();

  Serial.print("Turning: ");
  Serial.print(degrees);
  Serial.print(" d - ("); 
  Serial.print(pid_target_ticks);
  Serial.println(" ticks)");
}

void loop() {
  if ( LMotorSpeed < moveSpeed )
  {
    LMotorSpeed += 20;
    RMotorSpeed = LMotorSpeed;
    updateMotors();
  }
  
  driveStraight();

  delay(100);
}

void updateMotors()
{
  static int lastLSpeed = 0;
  static int lastRSpeed = 0;
  
  // LMotorGain is used to simulate the
  // left motor as being more or less powerful
  // than the right. We use this to test the PID
  // controler. You can set LMotorGain and
  // see how well PID can correct the motor
  // imbalance

  int lspeed = (((LMotorSpeed + adjustLMotor) * LMotorGain) / 100) * LMotorDirection;
  int rspeed = (RMotorSpeed + adjustRMotor) * RMotorDirection;

  constrain(lspeed, -POWER_OFFSET, POWER_OFFSET);
  constrain(rspeed, -POWER_OFFSET, POWER_OFFSET);

  if (lastLSpeed != lspeed)
  {
    LMotorDirection = (lspeed > 0);

    digitalWrite(DIR_B, LMotorDirection ? LOW : HIGH);
    analogWrite(PWM_B, abs(lspeed) + POWER_OFFSET);
    
    lastLSpeed = lspeed;
  }

  if (lastRSpeed != rspeed)
  {
    RMotorDirection = (rspeed > 0);
    
    digitalWrite(DIR_A, LMotorDirection ? LOW : HIGH);
    analogWrite(PWM_A, abs(rspeed) + POWER_OFFSET);

    lastRSpeed = rspeed;
  }
}
