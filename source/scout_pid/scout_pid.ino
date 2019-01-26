// WORKS board v2
const char FW_VERSION[] = " v0.4.2";

// T........ time
// L........ left encoder
// R........ right encoder
// f.... forward distance cm
// l.... left distance cm
// r.... right distance cm
// b.... back distance cm
// t.... top distance cm
// I|i........ id (I == left or right encoder are tracking)
// V.... vcc
// v....?.... firmware version
// 309.3 Hui Dong
// 297.67 Etonm

const unsigned long int baud = 115200;

// Pin assignments - ultra-sound distance sensor
const byte trigPinFront = 14;
const byte echoPinFront = 13;
const byte trigPinRight = 15;
const byte echoPinRight = 12;
const byte trigPinLeft = 16;
const byte echoPinLeft = 8;
const byte trigPinBack = 17;
const byte echoPinBack = 19;
const byte trigPinTop = 18;
const byte echoPinTop = 11;

// Pin assignments - motor
const byte ML1 = 9;
const byte ML2 = 10;
const byte MR1 = 5;
const byte MR2 = 6;

// Pin assignments - motor encoder
const byte encLPinA = 2; // INT0
const byte encLPinB = 4;
const byte encRPinA = 3; // INT1
const byte encRPinB = 7;

const bool RIGHT_MOTOR = true;
const bool LEFT_MOTOR = false;

// Hard-coded settings
const unsigned long maxDistance = 2UL; // ultra-sound sensors, meters
//const word VCC_MEASUREMENT_PERIOD = 100; // report battery level
//const int dly = 1;

static int vccRead (byte us =250);
//volatile long int usIntLeft = 0, usIntRight = 0;      // time between encoder pulses
//volatile bool isIntLeft = false, isIntRight = false;  // is time reading available
//static long int usIntNowLeft = 0, usIntNowRight = 0;
//static long int usIntLastLeft = 0, usIntLastRight = 0;

static int userMotorSpeedLeft = 0, userMotorSpeedRight = 0; // set by user LR commands
static int kP = 0, kI = 0;
static long int errDistSlaveMotorInt;
//static long int usIntLeft2, usIntRight2; // time between encoder pulses
static bool useOld = false;

static bool masterMotor;
static long int distLeftMotor, distRightMotor;
static long int pwmSlaveMotor, pwmMasterMotor; //, dPwmMax;
static long int speedCorrSlaveMotor, speedCorrSlaveMotorInt, speedCorrSlaveMotorTotal;
static int userMotorSpeedMaster, userMotorSpeedSlave;
static long int distMasterMotor, distSlaveMotor, expDistSlaveMotor;
static long int errDistSlaveMotor;
const bool reverseMotorDir = true;
const bool reverseEncoderDir = true;

void setup() {
  // TODO pinMode(13, OUTPUT); // 13==D13

  // Ultra-sound distance sensors pin setup
  pinMode(trigPinFront, OUTPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(trigPinBack, OUTPUT);
  pinMode(trigPinTop, OUTPUT);

  pinMode(echoPinFront, INPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(echoPinBack, INPUT);
  pinMode(echoPinTop, INPUT);

  // Clears trig pins
  digitalWrite(trigPinFront, LOW);
  digitalWrite(trigPinLeft, LOW);
  digitalWrite(trigPinRight, LOW);
  digitalWrite(trigPinBack, LOW);
  digitalWrite(trigPinTop, LOW);

  // Clears echo pins
  digitalWrite(echoPinFront, LOW);
  digitalWrite(echoPinLeft, LOW);
  digitalWrite(echoPinRight, LOW);
  digitalWrite(echoPinBack, LOW);
  digitalWrite(echoPinTop, LOW);
  
  // Motors pin setup
  pinMode(ML1, OUTPUT);
  pinMode(ML2, OUTPUT);
  pinMode(MR1, OUTPUT);
  pinMode(MR2, OUTPUT);
  //pinMode(LED_BUILTIN, OUTPUT);
  // IN1 IN2 
  // H   H   Short brake
  // L   H   CCW
  // H   L   CW
  // L   L   Stop (high impedance)
  setMotorSpeed(LEFT_MOTOR, 0);
  setMotorSpeed(RIGHT_MOTOR, 0);
  
  // Motor encoder pin setup
  pinMode(encLPinA, INPUT);
  //digitalWrite(encLPinA, HIGH);
  pinMode(encLPinB, INPUT);
  //digitalWrite(encLPinB, HIGH);
  attachInterrupt(digitalPinToInterrupt(encLPinA),
    encMLt, CHANGE);

  pinMode(encRPinA, INPUT);
  //digitalWrite(encRPinA, HIGH);
  pinMode(encRPinB, INPUT);
  //digitalWrite(encRPinB, HIGH);
  attachInterrupt(digitalPinToInterrupt(encRPinA),
    encMRt, CHANGE);
  
  Serial.begin(baud);
}

inline static int stopSpeed(bool brake) {
  return brake ? 0x8000 : 0;
}

//static word vccMeasurementCounter = 1;
volatile long int encL = 0, encR = 0;
static long int encLTgt = 0, encRTgt = 0, encLStart = 0, encRStart = 0;
static boolean encLTrack = false, encRTrack = false;
static boolean encLBrake = false, encRBrake = false;
static boolean encLTrackUp = true, encRTrackUp = true;
static unsigned long int id = 0;

static void executeCmd(const char cmd, const long arg) {
Serial.print("executeCmd ");
Serial.print(cmd);
Serial.println(arg, HEX);
  switch(cmd) {
//    case 'D':
//      dPwmMax = arg;
//      break;
    case 'O':
      useOld = arg;
      break;
    case 'i':
      id = arg;
      break;
    case 'L':
      setMotorSpeed(LEFT_MOTOR, arg);
      userMotorSpeedLeft = arg;
      break;
    case 'R':
      setMotorSpeed(RIGHT_MOTOR, arg);
      userMotorSpeedRight = arg;
      break;
    case 'P':
      kP = arg;
      break;
    case 'I':
      kI = arg;
      errDistSlaveMotorInt = 0;
      break;
    //case 'M':
    //  setMotorSpeed(LEFT_MOTOR, arg);
    //  setMotorSpeed(RIGHT_MOTOR, arg);
    //  break;
    case 'g':
    case 'G':
      // Set encoder target, starting value, start tracking
      encLBrake = (cmd == 'G');
      if (arg == 0) {
        encLTrack = false;
        if (userMotorSpeedLeft == 0)
          setMotorSpeed(LEFT_MOTOR, stopSpeed(encLBrake));
        break;
      }
      encLStart = encL;
      encLTgt = encLStart + arg;
      encLTrackUp = (arg > 0);
      encLTrack = true;
      break;
    case 'h':
    case 'H':
      encRBrake = (cmd == 'H');
      if (arg == 0) {
        encRTrack = false;
        if (userMotorSpeedRight == 0)
          setMotorSpeed(RIGHT_MOTOR, stopSpeed(encRBrake));
        break;
      }
      encRStart = encR;
      encRTgt = encRStart + arg;
      encRTrackUp = (arg > 0);
      encRTrack = true;
      break;
  }
}

char cmd = 0;
long cmdArg = 0;
static void parseSerial() {
  while(Serial.available() > 0) {
    const char c = Serial.read();
    switch(c) {
//      case 'D':
      case 'O':
      case 'L':
      case 'R':
      //case 'M':
      case 'g':
      case 'G':
      case 'h':
      case 'H':
      case 'i':
      case 'I':
      case 'P':
        cmd = c;
        cmdArg = 0;
        break;
      default:
        if (c >= '0' && c <= '9') {
          cmdArg = (cmdArg << 4) + c - '0';
        } else if (c >= 'a' && c <= 'f') {
          cmdArg = (cmdArg << 4) + c - 'a' + 10;
        } else {
          if (cmd != 0) {
            executeCmd(cmd, cmdArg);
            cmd = 0;
          }
        }
    }
  }
}

const unsigned long maxDuration = (maxDistance*2*1000*1000)/343;
word measureDistance(const byte trigPin, const byte echoPin) {  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  unsigned long duration = pulseInLong(echoPin, HIGH, maxDuration);

  if (duration == 0) {
    pinMode(echoPin, OUTPUT);
    delayMicroseconds(30); // magic number 20...30
    pinMode(echoPin, INPUT);
    return 0;
  }
  
  // Calculating the distance
  word distance =  (duration*343)/2000; // duration*0.0343/2;
  return distance;
}

static void setMotorSpeed(const boolean rightMotor, const int speed) {
  const byte in1 = reverseMotorDir ? (rightMotor ? MR1 : ML2) : (rightMotor ? MR2 : ML1);
  const byte in2 = reverseMotorDir ? (rightMotor ? MR2 : ML1) : (rightMotor ? MR1 : ML2);
  if (speed == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else if (speed > 0 && speed <= 255) {
    digitalWrite(in1, HIGH);
    analogWrite(in2, 255-speed);
  } else if (speed < -255) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  } else if (speed < 0) {
    digitalWrite(in2, HIGH);
    analogWrite(in1, 255+speed);
  }
}

inline static void checkEncLDone() {
  if (!encLTrack)
    return;
  encLTrack = encLTrackUp ? (encL < encLTgt) : (encL > encLTgt);
  // stop
  if (!encLTrack) {
    if (kP != 0) {
      encRTrack = false;
      setMotorSpeed(RIGHT_MOTOR, stopSpeed(encRBrake));
    }
    kP = 0;
    kI = 0;
    setMotorSpeed(LEFT_MOTOR, stopSpeed(encLBrake));
  }
}

inline static void checkEncRDone() {
  if (!encRTrack)
    return;
  encRTrack = encRTrackUp ? (encR < encRTgt) : (encR > encRTgt);
  // stop
  if (!encRTrack) {
    if (kP != 0) {
      encLTrack = false;
      setMotorSpeed(LEFT_MOTOR, stopSpeed(encLBrake));
    }
    kP = 0;
    kI = 0;
    setMotorSpeed(RIGHT_MOTOR, stopSpeed(encRBrake));
  }
}

static int vccRead (byte us) {
  analogRead(6);
  bitSet(ADMUX, 3);
  delayMicroseconds(us);
  bitSet(ADCSRA, ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  word x = ADC;
  return x ? (1100L * 1023) / x : -1;
}

static void encMLt() {
  // If pinA and pinB are both high or both low, it is spinning
  // forward. If they're different, it's going backward.
  //byte pins = PIND;
  if ((digitalRead(encLPinA) != digitalRead(encLPinB)) != reverseEncoderDir) {
    encL--;
//    usIntNowLeft = micros();
//    usIntLeft = usIntLastLeft - usIntNowLeft;
  } else {
    encL++;
//    usIntNowLeft = micros();
//    usIntLeft = usIntNowLeft - usIntLastLeft;
  }
//  isIntLeft = true; // measurement available flag
//  usIntLastLeft = usIntNowLeft;
}

static void encMRt() {
  // If pinA and pinB are both high or both low, it is spinning
  // forward. If they're different, it's going backward.
  //byte pins = PIND;
  if ((digitalRead(encRPinA) != digitalRead(encRPinB)) != reverseEncoderDir) {
    encR++;
//    usIntNowRight = micros();
//    usIntRight = usIntNowRight - usIntLastRight;
  } else {
    encR--;
//    usIntNowRight = micros();
//    usIntRight = usIntLastRight - usIntNowRight;
  }
//  isIntRight = true; // measurement available flag
//  usIntLastRight = usIntNowRight;
}

long int encRtmp, encLtmp;
void pid() {
  // L80 R80 G1000 H1000 P100 I10 
  // L40 R80 G800 H1000 P100 I10 
  // L80 R40 G1000 H800 P100 I10 
  // Lff80 Rffa0 Gfffff000 Hfffff000 P100 I10 
  // Lff80 Rff80 Gfffff000 Hfffff000 P100 I10 
  // if starting set starting motor speeds to user speed values

  encRtmp = encR;
  encLtmp = encL;
  distLeftMotor = encLtmp - encLStart;
  distRightMotor = encRtmp - encRStart;

  Serial.print("{\"func\":\"pid\",\"time\":");
  Serial.print(micros());

  // pick fastest motor to be master
  // TODO check speeds with opposite signs
  // if (abs(userMotorSpeedLeft) >= abs(userMotorSpeedRight)) {
  if (abs(userMotorSpeedLeft) > abs(userMotorSpeedRight)) {
    distMasterMotor = distLeftMotor;
    distSlaveMotor = distRightMotor;
    userMotorSpeedMaster = userMotorSpeedLeft;
    userMotorSpeedSlave = userMotorSpeedRight;
    masterMotor = LEFT_MOTOR;
    Serial.print(",\"slave\":\"R\\");
  } else {
    distMasterMotor = distRightMotor;
    distSlaveMotor = distLeftMotor;
    userMotorSpeedMaster = userMotorSpeedRight;
    userMotorSpeedSlave = userMotorSpeedLeft;
    masterMotor = RIGHT_MOTOR;
    Serial.print(",\"slave\":\"L\\");
  }
  Serial.print(",\"encR\":");
  Serial.print(encRtmp);
  Serial.print(",\"encRStart\":");
  Serial.print(encRStart);
  Serial.print(",\"encL\":");
  Serial.print(encLtmp);
  Serial.print(",\"encLStart\":");
  Serial.print(encLStart);
  Serial.print(",\"distMasterMotor\":");
  Serial.print(distMasterMotor);
  Serial.print(",\"distSlaveMotor\":");
  Serial.print(distSlaveMotor);
  Serial.print(",\"kP\":");
  Serial.print(kP);
  
  // expected distance of slower motor = (distance of faster motor) *
  // (user faster motor speed) / (user slower motor speed)
  // TODO pre-divide
  expDistSlaveMotor = distMasterMotor * userMotorSpeedSlave / userMotorSpeedMaster;

  // distance error = (distance of slower motor) - (expected distance of slower motor)
  errDistSlaveMotor = distSlaveMotor - expDistSlaveMotor;

  Serial.print(",\"expDistSlaveMotor\":");
  Serial.print(expDistSlaveMotor);
  Serial.print(",\"errDistSlaveMotor\":");
  Serial.print(errDistSlaveMotor);

  // Integrate distance error
  errDistSlaveMotorInt = errDistSlaveMotorInt + errDistSlaveMotor;

  // (slower motor pwm speed) = clamp( user speed value - (distance_error)*k )
  speedCorrSlaveMotor = errDistSlaveMotor * kP;
  speedCorrSlaveMotorInt = errDistSlaveMotorInt * kI;
  speedCorrSlaveMotorTotal = speedCorrSlaveMotor + speedCorrSlaveMotorInt;

  Serial.print(",\"errDistSlaveMotorInt\":");
  Serial.print(errDistSlaveMotorInt);
  Serial.print(",\"speedCorrSlaveMotor\":");
  Serial.print(speedCorrSlaveMotor >> 8);
  Serial.print(",\"speedCorrSlaveMotorInt\":");
  Serial.print(speedCorrSlaveMotorInt >> 8);
  Serial.print(",\"speedCorrSlaveMotorTotal\":");
  Serial.print(speedCorrSlaveMotorTotal >> 8);

  // slave (slower) motor PWM
  pwmSlaveMotor = userMotorSpeedSlave -
    (((speedCorrSlaveMotorTotal * userMotorSpeedSlave) / userMotorSpeedMaster) >> 8);
  pwmSlaveMotor = (pwmSlaveMotor > 255) ? 255 : pwmSlaveMotor;
  pwmSlaveMotor = (pwmSlaveMotor < -255) ? -255 : pwmSlaveMotor;
  
//  if (pwmSlaveMotor > userMotorSpeedSlave && pwmSlaveMotor > (userMotorSpeedSlave + dPwmMax))
//    pwmSlaveMotor = userMotorSpeedSlave + dPwmMax;
//  else if (pwmSlaveMotor < userMotorSpeedSlave && pwmSlaveMotor < (userMotorSpeedSlave - dPwmMax))
//    pwmSlaveMotor = userMotorSpeedSlave - dPwmMax;

  
  // master (faster) motor PWM
  pwmMasterMotor = userMotorSpeedMaster +
    (((speedCorrSlaveMotorTotal * userMotorSpeedMaster) / userMotorSpeedSlave) >> 8);
  pwmMasterMotor = (pwmMasterMotor > 255) ? 255 : pwmMasterMotor;
  pwmMasterMotor = (pwmMasterMotor < -255) ? -255 : pwmMasterMotor;

//  if (pwmMasterMotor > userMotorSpeedMaster && pwmMasterMotor > (userMotorSpeedMaster + dPwmMax))
//    pwmMasterMotor = userMotorSpeedMaster + dPwmMax;
//  else if (pwmMasterMotor < userMotorSpeedMaster && pwmMasterMotor < (userMotorSpeedMaster - dPwmMax))
//    pwmMasterMotor = userMotorSpeedMaster - dPwmMax;


  // Disallow master motor reversing rotation direction
  if (!(userMotorSpeedMaster > 0 && pwmMasterMotor >= 0 ||
      userMotorSpeedMaster < 0 && pwmMasterMotor <= 0)) {
    Serial.print(",\"pwmMasterReverseBlocked\":1");
    pwmMasterMotor = 0;
  } else
    Serial.print(",\"pwmMasterReverseBlocked\":0");

  Serial.print(",\"pwmMasterMotor\":");
  Serial.print(pwmMasterMotor);
  Serial.print(",\"pwmSlaveMotor\":");
  Serial.print(pwmSlaveMotor);
  Serial.print("}");

  if (kP == 0)
    return;

  // Set motor speed  
  setMotorSpeed(masterMotor, pwmMasterMotor);
  setMotorSpeed(!masterMotor, pwmSlaveMotor);
}

void pidLeft() {
  // L80 R80 G1000 H1000 P600 I80  // P400
  // L80 R40 G1000 H800 P600 I80 
  // Lff80 Rff80 Gfffff000 Hfffff000 P600 I80 
  // if starting set starting motor speeds to user speed values
  encRtmp = encR;
  encLtmp = encL;
  distMasterMotor = encRtmp - encRStart;
  distSlaveMotor = encLtmp - encLStart;

  Serial.print("{\"func\":\"pidLeft\",\"time\":");
  Serial.print(micros());
  Serial.print(",\"encR\":");
  Serial.print(encRtmp);
  Serial.print(",\"encRStart\":");
  Serial.print(encRStart);
  Serial.print(",\"distMasterMotor\":");
  Serial.print(distMasterMotor);
  Serial.print(",\"encL\":");
  Serial.print(encLtmp);
  Serial.print(",\"encLStart\":");
  Serial.print(encLStart);
  Serial.print(",\"distSlaveMotor\":");
  Serial.print(distSlaveMotor);
  Serial.print(",\"kP\":");
  Serial.print(kP);

  // expected distance of slower motor = (distance of faster motor) *
  // (user faster motor speed) / (user slower motor speed)
  // TODO pre-divide
  expDistSlaveMotor = distMasterMotor * userMotorSpeedLeft / userMotorSpeedRight;
  Serial.print(",\"expDistSlaveMotor\":");
  Serial.print(expDistSlaveMotor);

  // distance error = (distance of slower motor) - (expected distance of slower motor)
  errDistSlaveMotor = distSlaveMotor - expDistSlaveMotor;
//  uncomment for debug

  errDistSlaveMotorInt = errDistSlaveMotorInt + errDistSlaveMotor;

  // (slower motor pwm speed) = clamp( user speed value - (distance_error)*k )
  speedCorrSlaveMotor = errDistSlaveMotor * kP;
  speedCorrSlaveMotorInt = errDistSlaveMotorInt * kI;
  pwmSlaveMotor = userMotorSpeedLeft -
    ((speedCorrSlaveMotor + speedCorrSlaveMotorInt) >> 8);
  pwmSlaveMotor = (pwmSlaveMotor > 255) ? 255 : pwmSlaveMotor;
  pwmSlaveMotor = (pwmSlaveMotor < -255) ? -255 : pwmSlaveMotor;

//  if (pwmSlaveMotor > userMotorSpeedLeft && pwmSlaveMotor > (userMotorSpeedLeft + dPwmMax))
//    pwmSlaveMotor = userMotorSpeedLeft + dPwmMax;
//  else if (pwmSlaveMotor < userMotorSpeedLeft && pwmSlaveMotor < (userMotorSpeedLeft - dPwmMax))
//    pwmSlaveMotor = userMotorSpeedLeft - dPwmMax;
  
  Serial.print(",\"errDistSlaveMotor\":");
  Serial.print(errDistSlaveMotor);
  Serial.print(",\"errDistSlaveMotorInt\":");
  Serial.print(errDistSlaveMotorInt);
  Serial.print(",\"speedCorrSlaveMotor\":");
  Serial.print(speedCorrSlaveMotor >> 8);
  Serial.print(",\"speedCorrSlaveMotorInt\":");
  Serial.print(speedCorrSlaveMotorInt >> 8);
  Serial.print(",\"pwmSlaveMotor\":");
  Serial.print(pwmSlaveMotor);
  Serial.println("}");

  if (kP == 0)
    return;

  setMotorSpeed(LEFT_MOTOR, pwmSlaveMotor);
}

void pidRight() {
  // L40 R80 G800 H1000 P400 I80 
  // Lff80 Rffa0 Gfffff000 Hfffff000 P400 
  // if starting set starting motor speeds to user speed values
  encRtmp = encR;
  encLtmp = encL;
  distMasterMotor = encLtmp - encLStart;
  distSlaveMotor = encRtmp - encRStart;

  Serial.print("{\"func\":\"pidLeft\",\"time\":");
  Serial.print(micros());
  Serial.print(",\"encR\":");
  Serial.print(encRtmp);
  Serial.print(",\"encRStart\":");
  Serial.print(encRStart);
  Serial.print(",\"distMasterMotor\":");
  Serial.print(distMasterMotor);
  Serial.print(",\"encL\":");
  Serial.print(encLtmp);
  Serial.print(",\"encLStart\":");
  Serial.print(encLStart);
  Serial.print(",\"distSlaveMotor\":");
  Serial.print(distSlaveMotor);
  Serial.print(",\"kP\":");
  Serial.print(kP);

  // expected distance of slower motor = (distance of faster motor) *
  // (user faster motor speed) / (user slower motor speed)
  // TODO pre-divide
  expDistSlaveMotor = distMasterMotor * userMotorSpeedRight / userMotorSpeedLeft;
  Serial.print(",\"expDistSlaveMotor\":");
  Serial.print(expDistSlaveMotor);

  // distance error = (distance of slower motor) - (expected distance of slower motor)
  errDistSlaveMotor = distSlaveMotor - expDistSlaveMotor;
//  uncomment for debug

  errDistSlaveMotorInt = errDistSlaveMotorInt + errDistSlaveMotor;

  // (slower motor pwm speed) = clamp( user speed value - (distance_error)*k )
  speedCorrSlaveMotor = errDistSlaveMotor * kP;
  speedCorrSlaveMotorInt = errDistSlaveMotorInt * kI;
  pwmSlaveMotor = userMotorSpeedRight -
    ((speedCorrSlaveMotor + speedCorrSlaveMotorInt) >> 8);
  pwmSlaveMotor = (pwmSlaveMotor > 255) ? 255 : pwmSlaveMotor;
  pwmSlaveMotor = (pwmSlaveMotor < -255) ? -255 : pwmSlaveMotor;

//  if (pwmSlaveMotor > userMotorSpeedRight && pwmSlaveMotor > (userMotorSpeedRight + dPwmMax))
//    pwmSlaveMotor = userMotorSpeedRight + dPwmMax;
//  else if (pwmSlaveMotor < userMotorSpeedRight && pwmSlaveMotor < (userMotorSpeedRight - dPwmMax))
//    pwmSlaveMotor = userMotorSpeedRight - dPwmMax;
  
  Serial.print(",\"errDistSlaveMotor\":");
  Serial.print(errDistSlaveMotor);
  Serial.print(",\"errDistSlaveMotorInt\":");
  Serial.print(errDistSlaveMotorInt);
  Serial.print(",\"speedCorrSlaveMotor\":");
  Serial.print(speedCorrSlaveMotor >> 8);
  Serial.print(",\"speedCorrSlaveMotorInt\":");
  Serial.print(speedCorrSlaveMotorInt >> 8);
  Serial.print(",\"pwmSlaveMotor\":");
  Serial.print(pwmSlaveMotor);
  Serial.println("}");

  if (kP == 0)
    return;

  setMotorSpeed(RIGHT_MOTOR, pwmSlaveMotor);
}


void loop() {

  parseSerial();

  String s =
    "T" + String(micros(), HEX) +
    " L" + String(encL, HEX) + 
    " R" + String(encR, HEX);

//  if (isIntLeft) {
//    usIntLeft2 = usIntLeft;
//    s = s + " S" + String(usIntLeft2, HEX);
//    isIntLeft = false;
//  }
//
//  if (isIntRight) {
//    usIntRight2 = usIntRight;
//    s = s + " s" + String(usIntRight2, HEX);
//    isIntRight = false;
//  }

  if (useOld) {
    if ((userMotorSpeedLeft != 0) && (userMotorSpeedRight != 0)) {
      if (abs(userMotorSpeedLeft) <= abs(userMotorSpeedRight))
        pidLeft();
      else
        pidRight();
    }
  } else {
    //if ((kP != 0) && (userMotorSpeedLeft != 0) && (userMotorSpeedRight != 0)) {
    if ((userMotorSpeedLeft != 0) && (userMotorSpeedRight != 0))
      pid();
  }
  
  unsigned int distance;
  distance = 0; //measureDistance(trigPinFront, echoPinFront);
  s = s + " f" + String(distance, HEX);

  distance = 0; //measureDistance(trigPinLeft, echoPinLeft);
  s = s + " l" + String(distance, HEX);

  distance = measureDistance(trigPinRight, echoPinRight);
  s = s + " r" + String(distance, HEX);

  distance = measureDistance(trigPinBack, echoPinBack);
  s = s + " b" + String(distance, HEX);

  distance = measureDistance(trigPinTop, echoPinTop);
  s = s + (" t") + String(distance, HEX);

  checkEncLDone();
  checkEncRDone();

  s = s + ((encLTrack || encRTrack) ? " I" : " i") + String(id, HEX);

  //vccMeasurementCounter = vccMeasurementCounter - 1;
  //if (vccMeasurementCounter == 0) {
  //vccMeasurementCounter = VCC_MEASUREMENT_PERIOD;
  analogRead(0);
  s = s + " V" + String(vccRead(), HEX) + String(FW_VERSION);
  //}

  Serial.println(s);
  //delay(dly);
}

