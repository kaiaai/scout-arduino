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

// Hard-coded settings
const unsigned long maxDistance = 2UL; // ultra-sound sensors, meters
const word VCC_MEASUREMENT_PERIOD = 100; // report battery level
const int dly = 1;

static int vccRead (byte us =250);

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
  setMotorSpeed(false, 0);
  setMotorSpeed(true, 0);
  
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

word vccMeasurementCounter = 1;
volatile long int encL = 0, encR = 0;
long int encLTgt = 0, encRTgt = 0;
boolean encLTrack = false, encRTrack = false;
boolean encLBrake = false, encRBrake = false;
boolean encLTrackUp = true, encRTrackUp = true;
unsigned long int id = 0;

/*
void loop() {

  parseSerial();

  Serial.print("T");
  Serial.print(micros(), HEX);

  Serial.print(" L");
  Serial.print(encL, HEX); // TODO copy volatile var?

  Serial.print(" R");
  Serial.print(encR, HEX);

  unsigned int distance;
  distance = measureDistance(trigPinFront, echoPinFront);
  Serial.print(" f");
  Serial.print(distance, HEX);

  distance = measureDistance(trigPinLeft, echoPinLeft);
  Serial.print(" l");
  Serial.print(distance, HEX);

  distance = measureDistance(trigPinRight, echoPinRight);
  Serial.print(" r");
  Serial.print(distance, HEX);

  distance = measureDistance(trigPinBack, echoPinBack);
  Serial.print(" b");
  Serial.print(distance, HEX);

  distance = measureDistance(trigPinTop, echoPinTop);
  Serial.print(" t");
  Serial.print(distance, HEX);

  checkEncLDone();
  checkEncRDone();

  Serial.print((encLTrack || encRTrack) ? " I" : " i");
  Serial.print(id, HEX);

  vccMeasurementCounter = vccMeasurementCounter - 1;
  if (vccMeasurementCounter == 0) {
    vccMeasurementCounter = VCC_MEASUREMENT_PERIOD;
    analogRead(0);
    Serial.print(" V");
    Serial.print(vccRead(), HEX);
    Serial.print(FW_VERSION);
  }

  Serial.println();
  delay(dly);
}
*/

static void executeCmd(const char cmd, const long arg) {
  switch(cmd) {
    case 'i':
      id = arg;
      break;
    case 'L':
      setMotorSpeed(false, arg);
      break;
    case 'R':
      setMotorSpeed(true, arg);
      break;
    //case 'M':
    //  setMotorSpeed(false, arg);
    //  setMotorSpeed(true, arg);
    //  break;
    case 'g':
    case 'G':
      // Set encoder target, starting value, start tracking
      if (arg == 0) {
        encLTrack = false;
        break;
      }
      encLTgt = encL + arg;
      encLTrackUp = (arg > 0);
      encLTrack = true;
      encLBrake = (cmd == 'G');
      break;
    case 'h':
    case 'H':
      if (arg == 0) {
        encRTrack = false;
        break;
      }
      encRTgt = encR + arg;
      encRTrackUp = (arg > 0);
      encRTrack = true;
      encRBrake = (cmd == 'H');
      break;
  }
}

char cmd = 0;
long cmdArg = 0;
static void parseSerial() {
  while(Serial.available() > 0) {
    const char c = Serial.read();
    switch(c) {
      case 'L':
      case 'R':
      //case 'M':
      case 'g':
      case 'G':
      case 'h':
      case 'H':
      case 'i':
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
  const byte in1 = rightMotor ? MR2 : ML1;
  const byte in2 = rightMotor ? MR1 : ML2;
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

inline static int stopSpeed(bool brake) {
  return brake ? 0x8000 : 0;
}

inline static void checkEncLDone() {
  if (!encLTrack)
    return;
  encLTrack = encLTrackUp ? (encL < encLTgt) : (encL > encLTgt);
  // stop
  if (!encLTrack)
    setMotorSpeed(false, stopSpeed(encLBrake));
}

inline static void checkEncRDone() {
  if (!encRTrack)
    return;
  encRTrack = encRTrackUp ? (encR < encRTgt) : (encR > encRTgt);
  // stop
  if (!encRTrack)
    setMotorSpeed(true, stopSpeed(encRBrake));
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
  if (digitalRead(encLPinA) == digitalRead(encLPinB))
    encL--;
  else
    encL++;
}

static void encMRt() {
  // If pinA and pinB are both high or both low, it is spinning
  // forward. If they're different, it's going backward.
  //byte pins = PIND;
  if (digitalRead(encRPinA) == digitalRead(encRPinB))
    encR++;
  else
    encR--;
}

void loop() {

  parseSerial();

  String s =
    "T" + String(micros(), HEX) +
    " L" + String(encL, HEX) + 
    " R" + String(encR, HEX);
  
  unsigned int distance;
  distance = measureDistance(trigPinFront, echoPinFront);
  s = s + " f" + String(distance, HEX);

  distance = measureDistance(trigPinLeft, echoPinLeft);
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

  vccMeasurementCounter = vccMeasurementCounter - 1;
  if (vccMeasurementCounter == 0) {
    vccMeasurementCounter = VCC_MEASUREMENT_PERIOD;
    analogRead(0);
    s = s + " V" + String(vccRead(), HEX) + String(FW_VERSION);
  }

  Serial.println(s);
  //delay(dly);
}

