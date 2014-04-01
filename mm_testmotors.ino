class Pair {
  public:
   float a, b;
   
   Pair(float _a, float _b) {
    a = _a;
    b = _b;
  };
};

/*
2 : encoder R
3 : bump B
4 : encoder R
5 : output R
6 : output R
7 : encoder L
8 : encoder L
9 : output L
10: output L
11: bump L
12: bump R
*/

unsigned long microsPerStateChange = 1000000 / 360; // 1 m/s
float targetVoltage = 0;
float speedPerVolt = 0.48148148148148148148148148148148;
float mps = 0;

int prevStateL = 0;
unsigned long timeL;

int prevStateR = 0;
unsigned long timeR;

//// Pins
// Digital
const char encoderL1 = 7;
const char encoderL2 = 8;
const char encoderR1 = 2;
const char encoderR2 = 4;
const char bumpL = 11;
const char bumpR = 12;
const char bumpB = 3;
const char outputL1 = 9;
const char outputL2 = 10;
const char outputR1 = 5;
const char outputR2 = 6;

// Analog
const char distF = 0;

bool forward = true;

void setup() {
  pinMode(encoderL1, INPUT);
  pinMode(encoderL2, INPUT);
  pinMode(encoderR1, INPUT);
  pinMode(encoderR2, INPUT);
  pinMode(bumpB, INPUT);
  pinMode(bumpL, INPUT);
  pinMode(bumpR, INPUT);
  pinMode(outputL1, OUTPUT);
  pinMode(outputL2, OUTPUT);
  pinMode(outputR1, OUTPUT);
  pinMode(outputR2, OUTPUT);
  
  timeL = micros();
  timeR = micros();
  
  runAtSpeed(1);
  
  Serial.begin(9600);
}

void loop() {
  //See collision, turn left
  //stepDistSensors();
  //runAtSpeed((float)analogRead(0) / 1024);
  //moveF(1000);
  /*
  moveF(500);
  analogWrite(outputL1, 0);
  analogWrite(outputL2, 0);
  analogWrite(outputR1, 0);
  analogWrite(outputR2, 0);
  delay(1000);
  turnLDeg(90);
  analogWrite(outputL1, 0);
  analogWrite(outputL2, 0);
  analogWrite(outputR1, 0);
  analogWrite(outputR2, 0);
  delay(1000);
  */
  //Bump into walls, move backwards
  if(forward) {
    moveF(10);
  } else {
    moveB(10);
  }
  //stepWheelL(forward);
  //stepWheelR(forward);
  stepBumpSensors();
}

void moveF(unsigned long ms) {
  unsigned long t = millis() + ms;
  while(millis() < t) {
    Pair pL = stepWheelL(true);
    Pair pR = stepWheelR(true);
    analogWrite(outputL1, pL.a);
    analogWrite(outputL2, pL.b);
    analogWrite(outputR1, pR.a);
    analogWrite(outputR2, pR.b);
  }
}

void moveB(unsigned long ms) {
  unsigned long t = millis() + ms;
  while(millis() < t) {
    Pair pL = stepWheelL(false);
    Pair pR = stepWheelR(false);
    analogWrite(outputL1, pL.a);
    analogWrite(outputL2, pL.b);
    analogWrite(outputR1, pR.a);
    analogWrite(outputR2, pR.b);
  }
}

void turnLDeg(float deg) {
  // Physics doesn't work, time for magic
  //float t = ((deg * PI / 180) / 4.6171298405466970387243735763098 - mps) / 0.981;
  float C = 1;
  float t = (deg * PI / 180) * C / mps;
  turnL(t * 1000);
}

void turnRDeg(float deg) {
  float t = ((deg * PI / 180) / 4.6171298405466970387243735763098 - mps) / 0.981;
  turnR(t * 1000);
}

void turnL(unsigned long ms) {
  unsigned long t = millis() + ms;
  while(millis() < t) {
    Pair pL = stepWheelL(false);
    Pair pR = stepWheelR(true);
    analogWrite(outputL1, pL.a);
    analogWrite(outputL2, pL.b);
    analogWrite(outputR1, pR.a);
    analogWrite(outputR2, pR.b);
  }
}

void turnR(unsigned long ms) {
  unsigned long t = millis() + ms;
  while(millis() < t) {
    Pair pL = stepWheelL(false);
    Pair pR = stepWheelR(true);
    analogWrite(outputL1, pL.a);
    analogWrite(outputL2, pL.b);
    analogWrite(outputR1, pR.a);
    analogWrite(outputR2, pR.b);
  }
}

void stepDistSensors() {
  /*
  Voltage to distance graph corresponds approx. to:
  V = 18.666666666666666666666666666667 / x
  where V is volts and x is in cm
  */
  float dist = 18.666666666 / (analogRead(distF) * 5 / 1024);
  // dist is in cm
}

void stepBumpSensors() {
  //int forward;
  if (digitalRead(bumpB) == HIGH) {
    forward = true;
  }
  if (digitalRead(bumpL) == HIGH || digitalRead(bumpR) == HIGH) {
    forward = false;
  }
}

class Pair stepWheelL(bool forward) {
  Pair pair(0, 0);
  int state = getStateL();
  if (state != prevStateL) {
    // calculate time elapsed since last state change
    unsigned long microsElapsed = micros() - timeL;
    // adjust speed so as to match target time
    float error = (microsElapsed - microsPerStateChange) / microsPerStateChange; // positive == too long, negative == too short
    byte v = (byte)(((targetVoltage / 5) + error) * 255);
    pair = Pair(forward ? v : 0, forward ? 0 : v);
    prevStateL = state;
    timeL = micros();
  } else {
    // if state is not changing after a long time something is terribly wrong
    if (micros() - timeL > microsPerStateChange * 2) {
      pair = Pair(forward ? 255 : 0, forward ? 0 : 255);
    }
  }
  return pair;
}

class Pair stepWheelR(bool forward) {
  Pair pair(0, 0);
  int state = getStateR();
  if (state != prevStateR) {
    // calculate time elapsed since last state change
    unsigned long microsElapsed = micros() - timeR;
    // adjust speed so as to match target time
    float error = (microsElapsed - microsPerStateChange) / microsPerStateChange; // positive == too long, negative == too short
    byte v = (byte)(((targetVoltage / 5) + error) * 255);
    pair = Pair(forward ? v : 0, forward ? 0 : v);
    prevStateR = state;
    timeR = micros();
  } else {
    // if state is not changing after a long time something is terribly wrong
    if (micros() - timeR > microsPerStateChange * 2) {
      pair = Pair(forward ? 255 : 0, forward ? 0 : 255);
    }
  }
  return pair;
}

int getStateL() {
  int a = (digitalRead(encoderL1) == HIGH) ? 1 : 0;
  int b = ((digitalRead(encoderL2) == HIGH) ? 1 : 0) ^ a;
  return a * 2 + b;
}

int getStateR() {
  int a = (digitalRead(encoderR1) == HIGH) ? 1 : 0;
  int b = ((digitalRead(encoderR2) == HIGH) ? 1 : 0) ^ a;
  return a * 2 + b;
}

void runAtSpeed(float _mps) {
  // convert mps to microseconds / count
  mps = _mps;
  microsPerStateChange = 1000000 / (_mps * 360);
  targetVoltage = _mps / speedPerVolt;
}
