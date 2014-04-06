class Pair {
  public:
   float a, b;
   
   Pair(float _a, float _b) {
    a = _a;
    b = _b;
  };
};

/*
2 : encoder L
3 : bump B
4 : encoder L
5 : output L
6 : output L
7 : encoder R
8 : encoder R
9 : output R
10: output R
11: bump R
12: bump L
*/

//// Pins
// Digital
const char encoderR1 = 7;
const char encoderR2 = 8;
const char encoderL1 = 2;
const char encoderL2 = 4;
const char bumpR = 11;
const char bumpL = 12;
const char bumpB = 3;
const char outputR1 = 9;
const char outputR2 = 10;
const char outputL1 = 5;
const char outputL2 = 6;

// Analog
const char sensorF = 0;
const char sensorL = 1;
const char sensorR = 2;

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
  
  Serial.begin(9600);
}

void loop() {
  moveBothForward(100, 200);
  delay(1000);
}

void getSensorError() {
  
  /*
  Voltage to distance graph corresponds approx. to:
  V = 18.666666666666666666666666666667 / x
  where V is volts and x is in cm
  */
  
  analogRead(sensorL);
  analogRead(sensorR);
}

float distSensorVtoCM(float voltage) {
  return 18.666666666666666666666666666667 / voltage;
}

float distSensorCMtoV(float cm) {
  return 18.666666666666666666666666666667 / cm;
}

void moveBothForward(int ticks, float voltage) {  
  mLeft(voltage);
  mRight(voltage);
}

void mLeft(float voltage) {
  analogWrite(outputL1, voltage);
  analogWrite(outputL2, 0);
}

void mRight(float voltage) {
  analogWrite(outputR1, voltage);
  analogWrite(outputR2, 0);
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
