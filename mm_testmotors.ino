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
