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
const char outputL1 = 6;
const char outputL2 = 5;

// Analog
const char sensorF = 0;
const char sensorL = 1;
const char sensorR = 2;

// PID
const int targetTicksL = 5;
const int targetTicksR = 5;
const float SENSOR_PERIOD = 10; // in ms
float prevTime = 0;
int stateChangesL = 0;
int stateChangesR = 0;
int prevStateL = 0;
int prevStateR = 0;
float prevErrorL = 0;
float prevErrorR = 0;
float integralErrorL = 0;
float integralErrorR = 0;

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
  moveBothForward(128);
  Serial.begin(9600);
}

void loop() {
  // Update states
  int stateL = getStateL();
  int stateR = getStateR();
  
  if (prevStateL != stateL) {
    stateChangesL++;
  }
  if (prevStateR != stateR) {
    stateChangesR++;
  }

  // Update ticks and check for trigger
  /*
  if (millis() - prevTime >= SENSOR_PERIOD) {
    //adjustWheels();
    prevTime = millis();
    integralErrorL += getErrorL();
    integralErrorR += getErrorR();
  }
  */
}

void adjustWheels() {
  float errorL = getErrorL(); // error in ticks
  float errorR = getErrorR(); // error in ticks

  float Kp = -1000000; // TODO adjust
  float Ki = 0; // TODO adjust
  float Kd = 0; // TODO adjust
  float Pl = Kp * errorL;
  float Il = Ki * integralErrorL;
  float Dl = Kd * (errorL - prevErrorL) / SENSOR_PERIOD;
  float Pr = Kp * errorR;
  float Ir = Ki * integralErrorR;
  float Dr = Kd * (errorR - prevErrorR) / SENSOR_PERIOD;

  prevErrorL = errorL;
  prevErrorR = errorR;

  float responseL = Pl + Il + Dl;
  float responseR = Pr + Ir + Dr;

  if (responseL < 0) {
    responseL = 0;
  } else if (responseL > 255) {
    responseL = 255;
  }
  if (responseR < 0) {
    responseR = 0;
  } else if (responseR > 255) {
    responseR = 255;
  }

  moveL(responseL);
  moveR(responseR);
}

int getErrorL() {
  return stateChangesL - targetTicksL;
}

int getErrorR() {
  return stateChangesR - targetTicksR;
}

float distSensorVtoCM(float voltage) {
  return -15.1645 + 39.3177 / voltage + 2.91422 * voltage;
}

float distSensorCMtoV(float cm) {
  return -0.0556334 - 72.2653 / pow(cm, 2) + 30.2105 / cm + 0.00162233 * cm;
}

void moveBothForward(float voltage) {
  moveL(ticksToVL(100));
  moveR(ticksToVR(100));
}

float ticksToVL(int ticks) {
  return ticks / 1.06527;
}

float ticksToVR(int ticks) {
  return ticks / 1.11082;
}

int VtoTicksL(float voltage) {
  return voltage * 1.06527;
}

int VtoTicksR(float voltage) {
  return voltage * 1.11082;
}

float ticksToCM(int ticks) {
  return (ticks / 48) * 4.2 * PI;
}

void moveL(float voltage) {
  analogWrite(outputL1, voltage);
  analogWrite(outputL2, 0);
}

void moveR(float voltage) {
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
