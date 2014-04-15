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
int targetL = 5;
int targetR = 5;
const float SENSOR_PERIOD = 10; // in ms
float prevTime = 0;
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
  setSpeedInTicks(255);
  Serial.begin(9600);
}

void loop() {
  float al = distSensorVtoCM(analogRead(sensorL) * 5.0 / 1024);
  float ar = distSensorVtoCM(analogRead(sensorR) * 5.0 / 1024);
  
  if (al > ar) {
    moveL(clampVoltage(pow(ar / al, 4)));
    moveR(1);
  } else {
    moveR(clampVoltage(pow(al / ar, 4)));
    moveL(1);
  }
  
  delay(1);
}

float clampVoltage(float voltage) {
  if (voltage < 0) {
    return 0;
  } else if (voltage > 5) {
    return 5;
  } else {
    return voltage;
  }
}

float distSensorVtoCM(float voltage) {
  return -15.1645 + 39.3177 / voltage + 2.91422 * voltage;
}

float distSensorCMtoV(float cm) {
  return -0.0556334 - 72.2653 / pow(cm, 2) + 30.2105 / cm + 0.00162233 * cm;
}

void setSpeedInTicks(int ticks) {
  // Ticks per second
  targetL = ticksToVL(ticks);
  targetR = ticksToVR(ticks);
}

float ticksToVL(int ticks) {
  // Ticks per second to voltage
  return ticks / 2.13054;
}

float ticksToVR(int ticks) {
  return ticks / 2.22164;
}

int VtoTicksL(float voltage) {
  return voltage * 2.13054;
}

int VtoTicksR(float voltage) {
  return voltage * 2.22164;
}

float ticksToCM(int ticks) {
  return (ticks / 48) * 4.2 * PI;
}

void moveL(float c) {
  analogWrite(outputL1, targetL * c);
  analogWrite(outputL2, 0);
}

void moveR(float c) {
  analogWrite(outputR1, targetR * c);
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
