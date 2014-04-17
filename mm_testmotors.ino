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

// FUDGE
const float SENSOR_R_FUDGE = 0.779;

// PID
int targetL = 5;
int targetR = 5;
float prevTime = 0;
float integralError = 0;
float prevError = 0;

const float SENSOR_PERIOD = 1000; // in ms
float prevTimeEncoders = 0;
float ticksL = 0;
float ticksR = 0;
int stateL = 0;
int stateR = 0;

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
  delay(2000);
  turnL90();
  delay(10);
  moveL(0);
  moveR(0);
}

void loop() {
  //moveByEncoders();

}

void turnL90() {
  float totalTicksL = 0;
  float totalTicksR = 0;
  const float TOTAL_TICKS = 100;
  while(totalTicksL < TOTAL_TICKS && totalTicksR < TOTAL_TICKS) {
    int newStateL = getStateL();
    int newStateR = getStateR();
    if (newStateL != stateL) {
      float inc = stateDelta(stateL, newStateL);
      ticksL += inc;
      totalTicksL += inc;
      stateL = newStateL;
    }
    if (newStateR != stateR) {
      float inc = stateDelta(stateR, newStateR);
      ticksR += inc;
      totalTicksR += inc;
      stateR = newStateR;
    }
    if (millis() - prevTimeEncoders >= SENSOR_PERIOD) {
      ticksL *= 0.95833333333333333333333333333333;
      float error = (ticksL - ticksR) * 0.5 / SENSOR_PERIOD;
      moveL(leftVMotor(clamp(-(1 - error), -2, 0)));
      moveR(rightVMotor(clamp(1 + error, 0, 2)));
    
      ticksL = 0;
      ticksR = 0;
    
      prevTimeEncoders = millis();
    }
  }
}

bool isWallL() {
  return (readSensorL() > 1);
}

bool isWallR() {
  return (readSensorR() > 1);
}

void moveByEncoders() {
  int newStateL = getStateL();
  int newStateR = getStateR();
  if (newStateL != stateL) {
    ticksL += stateDelta(stateL, newStateL);
    stateL = newStateL;
  }
  if (newStateR != stateR) {
    ticksR += stateDelta(stateR, newStateR);
    stateR = newStateR;
  }
  if (millis() - prevTimeEncoders >= SENSOR_PERIOD) {
    ticksL *= 0.95833333333333333333333333333333;
    float error = (ticksL - ticksR) * 0.5 / SENSOR_PERIOD;
    moveL(leftVMotor(clamp(1 - error, 0, 2)));
    moveR(rightVMotor(clamp(1 + error, 0, 2)));
    
    ticksL = 0;
    ticksR = 0;
    
    prevTimeEncoders = millis();
  }
}

float readSensorL() {
  return analogRead(sensorL) * 5.0 / 1024;
}

float readSensorR() {
  return analogRead(sensorR) * SENSOR_R_FUDGE * 5.0 / 1024;
}

void moveBySensors() {
  float al = readSensorL(); // larger voltage means smaller distance
  float ar = readSensorR();
  float p = 1;
  float i = 1;
  float d = 0.2;
  float error = al - ar; // range: ~ [-0.5 , 0.5]
  // if error is positive, al > ar, dist(al) < dist(ar), closer to right
  // turn left if error is positive
  float timeElapsed = (millis() - prevTime) / 1000;
  
  float P = p * error;
  float I = i * integralError;
  float D = d * (error - prevError) / timeElapsed; // range 10
  float PID = P + I + D;
  moveL(leftVSensor(clamp(1 - PID, 0, 2)));
  moveR(rightVSensor(clamp(1 + PID, 0, 2)));
  
  integralError += error * timeElapsed;
  prevError = error;
  prevTime = millis();
}

float clamp(float v, float _min, float _max) {
  if (v < _min) {
    return _min;
  } else if (v > _max) {
    return _max;
  } else {
    return v;
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

float leftVSensor(float voltage) {
  return voltage * 0.95899425649410083663005614915752;
}

float rightVSensor(float voltage) {
  return voltage;
}

float ticksToVL(int ticks) {
  // Ticks per second to voltage
  //return ticks * 1 / 2.13054;
  return ticks * 0.469364574239;
}

float ticksToVR(int ticks) {
  //return ticks * 1 / 2.22164;
  return ticks * 0.450117930897;
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
  if (c >= 0) {
    analogWrite(outputL1, clamp(targetL * c, 0, 255));
    analogWrite(outputL2, 0);
  } else {
    analogWrite(outputL1, 0);
    analogWrite(outputL2, clamp(-targetL * c, 0, 255));
  }
}

void moveR(float c) {
  if (c >= 0) {
    analogWrite(outputR1, clamp(targetR * c, 0, 255));
    analogWrite(outputR2, 0);
  } else {
    analogWrite(outputR1, 0);
    analogWrite(outputR2, clamp(-targetR * c, 0, 255));
  }
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

float leftVMotor(float voltage) {
  return voltage;
}

float rightVMotor(float voltage) {
  return voltage;
}

int stateDelta(int from, int to) {
  if (from > to) {
    to += 4;
  }
  return to - from;
}
