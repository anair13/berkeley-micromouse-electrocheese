class State {
  public:
    int stateL;
    int stateR;
    float ticksL;
    float ticksR;
    float prevTime;
    float prevError;
    float integralError;
    
    State() {};
    
    void reset() {
      stateL = 0;
      stateR = 0;
      ticksL = 0;
      ticksR = 0;
      prevTime = millis();
      prevError = 0;
      integralError = 0;
    }
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
13: LED
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
const char led = 13;

// Analog
const char sensorF = 0;
const char sensorL = 1;
const char sensorR = 2;

// FUDGE
const float SENSOR_R_FUDGE = 0.779;
const float TICK_COUNTER_L_FUDGE = 0.95833333333333333333333333333333;

// Constants of motion
int targetL = 0;
int targetR = 0;

// Am I stuck?  Dear god I hope I'm not stuck...
float timeSinceLastUnstuck = millis();
int stuckiness = 0;

int ticksL;
int ticksR;
int stateL;
int stateR;
int prevTimeEncoders;
const float SENSOR_PERIOD = 10; // in ms

void setup_control() {
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
  pinMode(led, OUTPUT);
  setSpeedInTicks(200);
}

void lightOn() {
  digitalWrite(led, HIGH);
}

void lightOff() {
  digitalWrite(led, LOW);
}

bool moveF(float blocks) {
  int _counterL = 0;
  int _counterR = 0;
  int _stateL = 0;
  int _stateR = 0;
  int TOTAL_TICKS = blocks * 125;
  //setupTickCounters(&_counterL, &_counterR, &_stateL, &_stateR);
  //while ((readTickCounterL(&_counterL) + readTickCounterR(&_counterR)) / 2 < TOTAL_TICKS) {
  const int MODE_SENSOR = 0;
  const int MODE_ENCODER = 1;
  int mode = MODE_SENSOR;
  State state;
  state.reset();
  amNotStuck();
  while ((_counterL + _counterR) / 2 < TOTAL_TICKS) {
    float l = readSensorL();
    float r = readSensorR();
    if (isWallL() && isWallR() && abs(l - r) <= 1) {
      if (mode != MODE_SENSOR) {
        mode = MODE_SENSOR;
        state.reset();
      }
    } else {
      if (mode != MODE_ENCODER) {
        mode = MODE_ENCODER;
        state.reset();
      }
    }
    switch (mode) {
      case MODE_SENSOR:
        moveBySensors(&state);
      break;
      case MODE_ENCODER:
        moveByEncoders(&state);
      break;
    }
    int newStateL = getStateL();
    int newStateR = getStateR();
    if (_stateL != newStateL) {
      _counterL += stateDelta(_stateL, newStateL);
      _stateL = newStateL;
    }
    if (_stateR != newStateR) {
      _counterR += stateDelta(_stateR, newStateR);
      _stateR = newStateR;
    }
    //updateTickCounters(&_counterL, &_counterR, &_stateL, &_stateR);
    
    // Am I stuck?
    if (isStuck()) {
      lightOn();
      moveL(-1);
      moveR(-1);
      delay(200 + stuckiness * 50);
      lightOff();
      moveL(0);
      moveR(0);
      amNotStuck();
      return false;
    }
    
    delay(1);
  }
  amNotStuck();
  moveL(0);
  moveR(0);
  brake(1, 1);
  return true;
}

//void crashRecovery()

// negative is left, positive is right
void turn(int deg) {
  float ticksL = 0;
  float ticksR = 0;
  float totalTicksL = 0;
  float totalTicksR = 0;
  float stateL = 0;
  float stateR = 0;
  float TOTAL_TICKS = (13.0 * abs(deg)) / 90;
  float prevTimeEncoders = millis();
  amNotStuck();
  while(totalTicksL < TOTAL_TICKS && totalTicksR < TOTAL_TICKS) {
    int newStateL = getStateL();
    int newStateR = getStateR();
    if (newStateL != stateL) {
      //float inc = stateDelta(stateL, newStateL);
      float inc = 1;
      ticksL += inc;
      totalTicksL += inc;
      stateL = newStateL;
    }
    if (newStateR != stateR) {
      //float inc = stateDelta(stateR, newStateR);
      float inc = 1;
      ticksR += inc;
      totalTicksR += inc;
      stateR = newStateR;
    }
    if (millis() - prevTimeEncoders >= SENSOR_PERIOD) {
      ticksL *= TICK_COUNTER_L_FUDGE;
      float error = (ticksL - ticksR) * 0.5 / SENSOR_PERIOD;
      if (deg < 0) {
        moveL(leftVMotor(clamp(-(1 - error), -2, 0)));
        moveR(rightVMotor(clamp(1 + error, 0, 2)));
      } else {
        moveL(leftVMotor(clamp(1 - error, 0, 2)));
        moveR(rightVMotor(clamp(-(1 + error), -2, 0)));
      }
    
      ticksL = 0;
      ticksR = 0;
    
      prevTimeEncoders = millis();
    }
    if (isStuck()) {
      lightOn();
      moveL(-1);
      moveR(-1);
      delay(200 + stuckiness * 50);
      lightOff();
      moveL(0);
      moveR(0);
      amNotStuck();
    }
  }
  amNotStuck();
  if (deg > 0) {
    brake(-1, 1);
  }
  if (deg < 0) {
    brake(1, -1);
  }
}

bool isWallL() {
  return (readSensorL() > 1);
}

bool isWallR() {
  return (readSensorR() > 1);
}

void moveByEncoders(class State* state) {
  int newStateL = getStateL();
  int newStateR = getStateR();
  if (newStateL != state->stateL) {
    state->ticksL += stateDelta(state->stateL, newStateL);
    state->stateL = newStateL;
  }
  if (newStateR != state->stateR) {
    state->ticksR += stateDelta(state->stateR, newStateR);
    state->stateR = newStateR;
  }
  if (millis() - state->prevTime >= SENSOR_PERIOD) {
    state->ticksL *= 0.958333;
    float error = (state->ticksL - state->ticksR) * 0.5 / SENSOR_PERIOD;
    // Slow down if moving too fast
    if (state->ticksL < 0.5 * SENSOR_PERIOD) {
      moveL(leftVMotor(clamp(1 - error, 0, 2)));
      moveR(rightVMotor(clamp(1 + error, 0, 2)));
    } else {
      moveL(0);
      moveR(0);
    }
    
    state->ticksL = 0;
    state->ticksR = 0;
    
    state->prevTime = millis();
  }
}

//////////////////////////
void setupTickCounters(int* counterL, int* counterR, int* stateL, int* stateR) {
  *counterL = 0;
  *counterR = 0;
  *stateL = 0;
  *stateR = 0;
}

void updateTickCounters(int* counterL, int* counterR, int* stateL, int* stateR) {
  int newStateL = getStateL();
  int newStateR = getStateR();
  if (newStateL != *stateL) {
    *counterL += stateDelta(*stateL, newStateL);
    //*counterL ++;
    *stateL = newStateL;
  }
  if (newStateR != *stateR) {
    *counterR += stateDelta(*stateR, newStateR);
    //*counterR ++;
    *stateR = newStateR;
  }
}

void resetTickCounters(int* counterL, int* counterR) {
  *counterL = 0;
  *counterR = 0;
}

int readTickCounterL(int* counterL) {
  return *counterL * TICK_COUNTER_L_FUDGE;
}

int readTickCounterR(int* counterR) {
  return *counterR;
}
/////////////

float readSensorL() {
  return analogRead(sensorL) * 5.0 / 1024;
}

float readSensorR() {
  return analogRead(sensorR) * SENSOR_R_FUDGE * 5.0 / 1024;
}

float readSensorF() {
  float s = 0;
  for (int i = 0; i < 5; i++) {
    s += analogRead(sensorF);
  }
  return s / 1024.0;
  // return analogRead(sensorF) * 5.0 / 1024;
}

void turnBySensor(float dir) {
  turnBySensor2(dir, 0.02);
}

void turnBySensor2(float dir, float threshold) { // positive for clockwise
  if (threshold < 0.004) {
    return;
  }
  if (abs(dir) < 0.8) {
    dir = (dir < 0 ? -1 : 1) * 0.8;
  }
  float f_prev = readSensorF();
  moveR(dir);
  moveL(-dir);
  while (true) {
    delay(2);
    float f = readSensorF();
    if (f < f_prev - threshold) {
      break;
    }
    else {
      f_prev = f;
    }
  }
  //brake(-dir, dir);
  moveR(0);
  moveL(0);
  turnBySensor2(-dir, threshold / 2);
}

void moveBySensors(class State* state) {
  float al = readSensorL(); // larger voltage means smaller distance
  float ar = readSensorR();
  float p = 1;
  float i = 1;
  float d = 0.2;
  float error = al - ar; // range: ~ [-0.5 , 0.5]
  // if error is positive, al > ar, dist(al) < dist(ar), closer to right
  // turn left if error is positive
  float timeElapsed = (millis() - state->prevTime) / 1000;
  
  float P = p * error;
  float I = i * state->integralError;
  float D = d * (error - state->prevError) / timeElapsed; // range 10
  float PID = P + I + D;
  moveL(leftVSensor(clamp(1 - PID, 0, 2)));
  moveR(rightVSensor(clamp(1 + PID, 0, 2)));
  
  state->integralError += error * timeElapsed;
  state->prevError = error;
  state->prevTime = millis();
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

float frontVSensor(float voltage) {
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

void brake(int ldir, int rdir) {
  moveL(leftVMotor(-ldir));
  moveR(rightVMotor(-rdir));
  delay(150);
  moveR(0);
  moveL(0);
}

void amNotStuck() {
  timeSinceLastUnstuck = millis();
}

bool isStuck() {
  if (millis() > 2000 + timeSinceLastUnstuck) {
    return true;
  } else {
    return false;
  }
}
