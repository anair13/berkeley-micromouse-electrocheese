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

float integralError = 0;
float prevError = 0;

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
  /*
  Notes from Ashvin:
  Integral doesn't work yet and it doesn't seem very likely to work... maybe we need some limit/damper on the integration time bound...
  Slow speeds work better, especially at the start. Try speed = 220 
  Maybe we should average sensor reads
  Rethought the control structure to fit PID, but doesn't work. I'll email alice and william today about what they do
  */
  
  //Serial.println(analogRead(sensorL));
  //Serial.println(analogRead(sensorR));
  //float al = distSensorVtoCM(analogRead(sensorL) * 5.0 / 1024);
  //float ar = distSensorVtoCM(analogRead(sensorR) * 5.0 / 1024);
  float al = analogRead(sensorL) * 5.0 / 1024; // larger voltage means smaller distance
  float ar = analogRead(sensorR) * 0.779 * 5.0 / 1024;
  //float al = 1.0 / analogRead(sensorL);
  //float ar = 1.0 / analogRead(sensorR);
  float p = 1;
  float i = 1;
  float d = 0;
  float error = al - ar; // range: ~ [-0.5 , 0.5]
  // if error is positive, al > ar, dist(al) < dist(ar), closer to right
  // turn left if error is positive
  float timeElapsed = (millis() - prevTime) / 1000;
  
  float P = p * error;
  float I = i * integralError;
  float D = d * (error - prevError) / timeElapsed;
  float PID = P + I + D;
  moveL(leftV(clamp(1 - PID, 0, 2)));
  moveR(rightV(clamp(1 + PID, 0, 2)));
  
  integralError += error * timeElapsed;
  prevError = error;
  prevTime = millis();
  /*
  float difference = p * (ar - al);
  float R = ar + difference - derivative + i * integral;
  float L = al - difference + derivative - i * integral;
  float m = max(R, L);
  derivative = d * (R/m - L/m);
  integral += difference;
  moveR(R / m);
  moveL(L / m);  
  */
//  float bigger = max(al, ar);
//  moveR(ar / bigger);
//  moveL(al / bigger);
  
//  if (al > ar) {
//    //moveR((al - ar)/big); // this works counterintuitively well
//    moveR(ar/al);
//    moveL(1);
//  } else {
//    moveL(al/ar);
//    //moveL((ar - al)/big); // this works counterintuitively well
//    moveR(1);
//  }

  delay(1);
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

float clampVoltage(float voltage) {
  if (voltage < 0) {
    return 0;
  } else if (voltage > 255) {
    return 255;
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

float leftV(float voltage) {
  return voltage * 0.95899425649410083663005614915752;
}

float rightV(float voltage) {
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
  analogWrite(outputL1, clamp(targetL * c, 0, 255));
  analogWrite(outputL2, 0);
}

void moveR(float c) {
  analogWrite(outputR1, clamp(targetR * c, 0, 255));
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
