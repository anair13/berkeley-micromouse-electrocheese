int lEncoder = 0;
int rEncoder = 0;
int lEncoderPin = 0;
int rEncoderPin = 1;
int lEncoderPrevVal = 0;
int rDirection = 1; // -1, 0, or 1
int lDirection = 1; // -1, 0, or 1
int rMotorPin = 5;
int lMotorPin = 6;

int threshold = 512; // for encoders

int i = 0; // bounces back and forth for testing

void setup() {
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
}

int temp = 0;

void loop() {
  // encoder updating
  temp = analogRead(lEncoderPin);
  if (temp > threshold && lEncoderPrevVal < threshold) {
    lEncoder += lDirection;
  }
  lEncoderPrevVal = temp;
  
  // motor moving
  i++;
  if (i > 1000) {
    analogWrite(lMotorPin, 2000 - i);
  }
  else {
    analogWrite(lMotorPin, i);
  }
  if (i > 2000) {
    i = 0;
  }
}
