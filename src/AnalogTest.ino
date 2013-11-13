int i = 0;

void setup()
{
  //Serial.begin(9600);
}

void loop()
{
  int v = floor(abs(sin(2 * PI * (float)i / 50) * 127 + 128));
  //int v = sin(i) * 128 + 128;
  analogWrite(11, v);
  i += 1;
  //Serial.println(v);
  delay(10);
}
