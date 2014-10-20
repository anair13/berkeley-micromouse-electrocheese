/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#include <Encoder.h>
#include <EEPROM.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder left(2, 4);
Encoder right(3, 8);
//   avoid using pins with LEDs attached

long leftPos = 0;
long rightPos = 0;
long oldPos = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
  analogWrite(5, 0);
  analogWrite(6, 0);
  
  analogWrite(9, 0);
  analogWrite(10, 0);
}

void loop() {
  oldPos = leftPos;
  leftPos = left.read();
  if (oldPos != leftPos) {
    EEPROM.write(0, leftPos);
  }
  oldPos = rightPos;
  rightPos = right.read();
  if (oldPos != rightPos) {
    EEPROM.write(1, rightPos);
  }
}
