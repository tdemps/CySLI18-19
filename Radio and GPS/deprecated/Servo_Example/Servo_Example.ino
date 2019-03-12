#include <Servo.h>
 
// Create a servo instance.
Servo servo;
 
void setup() {
  Serial.begin(9600);
  // Attach servo output to pin 5.
  servo.attach(5);
}
 
void loop() {
  Serial.println("wat?");
  // Move to position 0, or a 1.0 millisecond long pulse:
  // Remember the servo module in Arduino takes in a position in degrees
  // from 0 to 180 instead of a pulse length in milliseconds or other value.
  servo.write(0);
  // Delay for a second.
  delay(1000);
  // Move to position 180, or a 2.0 millisecond long pulse and pause again.
  servo.write(60);
  delay(1000);
}
