#include <Servo.h>

// Create four servo objects
Servo servo8;
Servo servo9;
Servo servo10;
Servo servo11;

void setup() {
  // Attach the servos to their respective pins
  servo8.attach(8);
  servo9.attach(9);
  servo10.attach(10);
  servo11.attach(11);
}

void loop() {
  // Spin all servos to 180 degrees
  servo8.write(180);
  servo9.write(180);
  servo10.write(180);
  servo11.write(180);
  delay(1000); // Wait 1 second

  // Spin all servos back to 0 degrees
  servo8.write(0);
  servo9.write(0);
  servo10.write(0);
  servo11.write(0);
  delay(1000); // Wait 1 second
}