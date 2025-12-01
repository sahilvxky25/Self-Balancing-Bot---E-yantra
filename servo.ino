#include <Servo.h>

// Create Servo objects
Servo servo1;
Servo servo2;

void setup() {
  // Attach the servos to the respective pins
  servo1.attach(10);
  servo2.attach(11);
}

void loop() {
  // Sweep from 0 to 180 degrees
  for (int angle = 0; angle <= 180; angle++) {
    servo1.write(angle);
    servo2.write(angle);
    delay(15); // Small delay for smooth movement
  }

  // Sweep from 180 to 0 degrees
  for (int angle = 180; angle >= 0; angle--) {
    servo1.write(angle);
    servo2.write(angle);
    delay(15); // Small delay for smooth movement
  }
}
