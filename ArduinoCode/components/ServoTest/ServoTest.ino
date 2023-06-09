#include <Servo.h>

Servo servoMotor1; // Create a servo object
Servo servoMotor2;
int currentAngle = 90;
int targetAngle = 0;
int increment = 1;
int delayTime = 150;
void setup() {
  servoMotor1.attach(9); // Attach the servo to pin 9
  servoMotor2.attach(11);
}

void loop() {
  // gradually move the servo from 90 to 0 degrees and hold on at 0 degrees
  if (currentAngle > targetAngle) {
    currentAngle -= increment;
    servoMotor1.write(currentAngle);
    servoMotor2.write(currentAngle);
    delay(delayTime);
  }
  else {
    delay(100);
    targetAngle = 0;
    servoMotor1.write(targetAngle);
    servoMotor2.write(targetAngle);
    
  }
}
