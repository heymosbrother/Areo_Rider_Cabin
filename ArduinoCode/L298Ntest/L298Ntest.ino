#define MOTOR_IN1 8 // set up the IN1 pin
#define MOTOR_IN2 9 // set up the IN2 pin

void setup() {
  pinMode(MOTOR_IN1, OUTPUT); // set IN1 as output
  pinMode(MOTOR_IN2, OUTPUT); // set IN2 as output
}

void loop() {
  digitalWrite(MOTOR_IN1, HIGH); // set IN1 to HIGH
  digitalWrite(MOTOR_IN2, LOW); // set IN2 to LOW
}
