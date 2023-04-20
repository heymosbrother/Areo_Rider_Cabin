// set motor pins
#define MOTOR0_IN1 11 // set up the IN1 pin
#define MOTOR0_IN2 8 // set up the IN2 pin

float timer = 0;

void setup() {
  Serial.begin(9600);
  pinMode(MOTOR0_IN1, OUTPUT); // set IN1 as output
  pinMode(MOTOR0_IN2, OUTPUT); // set IN2 as output

  // wait time
    delay(5000);
    timer=0;
}

void loop() {
  if(millis()<=5500){
    digitalWrite(MOTOR0_IN1, HIGH); // set IN1 to HIGH
    digitalWrite(MOTOR0_IN2, LOW); // set IN2 to LOW
    timer += millis();
  }
  else{
    digitalWrite(MOTOR0_IN1, LOW);
    digitalWrite(MOTOR0_IN2, LOW); 
  }
  Serial.println(millis());
}
