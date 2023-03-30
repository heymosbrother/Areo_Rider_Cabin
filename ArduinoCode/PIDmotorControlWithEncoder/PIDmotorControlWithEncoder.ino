// Motor settings

/*  
 *   Motor characteristic
 *    Gear ratio: 47
 *    Encoder ticks per round: 11 ticks
 *    Ticks per motor revolution: 470 ticks
*/    

//  Pin number format: {PWM pin, IN1(+), IN2(-)}
const int motor1[] = {9, 7, 6};
const int motor2[] = {10, 4, 5};

// Encoder settings
//  Define two pins for encoder A, B channels

//  motor1 encoder
const int ENC1A = 2; // yellow line
const int ENC1B = 13; // green line

//  motor2 encoder
const int ENC2A = 3; // yellow line
const int ENC2B = 11; // green line

//  records the position of the encoder
int enc1pos = 0;
int enc2pos = 0;

// PID variables
long prevTime = 0;
float prevError1 = 0;
float prevError2 = 0;
float eIntegral1 = 0;
float eIntegral2 = 0;


void setup() {
    Serial.begin(9600);

    // Initiate the pins for the motors
    for (int i = 0; i < 3; i++) {
      pinMode(motor1[i], OUTPUT);
      pinMode(motor2[i], OUTPUT);
    }

    // Initiate the pins for the encoders
    pinMode(ENC1A, INPUT);
    pinMode(ENC1B, INPUT);
    pinMode(ENC2A, INPUT);
    pinMode(ENC2B, INPUT);

    // set up attachInterrupt function to call specific function immediately when pin value is changed
    attachInterrupt(digitalPinToInterrupt(ENC1A), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2A), readEncoder2, RISING);
}

void loop() {
    /*
    // Test the encoder direction by showing A, B channel values
    // store the value generated when the encoders spin
    int a1 = digitalRead(ENC1A);
    int b1 = digitalRead(ENC1B);
    int a2 = digitalRead(ENC2A);
    int b2 = digitalRead(ENC2B);

    // print encoder 1 status
    Serial.print("Encoder 1 Channel A: ");
    Serial.print(a1);
    Serial.print(" Channel B: ");
    Serial.print(b1);
    Serial.print("  position: ");
    Serial.print(enc1pos);

    Serial.print(" \t");

    // print encoder 2 status
    Serial.print("Encoder 2 Channel A: ");
    Serial.print(a2);
    Serial.print(" Channel B: ");
    Serial.print(b2);
    Serial.print("  position: ");
    Serial.print(enc2pos);

    Serial.println();
    */

    /*
    setMotor(1, motor1, 100);
    delay(200);
    Serial.println(enc1pos);
    setMotor(-1, motor1, 100);
    delay(200);
    Serial.println(enc1pos);
    setMotor(0, motor1, 100);
    delay(200);
    Serial.println(enc1pos);
    */

    
    PIDpositionControl(4 * 470, 0 * 470);   //Input format is encoder ticks, please times 470 if you want to input revolution
}

// the readEncoder function
void readEncoder1() {
  int bValue = digitalRead(ENC1B);
  if (bValue > 0) {
    enc1pos--;
  }
  else{
    enc1pos++;
  }
}
void readEncoder2() {
  int bValue = digitalRead(ENC2B);
  if (bValue > 0) {
    enc2pos++;
  }
  else {
    enc2pos--;
  }
}

// Set motor functions
//  Input format: {direction, motor[], PWM value}
void setMotor(int dir, const int motor[], int pwmValue) {
  analogWrite(motor[0], pwmValue);

  // dir == 1 --> forward direction
  if(dir == 1) {
    digitalWrite(motor[1], HIGH);
    digitalWrite(motor[2], LOW);
  }
  else if (dir == -1) {
    digitalWrite(motor[1], LOW);
    digitalWrite(motor[2], HIGH);
  }
  else {
    digitalWrite(motor[1], LOW);
    digitalWrite(motor[2], LOW);
  }
}

// PID position control function, input unit: encoder ticks, need further calculation to turn into revolution
void PIDpositionControl(int targetPosition1, int targetPosition2) {
  // PID parameters
  float Kp = 0.5;
  float Ki = 0.1;
  float Kd = 0.02;

  // calculate time difference
  long currTime = micros();

  float deltaTime = ((float)(currTime-prevTime))/1.0e6;   // change the unit from ms to s

  prevTime = currTime;

  // calculate the error
  int error1 = enc1pos - targetPosition1;
  int error2 = enc2pos - targetPosition2;

  // derivative
  float dt1 = (error1 - prevError1) / (deltaTime);
  float dt2 = (error2 - prevError2) / (deltaTime);

  // integral
  eIntegral1 += error1 * deltaTime;
  eIntegral2 += error2 * deltaTime;

  // set control PWM signal & motor power
  float u1 = Kp * error1 + Kd * dt1 + Ki * eIntegral1;
  float u2 = Kp * error2 + Kd * dt2 + Ki * eIntegral2;
  float power1 = fabs(u1); // take absolute values of u1
  float power2 = fabs(u2);
  if(power1 > 255) {
    power1 = 255;
  }
  if(power2 > 255) {
    power2 = 255;
  }

  // motor direction
  int dir1 = 1, dir2 = 1;
  if(u1<0) {
    dir1 = -1;
  }
  if(u2<0) {
    dir2 = -1;
  }
  
  // signal the motor
  setMotor(dir1, motor1, power1);
  setMotor(dir2, motor2, power2);

  // Store previous error
  prevError1 = error1;
  prevError2 = error2;

  Serial.println("Motor position control:");
  Serial.print("target 1 : ");
  Serial.print(targetPosition1);
  Serial.print("\t real 1 : ");
  Serial.print(enc1pos);
  Serial.print("   \t target2: ");
  Serial.print(targetPosition2);
  Serial.print("\t real 2 : ");
  Serial.println(enc2pos);
  for(int i = 0; i < 100; i++) Serial.print("-");
  Serial.println();
}

// PID 
