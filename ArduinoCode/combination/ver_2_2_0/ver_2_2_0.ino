#include <Wire.h>
#include <MPU6050_light.h> // use official library rathrer than self build IMU
#include <Servo.h>

// try to source class in other files
#include "Tracer.h"
#include "Motor.h"

// create needed objects

//  Servos, range: (97, 0), notice that 97 is the original position
Servo servoHead;
Servo servoTail;
//  left and right motors
Motor motor_left(10, 12, 8, 2, 13); // (PWM, inPlus, inMinus, ENC_A, ENC_B)
Motor motor_right(5, 6, 7, 3, 4);  // (PWM, inPlus, inMinus, ENC_A, ENC_B)
//  IMU MPU6050
MPU6050 imu(Wire);
//  the Tracers
Tracer tracer1(A0);
int blackedAmount = 0;
bool lastBlackValue = false;
// servo motor expansion time
int timeCounter = 90;
int targetAngle = 0;
int currentAngle = 90;
int increment = 1;
int delayTime = 150;

// PID parameters
float Kp_straight = 2, Kd_straight = 0.05;
float prevError_straight;

// State machine
enum State
{
    preparation,
    initialStraight,
    climbOnSlope,
    adjustTurning,
    adjustGoStraight,
    stop
};
State currentState;

long long int timePin;
int duration;
void setup()
{
    // attach interrupts
    attachInterrupt(digitalPinToInterrupt(motor_left.ENC_A), readEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_right.ENC_A), readEncoderRight, RISING);
        
    Serial.begin(115200);
    Wire.begin();
    
    // object initialization
    imu.begin();
    imu.calcOffsets();
    motor_left.SetVelocity(0);
    motor_right.SetVelocity(0);
    //  Servos
    servoHead.attach(11);
    servoTail.attach(9);
    servoHead.write(90);
    servoTail.write(90);

    prevError_straight = 0; // set initial error for PID go straight
    timePin = millis();
    duration = 5000; // 5 seconds
    blackedAmount = 0;

    currentState = preparation; // set intial state
    Serial.println(currentState);

    delay(2000);
}

void loop()
{
    imu.update();
    Serial.print("On black "); Serial.print(tracer1.onBlack());
    Serial.print("\t state: ");
    Serial.println(currentState);
    // print out the IMU values
    
    Serial.print("AngleX: ");
    Serial.print(imu.getAngleX());
    Serial.print(" AngleY: ");
    Serial.print(imu.getAngleY());
    Serial.print(" AngleZ: ");
    Serial.println(imu.getAngleZ());
    Serial.println();
    
    
    // Restart the process if the vehicle is being lifted by hand
    /*
    if (imu.getAccZ() > 1.3) 
    {
        currentState = preparation;
    }
    */

    // State machine 
    switch (currentState)
    {
    case preparation:
        if (currentAngle < targetAngle)
        {
            currentState = initialStraight;
        }
        else
        {
            currentAngle -= increment;
            servoHead.write(currentAngle);
            servoTail.write(currentAngle);
            delay(delayTime);
        }
        break;
    case initialStraight:
        // go straight until get on the slope
        if (imu.getAngleX() < -5)
        {
            currentState = climbOnSlope;
            servoHead.write(0);
            servoTail.write(0);
            timePin = millis();
        }
        GoStraight(100, 0);
        break;
    case climbOnSlope:
        if (tracer1.onBlack())
        {
            currentState = adjustTurning;
        }
        GoStraight(70, 0);
        break;
    case adjustTurning:
        if (imu.getAngleZ() >= 23)
        {
            currentState = adjustGoStraight;
        }
        motor_left.SetVelocity(-15);
        motor_right.SetVelocity(-30);
        break;
    case adjustGoStraight:
        if (tracer1.onBlack())
        {
            currentState = stop;
        }
        GoStraight(50, 23);
        break;
    case stop:
        // stop the vehicle
        motor_left.StopAllMotors();
        motor_right.StopAllMotors();
        break;
    }
    
}

// Go straight at a target speed and follow along the target angle, return the control signal magnitude "u"
float GoStraight(int targetSpeed, int targetAngle) // in rpm
{
    // Use P(I?)D to control the vehicle angle
    float error = targetAngle - imu.getAngleZ();
    // control signal u
    float u = Kp_straight * error + Kd_straight * (error - prevError_straight);
    // update prevError_straight
    prevError_straight = error;
    // set motor speed
    motor_left.SetVelocity(targetSpeed - u);
    motor_right.SetVelocity(-targetSpeed - u);
    return u;
}

// Turn to a certain angle, return the control signal magnitude u
float turnAngle(int targetAngle)
{
    // Use P(I?)D to control the vehicle angle
    float error = targetAngle - imu.getAngleZ();
    // control signal u
    float u = Kp_straight * error + Kd_straight * (error - prevError_straight);
    // update prevError_straight
    prevError_straight = error;
    // set motor speed
    motor_left.SetVelocity(-1 * u);
    motor_right.SetVelocity(-1 * u);
    return u;
}
 
// Encoder interrupt functions 
void readEncoderLeft()
{
    motor_left.SetEncoderPosition(digitalRead(motor_left.ENC_B));
}
void readEncoderRight()
{
    motor_right.SetEncoderPosition(digitalRead(motor_right.ENC_B));
}
