#include <Wire.h>

// try to source class in other files
#include "Tracer.h"
#include "Motor.h"
#include "IMU.h"

// create needed objects
//  Tracers
Tracer tracers(A0, A1, A2, A3);
//  left and right motors
Motor motor_left(10, 9, 8, 2, 11);
Motor motor_right(5, 6, 7, 3, 4);
//  the IMU
IMU imu;

#pragma region[Global variables]

// time
unsigned long startTime;
unsigned long duration;

// PID parameters
float Kp = 5, Kd = 0.1;
float prev_error = 0;

#pragma endregion

// State machine
enum State
{
    preparation,
    initial_straight,
    turnTowardsWind,
    climing,
    returnToCenterLine,
    returnStraight,
    turning,
    stop,
    testMotor
};
State currentState = preparation;
int prepareTime = 0;

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    imu.calcOffsets();
    // attach interrupts
    attachInterrupt(digitalPinToInterrupt(motor_left.ENC_A), readEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_right.ENC_A), readEncoderRight, RISING);

    motor_left.SetVelocity(0);
    motor_right.SetVelocity(0);
}

void loop()
{
    imu.update();
    // Emergency reset if the car encounters a large `shock
    if (imu.ShockDetect(0.5)) currentState = preparation;

    // State machine part
    switch (currentState)
    {
    case preparation:
        // wait for three seconds, then change to next state
        if (millis() >= 3000) currentState = testMotor; // test the motor, remember to change to initial_straight when done
        GoStraight(motor_left, motor_right, 0, 0, imu.getAngleZ());
        break;
    case initial_straight:
        // change to climing mode
        if (imu.getAngleX() <= -10) currentState = turnTowardsWind;
        GoStraight(motor_left, motor_right, 50, 0, imu.getAngleZ());
        break;
    case turnTowardsWind:
        // turn the car to get close to the wind
        float targetAngle = 30;
        // 
        if (imu.getAngleZ() >= targetAngle) 
        {
            startTime = millis();
            currentState = returnToCenterLine;
        }
        turnAngle(motor_left, motor_right, 50, targetAngle, imu.getAngleZ());
        break;
    case climing:
        // climb for a certain time
        if (millis() - startTime >= duration) currentState = returnToCenterLine;
        GoStraight(motor_left, motor_right, 80, 0, imu.getAngleZ());
        break;
    case returnToCenterLine:
        // turn the car to get back to the center line
        targetAngle = -30;
        turnAngle(motor_left, motor_right, 30, targetAngle, imu.getAngleZ());
        if (tracers.OnBlack(3))
        {
            currentState = returnStraight;
        }
        break;
    case returnStraight:
        turnAngle(motor_left, motor_right, 30, 0, imu.getAngleZ());
        if (imu.getAngleZ() >= 0)
        {
            startTime = millis();
            currentState = turning;
        }
        break;
    case turning:
        break;
    case stop:
        motor_left.SetVelocity(0);
        motor_right.SetVelocity(0);
        break;
    case testMotor:
        motor_left.SetVelocity(50);
        motor_right.SetVelocity(50);
        break;
    default:
        break;
    }
}

// Go straight function
void GoStraight(Motor leftMotor, Motor rightMotor ,int targetSpeed, int targetAngle, float zAngle) // in rpm
{
    // Use P(I?)D to control the vehicle angle
    float error = targetAngle - zAngle;
    // control signal u
    float u = Kp * error + Kd * (error - prev_error);
    // update prev_error
    prev_error = error;
    // set motor speed
    leftMotor.SetVelocity(targetSpeed + u);
    rightMotor.SetVelocity(targetSpeed - u);
}

void turnAngle(Motor leftMotor, Motor rightMotor,int turnSpeed, int targetAngle, float zAngle)
{
    // Use P(I?)D to control the vehicle angle
    float error = targetAngle - zAngle;
    // control signal u
    float u = Kp * error + Kd * (error - prev_error);
    // update prev_error
    prev_error = error;
    // set motor speed
    leftMotor.SetVelocity(turnSpeed + u);
    rightMotor.SetVelocity(turnSpeed -u);
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
