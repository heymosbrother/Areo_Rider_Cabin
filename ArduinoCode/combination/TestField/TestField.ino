// Self made class
#include "Motor.h"
// #include "IMU.h"

// Official libraries
#include <Wire.h>
#include <MPU6050_light.h>

// Objects

Motor motor_left(10, 9, 8, 2, 11);
Motor motor_right(5, 6, 7, 3, 4);

MPU6050 imu(Wire);

// PID parameters
float Kp = 5, Kd = 0.1;
float prev_error = 0;


enum State
{
    preperation,
    test1,
    test2,
    test3,
    testEnd
};
State currentState;

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    imu.begin();
    imu.calcOffsets();

    // attach interrupts  
    attachInterrupt(digitalPinToInterrupt(motor_left.ENC_A), readEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_right.ENC_A), readEncoderRight, RISING);

    prev_error = 0;
    
    delay(2000);
    currentState = test1;
}

void loop()
{
    
    imu.update();
    // show value
    Serial.println(currentState);

    switch (currentState)
    {
    case test1:
        if (millis() >= 20000)
        {
            currentState = test2;
        }
        GoStraight(30, 0);
        break;
    case test2:
        if (millis() >= 30000)
        {
            currentState = test3;
        }
        motor_left.SetVelocity(0);
        motor_right.SetVelocity(30);
        break;
    case test3:
        if (millis() >= 40000)
        {
            currentState = testEnd;
        }
        turnAngle(90);
        break;
    case testEnd:
        motor_left.StopAllMotors();
        motor_right.StopAllMotors();
        break;
    default:    
        break;
    }
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

// Go straight function
float GoStraight(int targetSpeed, int targetAngle) // in rpm
{
    // Use P(I?)D to control the vehicle angle
    float error = targetAngle - imu.getAngleZ();
    // control signal u
    float u = Kp * error + Kd * (error - prev_error);
    // update prev_error
    prev_error = error;
    // set motor speed
    motor_left.SetVelocity(targetSpeed - u);
    motor_right.SetVelocity(-targetSpeed - u);
    return u;
}


// Turn to a certain angle
float turnAngle(int targetAngle)
{
    // Use P(I?)D to control the vehicle angle
    float error = targetAngle - imu.getAngleZ();
    // control signal u
    float u = Kp * error + Kd * (error - prev_error);
    // update prev_error
    prev_error = error;
    // set motor speed
    motor_left.SetVelocity(-1 * u);
    motor_right.SetVelocity(-1 * u);
    return u;
}
