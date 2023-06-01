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

// State machine
enum State
{
    preparation,
    initial_straight,
    climing,
    turning,
    stop
};
State currentState = preparation;
int prepareTime = 0;

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    imu.initialize();
    // attach interrupts
    attachInterrupt(digitalPinToInterrupt(motor_left.ENC_A), readEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_right.ENC_A), readEncoderRight, RISING);

    motor_left.SetVelocity(0);
    motor_right.SetVelocity(0);
}

void loop()
{
    imu.UpdateAngle();
    // Emergency reset if the car encounters a large `shock
    if (imu.ShockDetect(0.5)) currentState = preparation;

    // State machine part
    switch (currentState)
    {
    case preparation:
        // wait for three seconds, then change to next state
        if (millis() >= 3000) currentState = initial_straight;
        motor_left.SetVelocity(0);
        motor_right.SetVelocity(0);
        break;
    case initial_straight:
        // change to climing mode
        if (imu.Pitch >= 10) currentState = climing;

        break;
    case climing:

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
