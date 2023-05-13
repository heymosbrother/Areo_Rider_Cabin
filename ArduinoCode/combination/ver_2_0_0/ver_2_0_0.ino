#include <Wire.h>

// try to source class in other files
#include "Tracer.h"
#include "Motor.h"
#include "IMU.h"

// create needed objects
//  Tracers
Tracer tracers(A0, A1, A2, A3);
//  left and right motors
Motor motor_left(9, 7, 6, 2, 13);
Motor motor_right(10, 8, 5, 3, 12);
//  the IMU
IMU imu;

// State machine
enum State
{
    preparation,
    initial_straight,
    climing
};
State currentState = preparation;
int prepareTime = 0;

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    imu.initialize();
    // attach interrupts
    attachInterrupt(digitalPinToInterrupt(motor_left.encA), readEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_right.encA), readEncoderRight, RISING);

    motor_left.SetVelocity(0);
    motor_right.SetVelocity(0);
}

void loop()
{
    imu.updateAngle();
    // Emergency reset if the car encounters a shock
    if () currentState = preparation;

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
        if (imu.pitch >= 10) currentState = climing;

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
    motor_left.position += digitalRead(motor_left.encB) ? 1 : -1;
}
void readEncoderRight()
{
    motor_right.position += digitalRead(motor_right.encB) ? 1 : -1;
}
