// try to source class in other files
#include "Tracer.h"
#include "Motor.h"

// create needed objects
//  Tracers
Tracer tracer1(A0);
//  left and right motors
Motor motor_left(9, 7, 6, 2, 13);
Motor motor_right(10, 8, 5, 3, 12);

void setup()
{
    Serial.begin(9600);

    // attach interrupts
    attachInterrupt(digitalPinToInterrupt(motor_left.encA), readEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_right.encA), readEncoderRight, RISING);
}

void loop()
{
    Serial.println(tracer1.getAnalog());
    motor_left.SetVelocity(60);
    motor_right.SetVelocity(60);
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