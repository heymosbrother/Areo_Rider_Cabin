#include "Motor.h"
#include "Arduino.h"

Motor::Motor(int pwm, int inPlus, int inMinus, int encA, int encB)
{
    // assign pins
    this->pwm = pwm;
    this->inPlus = inPlus;
    this->inMinus = inMinus;
    this->encA = encA;
    this->encB = encB;
    // initialize pins
    pinMode(pwm, OUTPUT);
    pinMode(inPlus, OUTPUT);
    pinMode(inMinus, OUTPUT);
    pinMode(encA, INPUT);
    pinMode(encB, INPUT);

    // initialize variables
    position = 0;
    v_original = 0;
    v_filtered = 0;
    time_prev = 0;
    position_prev = 0;
    v_original_prev = 0;
    v_filtered_prev = 0;
    e_integral = 0;
    e_prev = 0;
    // PID parameters
    Kp = 5, Ki = 1, Kd = 0;
}

Motor::SetMotorPWM(int pwmVal, int dir)
{
    analogWrite(pwm, pwmVal);
    if (dir == 1)
    {
        digitalWrite(inPlus, HIGH);
        digitalWrite(inMinus, LOW);
    }
    else if(dir == -1)
    {
        digitalWrite(inPlus, LOW);
        digitalWrite(inMinus, HIGH);
    }
    else
    {
        digitalWrite(inPlus, LOW);
        digitalWrite(inMinus, LOW);
    }
}

Motor::SetVelocity(float target_velocity)
{
    // get current time
    float time_now = micros();
    // calculate velocity (rpm)
    float v_original = (position - position_prev) / ((float)(time_now - time_prev)) * 60 * 1000000 / 517;
    // filter velocity
    v_filtered = 0.854 * v_filtered_prev + 0.0728 * v_original + 0.0728 * v_original_prev;
    // update variables
    v_filtered_prev = v_filtered;
    v_original_prev = v_original;
    time_prev = time_now;
    position_prev = position; 

    // compute gain u using PID
    float e = target_velocity - v_filtered;
    e_integral += e;
    float u = Kp * e + Ki * e_integral + Kd * (e - e_prev);

    // set motor speed and direction
    int dir = 0;
    if (u > 0) dir = 1;
    else if (u < 0) dir = -1;
    else dir = 0;
    // calculate pwm value
    int pwmVal = (int)fabs(u);
    if (pwmVal > 255) pwmVal = 255;
    // set motor
    SetMotorPWM(pwmVal, dir);

    return;
}