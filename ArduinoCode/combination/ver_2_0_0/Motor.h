#ifndef MOTOR_H
#define MOTOR_H

class Motor {
private:
    // PID variables
    float e_integral, e_prev;
    float Kp, Ki, Kd;
    float time_prev;
    // filter variables
    float v_original, v_original_prev;
    float v_filtered, v_filtered_prev;

public:
    // motor pins
    int pwm;
    int inPlus;
    int inMinus;
    // encoder pins
    int encA;
    int encB;
    // Variables
    int position, position_prev;
    
    // Constructor
    Motor(int pwm, int inPlus, int inMinus, int encA, int encB);
    // Methods
    void SetMotorPWM(int pwmVal, int dir);
    void SetVelocity(float target_velocity);
};

#endif