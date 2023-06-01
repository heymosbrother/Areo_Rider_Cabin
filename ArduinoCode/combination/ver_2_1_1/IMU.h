#ifndef IMU_H
#define IMU_H

#include <MPU6050_light.h>
#include <Wire.h>
#include "Arduino.h"


// inherit from Mpu6050 class
class IMU : public MPU6050 
{
private:


public:
    // methods
    IMU();
    void ShowAngle();
    void ShowAccel();
    void ShowGyro();
    bool ShockDetect(int sensitivity); // in percentage (0 ~ 100)
};

#endif
