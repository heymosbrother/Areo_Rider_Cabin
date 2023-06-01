#ifndef IMU_H
#define IMU_H

#include <MPU6050.h>
#include <Wire.h>
#include "Arduino.h"


// inherit from Mpu6050 class
class IMU : public MPU6050 
{
private:
    int16_t accele_original[6];
    int16_t accele_filtered[6];
    int16_t prev_accele_orginal[6];
    int16_t prev_accele_filtered[6];

public:
    // public variables
    float Roll, Pitch;

    // methods
    IMU();
    void UpdateAngle();
    bool ShockDetect(int sensitivity);
};

#endif
