#ifndef IMU_H
#define IMU_H

#include <Mpu6050.h>
#include <Wire.h>
#include <Math.h>

// inherit from Mpu6050 class
class IMU : public Mpu6050 
{
private:
    int_16_t accele_original[6];
    int_16_t accele_filtered[6];
    int_16_t prev_accele_orginal[6];
    int_16_t prev_accele_filtered[6];

public:
    // public variables
    float roll, pitch;

    // methods
    IMU();
    float updateAngle();
    bool shockDetect(int sensitivity);
}

#endif