#include IMU.h
#include "Arduino.h"
#include <Math.h>

IMU::IMU() : MPU6050()
{
    // initialize variables
    row = 0;
    pitch = 0;
    for (int i = 0; i < 6; i++)
    {
        prev_accele_orginal[i] = 0;
        prev_accele_filtered[i] = 0;
    }
}

void IMU::updateAngle()
{
    int16_t accele_original[] = {0,0,0,0,0,0}; // ax, ay, az, gx, gy, gz
    getMotion6(&accele_original[0], &accele_original[1], &accele_original[2], &accele_original[3], &accele_original[4], &accele_original[5]);

    // low pass filter
    int16_t accele_filtered[] = {0,0,0,0,0,0};

    for (int i = 0; i < 6; i++)
    {
        accele_filtered[i] = 0.969 * prev_accele_filtered[i] + 0.0155 * accele_original[i] + 0.0155 * prev_accele_orginal[i];

        // update previous values
        prev_accele_orginal[i] = accele_original[i];
        prev_accele_filtered[i] = accele_filtered[i];
    }

    // change ax, ay, az into g
    float ax = -2.51 + sqrt((13710.8-accele_filtered[0])/38.43);
    float ay = 19.99 
}