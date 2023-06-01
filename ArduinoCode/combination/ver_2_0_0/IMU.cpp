#include "IMU.h"
#include "Arduino.h"

IMU::IMU() : MPU6050()
{
    // initialize variables
    Roll = 0;
    Pitch = 0;
    for (int i = 0; i < 6; i++)
    {
        accele_original[i] = 0;
        accele_filtered[i] = 0;
        prev_accele_orginal[i] = 0;
        prev_accele_filtered[i] = 0;
    }
}

void IMU::UpdateAngle()
{
    getMotion6(&accele_original[0], &accele_original[1], &accele_original[2], &accele_original[3], &accele_original[4], &accele_original[5]);

    // low pass filter
    for (int i = 0; i < 6; i++)
    {
        accele_filtered[i] = 0.969 * prev_accele_filtered[i] + 0.0155 * accele_original[i] + 0.0155 * prev_accele_orginal[i];

        // update previous values
        prev_accele_orginal[i] = accele_original[i];
        prev_accele_filtered[i] = accele_filtered[i];
    }

    // change ax, ay, az into g
    float ax = (accele_filtered[0]-603)/(16554);
    float ay = (accele_filtered[1]+156)/(16625);
    float az = (accele_filtered[2]-38.5)/(16580);

    // calculate roll and pitch angle using ax, ay, az
    Roll = atan2(ay, az)*180/3.1416;
    Pitch = atan2(-ax, sqrt(ay*ay+az*az))*180/3.1416;
}

bool IMU::ShockDetect(int sensitivity)
{
    // need to get shock acceleration value by experiment later


    bool isShocked = false;
    for (int i = 0; i < 6; i++)
    {
        // if (abs(accele_original - prev_accele_orginal))
    }

    return isShocked;
}