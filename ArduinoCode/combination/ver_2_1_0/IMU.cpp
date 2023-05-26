#include "IMU.h" 

void IMU::ShowAngle()
{
    Serial.print("Angle  ");
    Serial.print("X : ");
    Serial.print(getAngleX());
    Serial.print("\tY : ");
    Serial.print(getAngleY());
    Serial.print("\tZ : ");
    Serial.println(getAngleZ());
}

void IMU::ShowAccel()
{
    Serial.print("Acceleration  ");
    Serial.print("X : ");
    Serial.print(getAccX());
    Serial.print("\tY : ");
    Serial.print(getAccY());
    Serial.print("\tZ : ");
    Serial.println(getAccZ());
}

void IMU::ShowGyro()
{
    Serial.print("Gyroscope  ");
    Serial.print("X : ");
    Serial.print(getGyroX());
    Serial.print("\tY : ");
    Serial.print(getGyroY());
    Serial.print("\tZ : ");
    Serial.println(getGyroZ());
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
