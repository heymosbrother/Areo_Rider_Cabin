#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;


float angle[3]; // row, pitch, yaw

void setup()
{
    Wire.begin();
    Serial.begin(9600);
    mpu.initialize();

    // Enable the DMP
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);
}

void loop()
{
    
}

void getAngle()
{
    // get quaternion
    Quaternion q;
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // calculate Euler angles
    //  roll (x-axis rotation)
    angle[0] = atan2(2*q.y*q.z + 2*q.w*q.x, q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    //  pitch (y-axis rotation)
    angle[1] = -asin(2*q.x*q.z - 2*q.w*q.y);
    //  yaw (z-axis rotation)
    angle[2] = atan2(2*q.x*q.y + 2*q.w*q.z, q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);

    // convert to degree
    angle[0] *= RAD_TO_DEG;
    angle[1] *= RAD_TO_DEG;
    angle[2] *= RAD_TO_DEG;

    // print angle
    Serial.print("roll: "); Serial.print(angle[0]); Serial.print("\t");
    Serial.print("pitch: "); Serial.print(angle[1]); Serial.print("\t");
    Serial.print("yaw: "); Serial.print(angle[2]); Serial.print("\t");

}
