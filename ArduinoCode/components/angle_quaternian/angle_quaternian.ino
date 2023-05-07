#include <MPU6050.h>
#include <Wire.h>

class Quaternion {
public:
  float w, x, y, z;

  Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
  Quaternion() : w(1.0), x(0.0), y(0.0), z(0.0) {}

  Quaternion operator*(const Quaternion& q) const {
    return Quaternion(
      w*q.w - x*q.x - y*q.y - z*q.z,
      w*q.x + x*q.w + y*q.z - z*q.y,
      w*q.y - x*q.z + y*q.w + z*q.x,
      w*q.z + x*q.y - y*q.x + z*q.w);
  }
};

MPU6050 mpu;

float angle[3]; // row, pitch, yaw

void setup()
{
    Wire.begin();
    Serial.begin(9600);
    mpu.initialize();

    // Enable the DMP
    mpu.setDMPEnabled(true);
}

void loop()
{
    getAngle();
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
