// Self made class
#include "Motor.h"
// #include "IMU.h"

// Official libraries
#include <Wire.h>
#include <MPU6050_light.h>

// Objects

Motor motor_left(10, 9, 8, 2, 11);
Motor motor_right(5, 6, 7, 3, 4);

MPU6050 imu(Wire);

void setup()
{
    Serial.begin(115200);
    Wire.begin();
    imu.begin();
    imu.calcOffsets();

    // attach interrupts
    
    attachInterrupt(digitalPinToInterrupt(motor_left.ENC_A), readEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_right.ENC_A), readEncoderRight, RISING);
    
}

void loop()
{
    
    imu.update();
    // show value
    Serial.print("X : ");
    Serial.print(imu.getAngleX());
    Serial.print("\tY : ");
    Serial.print(imu.getAngleY());
    Serial.print("\tZ : ");
    Serial.println(imu.getAngleZ());
    delay(100);
    

    // motor part
    
    motor_left.SetVelocity(30);
    motor_right.SetVelocity(-30);

    /*
    Serial.print("leftMotor");
    Serial.print(motor_left.ShowVelocity());
    Serial.print("\t rightMotor");
    Serial.println(motor_right.ShowVelocity());
    */
}

// Encoder interrupt functions 
void readEncoderLeft()
{
    motor_left.SetEncoderPosition(digitalRead(motor_left.ENC_B));
}
void readEncoderRight()
{
    motor_right.SetEncoderPosition(digitalRead(motor_right.ENC_B));
}
