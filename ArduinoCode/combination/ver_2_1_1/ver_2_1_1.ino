#include <Wire.h>
# include <MPU6050_light.h> // use official library rathrer than self build IMU

// try to source class in other files
#include "Tracer.h"
#include "Motor.h"

// create needed objects
//  Tracers
Tracer tracers(A0, A1, A2, A3);
//  left and right motors
Motor motor_left(10, 9, 8, 2, 11);
Motor motor_right(5, 6, 7, 3, 4);
//  the IMU
MPU6050 imu(Wire);
//  the Tracer
const int TracerPin = 12;

// PID parameters
float Kp = 5, Kd = 0.1;
float prev_error;

// State machine
enum State
{
    preparation,
    initial_straight,
    climb_a_little,
    driveTowardsWind,
    driveStriaght_alongWind,
    stop
};
State currentState;

long long int timePin;
int duration;
void setup()
{
    // attach interrupts
    attachInterrupt(digitalPinToInterrupt(motor_left.ENC_A), readEncoderLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(motor_right.ENC_A), readEncoderRight, RISING);
        
    Serial.begin(115200);
    Wire.begin();
    imu.begin();
    imu.calcOffsets();

    motor_left.SetVelocity(0);
    motor_right.SetVelocity(0);

    // tracer
    pinMode(TracerPin, INPUT);

    prev_error = 0; // set initial error for PID go straight
    timePin = millis();
    duration = 5000; // 5 seconds

    currentState = initial_straight; // set intial state
    Serial.println(currentState);

    delay(2000);
}

void loop()
{
    imu.update();
    Serial.println(currentState);
    // print out the IMU values
    /*
    Serial.print("AngleX: ");
    Serial.print(imu.getAngleX());
    Serial.print(" AngleY: ");
    Serial.print(imu.getAngleY());
    Serial.print(" AngleZ: ");
    Serial.println(imu.getAngleZ());
    */
    
    // Restart the process if the vehicle is being lifted by hand
    /*
    if (imu.getAccZ() > 1.3) 
    {
        currentState = preparation;
    }
    */

    // State machine 
    switch (currentState)
    {
    case initial_straight:
        // go straight until get on the slope
        if (imu.getAngleX() < -5)
        {
            currentState = climb_a_little;
            timePin = millis();
        }
        GoStraight(40, 0);
        break;
    case climb_a_little:
        // climb a little bit
        if (millis() - timePin > 1500)
        {
            currentState = driveTowardsWind;
            timePin = millis();
        }
        GoStraight(60, 0);
        break;
    case driveTowardsWind:  // revise here, start from here...................................................
        // drive towards the wind
        if (millis() - timePin > 1000)
        {
            currentState = stop;
            timePin = millis();
        }
        GoStraight(60, 30);
        break;
    case stop:
        // stop the vehicle
        motor_left.StopAllMotors();
        motor_right.StopAllMotors();
        break;
    }
    
}

// Go straight at a target speed and follow along the target angle, return the control signal magnitude "u"
float GoStraight(int targetSpeed, int targetAngle) // in rpm
{
    // Use P(I?)D to control the vehicle angle
    float error = targetAngle - imu.getAngleZ();
    // control signal u
    float u = Kp * error + Kd * (error - prev_error);
    // update prev_error
    prev_error = error;
    // set motor speed
    motor_left.SetVelocity(targetSpeed - u);
    motor_right.SetVelocity(-targetSpeed - u);
    return u;
}

// Turn to a certain angle, return the control signal magnitude u
float turnAngle(int targetAngle)
{
    // Use P(I?)D to control the vehicle angle
    float error = targetAngle - imu.getAngleZ();
    // control signal u
    float u = Kp * error + Kd * (error - prev_error);
    // update prev_error
    prev_error = error;
    // set motor speed
    motor_left.SetVelocity(-1 * u);
    motor_right.SetVelocity(-1 * u);
    return u;
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
