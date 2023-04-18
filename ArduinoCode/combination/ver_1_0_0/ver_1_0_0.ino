/*  This code is used to let the AMR Drive straight,even with interuption occurs

    How to implement?

    Use PID control the target rotational acceleration of the z axis of the IMU to 0
 */

#pragma region [include libraries]

#include <Wire.h>  // I2C library
#include <MPU6050.h>    // MPU6050 library
#include <util/atomic.h>    // for atomic operation (PID)

#pragma endregion

#pragma region [IMU variables]

// use an enum to represent the axes
enum Axis
{
    X_linear,
    Y_linear,
    Z_linear,
    X_angular,
    Y_angular,
    Z_angular
}typedef Axis;

// MPU6050
MPU6050 accelgyro;

// variables for low pass filter
int16_t prev_accele_orginal[] = {0,0,0,0,0,0}; // ax, ay, az, gx, gy, gz
int16_t prev_accele_filtered[] = {0,0,0,0,0,0};
int16_t curr_accele_filtered[]={0,0,0,0,0,0}; 

#pragma endregion

#pragma region [PID variables]

// Pins for motor0 and motor1
//  Motor 0
const int motor0Pins[] = {2, 13, 9, 7, 6};// ENCA, ENCB, PWM, IN1, IN2
//  Motor 1
const int motor1Pins[] = {3, 12, 10, 5, 4};// ENCA, ENCB, PWM, IN1, IN2

// Time and position
long prevT[] = {0, 0};
int posPrev[] = {0, 0};

// Use the "volatile" directive for variables
//  used in an interrupt
volatile int pos_i[] = {0, 0};
volatile float velocity_i[] = {0, 0};
volatile long prevT_i[] = {0, 0};

// low-pass filter the motor velocity
float v1Filt[] = {0, 0};
float v1Prev[] = {0, 0};
float v2Filt[] = {0, 0};
float v2Prev[] = {0, 0};

// PID control
float eintegral[] = {0, 0};

#pragma endregion

#pragma region [Drive straight variables]

// Integral
int16_t angular_e_integral = 0;
// derivative 
int16_t angular_e_prev = 0;

#pragma endregion

void setup()
{
    Serial.begin(115200);

    // IMU part
    Wire.begin();
    accelgyro.initialize();

    // PID velocity control part

    // Motor 0
    pinMode(motor0Pins[0], INPUT);
    pinMode(motor0Pins[1], INPUT);
    pinMode(motor0Pins[2], OUTPUT);
    pinMode(motor0Pins[3], OUTPUT);
    pinMode(motor0Pins[4], OUTPUT);
    // Motor 1
    pinMode(motor1Pins[0], INPUT);
    pinMode(motor1Pins[1], INPUT);
    pinMode(motor1Pins[2], OUTPUT);
    pinMode(motor1Pins[3], OUTPUT);
    pinMode(motor1Pins[4], OUTPUT);

    // attach interrupt for encoder
    attachInterrupt(digitalPinToInterrupt(motor0Pins[0]), readEncoder0, RISING);
    attachInterrupt(digitalPinToInterrupt(motor1Pins[0]), readEncoder1, RISING);
}

void loop()
{
    // Set target speed
    float targetSpeed[] = {80,30}; // left motor, right motor (in rpm)
    // PIDvelocityControl(targetSpeed);

    // IMU part
    updateAccel();

    // PID drive straight part
    driveStraight(80);
}

#pragma region [PID drive straight functions]

// Use PID control to drive straight, use target rotational acceleration of the z axis of the IMU to 0
void driveStraight(int straightSpeed)
{
    int offset = 160;
    // Get the current rotational acceleration of the z axis of the IMU
    int16_t alpha_z = (curr_accele_filtered[Z_angular] - offset);
    
    // PID parameters, need to be futher tested
    float Kp = 0.1;
    float Ki = 0.1;
    float Kd = 0.1;

    // PID control
    int e = 0 - alpha_z;

    // Integral
    angular_e_integral += e;

    // Derivative
    int de = e - angular_e_prev;
    angular_e_prev = e;

    // Driving signal
    int drivingSignal = Kp * e + Ki * angular_e_integral + Kd * de;

    // Set the speed of the left and right motor, notice that direction need to be considered
    float targetSpeed[] = {straightSpeed + drivingSignal, straightSpeed - drivingSignal}; // left motor, right motor (in rpm)
    PIDvelocityControl(targetSpeed);

}

#pragma endregion

#pragma region [PID velocity control]

void PIDvelocityControl(float targetSpeed[])    
{
    /* Due to the encoder direction definition,
     *  the right motor need to turn reversely to let the car go straight
    */
    targetSpeed[1] = -targetSpeed[1];

    // read the position in an atomic block
    int pos[2] = {0, 0};
    float velocity2[2] = {0, 0};
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pos[0] = pos_i[0];
        pos[1] = pos_i[1];
        velocity2[0] = velocity_i[0];
        velocity2[1] = velocity_i[1];
    }

    // Compute velocity with method 1
    long currT[] = {micros(), micros()};
    float deltaT[] = {((float)(currT[0] - prevT[0])) / 1.0e6, ((float)(currT[1] - prevT[1])) / 1.0e6};
    float velocity1[] = {(pos[0] - posPrev[0]) / deltaT[0], (pos[1] - posPrev[1]) / deltaT[1]};
    posPrev[0] = pos[0];
    posPrev[1] = pos[1];
    prevT[0] = currT[0];
    prevT[1] = currT[1];

    // Convert count/sec to RPM (517 means ticks per revolution)
    float v1[] = {velocity1[0] / 517.0 * 60.0, velocity1[1] / 517.0 * 60.0};
    float v2[] = {velocity2[0] / 517.0 * 60.0, velocity2[1] / 517.0 * 60.0};

    // Low-pass filter (25 Hz cutoff)
    v1Filt[0] = 0.854*v1Filt[0] + 0.0728*v1[0] + 0.0728*v1Prev[0];
    v1Filt[1] = 0.854*v1Filt[1] + 0.0728*v1[1] + 0.0728*v1Prev[1];
    v1Prev[0] = v1[0];
    v1Prev[1] = v1[1];
    v2Filt[0] = 0.854*v2Filt[0] + 0.0728*v2[0] + 0.0728*v2Prev[0];
    v2Filt[1] = 0.854*v2Filt[1] + 0.0728*v2[1] + 0.0728*v2Prev[1];
    v2Prev[0] = v2[0];
    v2Prev[1] = v2[1];

    // Compute the control signal "u"
    float Kp[] = {5, 5};
    float Ki[] = {5, 5};
    float e[] = {targetSpeed[0] - v1Filt[0], targetSpeed[1] - v1Filt[1]};
    eintegral[0] += e[0] * deltaT[0];
    eintegral[1] += e[1] * deltaT[1];

    float u[] = {Kp[0]*e[0] + Ki[0]*eintegral[0], Kp[1]*e[1] + Ki[1]*eintegral[1]};

    // Set the motor speed and dirction
    int dir[] = {0, 0};
    //  Motor 0
    if (u[0] > 0)
    {
        dir[0] = 1;
    }
    else if (u[0] < 0)
    {
        dir[0] = -1;
    }
    else
    {
        dir[0] = 0;
    }
    //  Motor 1
    if (u[1] > 0)
    {
        dir[1] = 1;
    }
    else if (u[1] < 0)
    {
        dir[1] = -1;
    }
    else
    {
        dir[1] = 0;
    }

    // Set the motor speed
    int pwr[] = {(int)fabs(u[0]), (int)fabs(u[1])};
    if(pwr[0] > 255)
    {
        pwr[0] = 255;
    }
    if(pwr[1] > 255)
    {
        pwr[1] = 255;
    }

    setMotor0(dir[0], pwr[0]);
    setMotor1(dir[1], pwr[1]);

    /*
    // print motor velocity
    Serial.print("Motor Speed:[ left: "); Serial.print(v2Filt[0]); Serial.print("\t right: "); Serial.print(v2Filt[1]); Serial.println("\t]");
    Serial.print("Target Speed:[ left: "); Serial.print(targetSpeed[0]); Serial.print("\t right: "); Serial.print(targetSpeed[1]); Serial.println("\t]");
    // Seperation line
    for(int i = 0; i<30;i++)
    {
      Serial.print("-");
    }
    Serial.println();
    Serial.println();
    */
}

// set Motor speed funtions
void setMotor0(int dir, int pwmVal)
{
    analogWrite(motor0Pins[2], pwmVal);
    if (dir == 1)
    {
        digitalWrite(motor0Pins[3], HIGH);
        digitalWrite(motor0Pins[4], LOW);
    }
    else if(dir == -1)
    {
        digitalWrite(motor0Pins[3], LOW);
        digitalWrite(motor0Pins[4], HIGH);
    }
    else
    {
        digitalWrite(motor0Pins[3], LOW);
        digitalWrite(motor0Pins[4], LOW);
    }
}
void setMotor1(int dir, int pwmVal)
{
    analogWrite(motor1Pins[2], pwmVal);
    if (dir == 1)
    {
        digitalWrite(motor1Pins[3], HIGH);
        digitalWrite(motor1Pins[4], LOW);
    }
    else if(dir == -1)
    {
        digitalWrite(motor1Pins[3], LOW);
        digitalWrite(motor1Pins[4], HIGH);
    }
    else
    {
        digitalWrite(motor1Pins[3], LOW);
        digitalWrite(motor1Pins[4], LOW);
    }
}

// read encoder function
void readEncoder0()
{
    // Read encoder B when ECNA goes high
    int b = digitalRead(motor0Pins[1]) ? 1 : -1;
    int increment = 0;
    if (b>0)
    {
        increment = 1;
    }
    else
    {
        increment = -1;
    }
    pos_i[0] += increment;

    // Compute velocity with method 2
    long currT = micros();
    float deltaT = ((float)(currT - prevT_i[0])) / 1.0e6;
    velocity_i[0] = increment / deltaT;
    prevT_i[0] = currT;
}
void readEncoder1()
{
    // Read encoder B when ECNA goes high
    int b = digitalRead(motor1Pins[1]) ? 1 : -1;
    int increment = 0;
    if (b>0)
    {
        increment = 1;
    }
    else
    {
        increment = -1;
    }
    pos_i[1] += increment;

    // Compute velocity with method 2
    long currT = micros();
    float deltaT = ((float)(currT - prevT_i[1])) / 1.0e6;
    velocity_i[1] = increment / deltaT;
    prevT_i[1] = currT;
}

#pragma endregion


#pragma region [Accelerometer]

// low pass filter the accelerometer data
void updateAccel()
{
    int16_t accele_original[]={0,0,0,0,0,0}; // ax, ay, az, gx, gy, gz
    accelgyro.getMotion6(&accele_original[0], &accele_original[1], &accele_original[2], &accele_original[3], &accele_original[4], &accele_original[5]);

    // low pass filter the accelerometer data
    for (int  i = 0; i < 6; i++)
    {
        curr_accele_filtered[i] = 0.969 * prev_accele_filtered[i] + 0.0155 * accele_original[i] + 0.0155 * prev_accele_original[i];

        // update previous values
        prev_accele_original[i] = curr_accele_original[i];
        prev_accele_filtered[i] = curr_accele_filtered[i];
    }

    // print values

}


#pragma endregion
