#include <util/atomic.h>

// Pins for motor0 and motor1
// Motor 0
const int motor0Pins[] = {2, 13, 9, 7, 6};// ENCA, ENCB, PWM, IN1, IN2
// Motor 1
const int motor1Pins[] = {3, 12, 10, 4, 5};// ENCA, ENCB, PWM, IN1, IN2

// global variables
long prevT[] = {0, 0};
int posPrev[] = {0, 0};

// Use the "volatile" directive for variables
// used in an interrupt
volatile int pos_i[] = {0, 0};
volatile float velocity_i[] = {0, 0};
volatile long prevT_i[] = {0, 0};

float v1Filt[] = {0, 0};
float v1Prev[] = {0, 0};
float v2Filt[] = {0, 0};
float v2Prev[] = {0, 0};

float eintegral[] = {0, 0};

void setup()
{
    Serial.begin(115200);
    
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

    attachInterrupt(digitalPinToInterrupt(motor0Pins[0]), readEncoder0, RISING);
    attachInterrupt(digitalPinToInterrupt(motor1Pins[0]), readEncoder1, RISING);
}

void loop()
{
    // Set target speed
    float targetSpeed[] = {150,0};

    PIDvelocityControl(targetSpeed);
}


void PIDvelocityControl(float targetSpeed[])
{
    // read the position in an atomic block
    // to avoid potential misreads
    int pos[] = {0, 0};
    float velocity2[] = {0, 0};
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        pos[0] = pos_i[0];
        velocity2[0] = velocity_i[0];
        pos[1] = pos_i[1];
        velocity2[1] = velocity_i[1];
    }

    // Compute velocity with method 1
    long currT[] = {micros(), micros()};
    float deltaT[] = {((float)(currT[0] - prevT[0])) / 1.0e6, ((float)(currT[1] - prevT[1])) / 1.0e6};
    float velocity1[] = {(pos[0] - posPrev[0]) / deltaT[0], (pos[1] - posPrev[1]) / deltaT[1]};
    posPrev[0] = pos[0];
    posPrev[1] = pos[1];
    prevT[0] = currT[0];
    prevT[0] = currT[1];

    // Convert count/s to RPM
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

    // Compute the control signal u
    float Kp = 5;
    float Ki = 5;
    float e[] = {targetSpeed[0] - v1Filt[0], targetSpeed[1] - v1Filt[1]};
    eintegral[0] += e[0] * deltaT[0];
    eintegral[1] += e[1] * deltaT[1];

    float u[] = {Kp*e[0] + Ki*eintegral[0], Kp*e[1] + Ki*eintegral[1]};

    // Set the motor speed and direction
    int dir[] = {0, 0};
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

    // Print the data
    
}

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