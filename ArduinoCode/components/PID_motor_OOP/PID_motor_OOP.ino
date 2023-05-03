class Motor_with_Encoder
{
    public:
    // motor pins
    int pwm_pin;
    int inPlus_pin;
    int inMinus_pin;
    // encoder pins
    int ENCA_pin;
    int ENCB_pin;

    // Variables
    //  motor properties
    int position;
    float v_original;
    float v_filtered;
    //  functional variables
    float time_prev;
    int position_prev;
    float v_original_prev;
    float v_filtered_prev;
    //  PID variables
    float e_integral;
    float e_prev;
    //  PID parameters
    float Kp, Ki, Kd;

    // Constructor
    Motor_with_Encoder(int pwm_pin, int inPlus_pin, int inMinus_pin, int ENCA_pin, int ENCB_pin)
    {
        // assign pins
        this->pwm_pin = pwm_pin;
        this->inPlus_pin = inPlus_pin;
        this->inMinus_pin = inMinus_pin;
        this->ENCA_pin = ENCA_pin;
        this->ENCB_pin = ENCB_pin;

        // initialize variables
        position = 0;
        v_original = 0;
        v_filtered = 0;
        time_prev = 0;
        position_prev = 0;
        v_original_prev = 0;
        v_filtered_prev = 0;
        e_integral = 0;
        e_prev = 0;
        // PID parameters
        Kp = 5, Ki = 1, Kd = 0;

        attachInterrupt(digitalPinToInterrupt(ENCA_pin), readEncoder_left, RISING);
    }

    // Methods
    static void readEncoder()
    {
        int b = digitalRead(ENCB_pin) ? 1 : -1;
        int increment = 0;
        if (b>0) increment = 1;
        else increment = -1;
        position += increment;
    }

    float setMotor(int pwmVal, int dir)
    {
        analogWrite(pwm_pin, pwmVal);
        if (dir == 1)
        {
            digitalWrite(inPlus_pin, HIGH);
            digitalWrite(inMinus_pin, LOW);
        }
        else if(dir == -1)
        {
            digitalWrite(inPlus_pin, LOW);
            digitalWrite(inMinus_pin, HIGH);
        }
        else
        {
            digitalWrite(inPlus_pin, LOW);
            digitalWrite(inMinus_pin, LOW);
        }
    }

    void SetVelocity(float target_speed)
    {
        // get current time
        float time_curr = micros();
        // calculate velocity (rpm)
        float v =  ((position - position_prev) / (((float)(time_curr - time_prev))/1.0e6) ) / 517 * 60;
        // filter the velocity
        float v_filt = 0.854 * v_filtered_prev + 0.0728 * v + 0.0728 * v_original_prev;
        // update variables
        v_filtered = v_filt;
        time_prev = time_curr;
        v_original_prev = v;
        v_filtered_prev = v_filt;
        position_prev = position;
        
        // compute control signal u using PID
        float e = target_speed - v_filt;
        e_integral += e;
        float u = Kp * e + Ki * e_integral + Kd * (e - e_prev);

        // Set motor speed and direction
        int dir = 0;
        if (u > 0) dir = 1;
        else if (u < 0) dir = -1;
        else dir = 0;
        // calculate pwm value
        int pwmVal = (int)fabs(u);
        if (pwmVal > 255) pwmVal = 255;
        // set motor
        setMotor(pwmVal, dir);

        return;
    }
};

void setup()
{
    // declare left and right motors
    Motor_with_Encoder motor_left(9, 7, 6, 2, 13);
}

void loop()
{
    // set target speed
    float target_speed = 100;
    // set motor speed
    motor_left.SetVelocity(target_speed);
}
