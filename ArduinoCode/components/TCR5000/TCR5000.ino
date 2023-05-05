class Tracer 
{
    private:
    // pin numbers
    int analog_pin;

    public: 
    Tracer(int analog_pin)
    {
        this->analog_pin = analog_pin;
        // set pin mode
        pinMode(analog_pin, INPUT);
    }

    int getAnalog()
    {
        return analogRead(analog_pin);
    }

    bool onBlack()
    {
        // when on black, the analog value will greater than 100
        return getAnalog() > 100;
    }
};

Tracer tracer1(2, A0);

void setup()
{
    Serial.begin(9600);
}

void loop()
{

}
