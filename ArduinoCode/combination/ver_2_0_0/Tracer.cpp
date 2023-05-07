#include "Tracer.h"
#include <Arduino.h>

// Constructor
Tracer::Tracer(int analog_pin)
{
    this->analog_pin = analog_pin;
    // set pin mode
    pinMode(analog_pin, INPUT);
}

int Tracer::getAnalog()
{
    return analogRead(analog_pin);
}

bool Tracer::onBlack()
{
    // when on black, the analog value will greater than 100
    return getAnalog() > 100;
}