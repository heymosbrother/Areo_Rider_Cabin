#include "Tracer.h"
#include <Arduino.h>

/*
    To use this scrips, you need to place the tracers like this:
             
           ____1____2      ↑ Forward direction
          |         |
          |         |
          |         |
          |         |
          |         |
           ____3____4

*/

// Constructor
Tracer::Tracer(int analogPin)
{
    aPin = analogPin;
    pinMode(aPin, INPUT);
}

int Tracer::getAnalog()
{
    return analogRead(aPin);
}

bool Tracer::onBlack()
{
    // when on black, the analog value will greater than 100
    return getAnalog() > 100;
}

