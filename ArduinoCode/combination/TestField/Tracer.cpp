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
Tracer::Tracer(int analog_pin1, int analog_pin2, int analog_pin3, int analog_pin4)
{
    aPins[0] = analog_pin1;
    aPins[1] = analog_pin2;
    aPins[2] = analog_pin3;
    aPins[3] = analog_pin4;
    for (int i = 0; i < 4; i++) pinMode(aPins[i], INPUT);
}

int Tracer::getAnalog(int numTag)
{
    numTag--;
    if (numTag < 1 || numTag > 4) return; // avoid error
    return analogRead(aPins[numTag]);
}

bool Tracer::onBlack(int numTag)
{
    numTag--;
    if (numTag < 1 || numTag > 4) return;
    // when on black, the analog value will greater than 100
    return getAnalog(numTag) > 100;
}

bool Tracer::OnBlack(int numTag)
{
    numTag--;
    if (numTag < 1 || numTag > 4) return;
    // when on black, the analog value will greater than 100
    return getAnalog(numTag) > 100;
}
