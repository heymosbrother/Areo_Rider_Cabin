// try to source class in other files
#include "Tracer.h"

// create tracers
Tracer tracer_left(A0);

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    Serial.println(tracer_left.getAnalog());
    delay(100);
}
