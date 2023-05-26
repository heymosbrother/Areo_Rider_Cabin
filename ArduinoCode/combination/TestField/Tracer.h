#ifndef TRACER_H
#define TRACER_H

class Tracer {
public:
    Tracer(int analog_pin1, int analog_pin2, int analog_pin3, int analog_pin4); // Constructor
    bool OnBlack(int numTag);
    
private:
    int aPins[4];
    int getAnalog(int numTag);
    bool onBlack(int numTag);
};

#endif // TRACER_H