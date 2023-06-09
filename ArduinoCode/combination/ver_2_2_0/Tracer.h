#ifndef TRACER_H
#define TRACER_H

class Tracer {
public:
    Tracer(int analogPin); // Constructor
    int getAnalog();
    bool onBlack();
        
private:
    int aPin;

};

#endif // TRACER_H
