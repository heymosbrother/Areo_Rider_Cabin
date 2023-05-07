#ifndef TRACER_H
#define TRACER_H

class Tracer {
public:
  Tracer(int analog_pin); // Constructor
  int getAnalog();
  bool onBlack();

private:
    int analog_pin;
};

#endif // TRACER_H