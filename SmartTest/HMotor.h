#ifndef HMotor_h
#define HMotor_h

#include <Arduino.h>

class HMotor {
  private:
    uint8_t a;
    uint8_t b;
    uint8_t s;
    
  public:
    HMotor(uint8_t aPin, uint8_t bPin, uint8_t speedPin);
    void drive(uint8_t speed, uint8_t direction);
};

#endif
