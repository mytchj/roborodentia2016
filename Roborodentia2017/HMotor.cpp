#include "HMotor.h"
#include "Location.h"
#include "config.h"

  HMotor::HMotor(uint8_t aPin, uint8_t bPin, uint8_t speedPin) {
    a = aPin;
    b = bPin;
    s = speedPin;
    
    pinMode(a, OUTPUT);
    pinMode(b, OUTPUT);
    pinMode(s, OUTPUT);
  };
  
  void HMotor::drive(int speed, uint8_t direction) {
    if (direction == BACKWARD) {
      digitalWrite(a, HIGH);
      digitalWrite(b, LOW);
    }
    else if (direction == FORWARD) {
      digitalWrite(a, LOW);
      digitalWrite(b, HIGH);
    }
    else if (direction == BRAKE){
      digitalWrite(a, HIGH);
      digitalWrite(b, HIGH);
    }
    else {
      digitalWrite(a, LOW);
      digitalWrite(b, LOW);
    }
    speed = abs(map(speed, -512, 512, -255, 255));
    if(speed < 50)
      speed = 0;
    analogWrite(s, speed);
  };

