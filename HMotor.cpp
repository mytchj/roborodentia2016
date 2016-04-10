// Robot Localization via sonar
#include <Arduino.h>
#include "HMotor.h"
#include "AFMotor.h"
#include "NewPing.h"
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
  
  void HMotor::drive(uint8_t speed, uint8_t direction) {
    if (direction == FORWARD) {
      digitalWrite(a, HIGH);
      digitalWrite(b, LOW);
    }
    else if (direction == BACKWARD) {
      digitalWrite(a, LOW);
      digitalWrite(b, HIGH);
    }
    else {
      digitalWrite(a, LOW);
      digitalWrite(b, LOW);
    }
    
    analogWrite(s, speed);
  };

