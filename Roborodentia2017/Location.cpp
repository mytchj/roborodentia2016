// Robot Localization via sonar
#include <Arduino.h>
#include "Location.h"
#include "config.h"

// IRsensors
static const int irRead[IRCOUNT] = {A2, A3, A4, A5, A6, A7};
volatile unsigned long int Location::encoderCountx = 0;
volatile unsigned long int Location::encoderCounty = 0;
int Location::irRaw[IRCOUNT];
boolean Location::ir[IRCOUNT];
  
void Location::Init(void) {
  pinMode(ENCODERPINX, INPUT_PULLUP);
  pinMode(ENCODERPINY, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODERPINX), encoderISRx, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODERPINY), encoderISRy, CHANGE);
  // IR Sensors
  for (int i = 0; i < IRCOUNT; i++)
    pinMode(irRead[i], INPUT);
}

void Location::encoderISRx() {
  Location::encoderCountx++;
}
void Location::encoderISRy() {
  Location::encoderCounty++;
}

unsigned long Location::getEncoderx() {  
  return Location::encoderCountx;
}

unsigned long Location::getEncodery() {  
  return Location::encoderCounty;
}

void Location::resetEncoders() {
  Location::encoderCountx = 0;
  Location::encoderCounty = 0;
}

void Location::printEncoderCount() {  
  Serial.print(Location::encoderCountx);
  Serial.print("  \t  ");
  Serial.println(Location::encoderCounty);
}

boolean* Location::updateInfrared() {
  for (int i = 0; i < IRCOUNT; i++)
    ir[i] = irMap(irRaw[i] = analogRead(irRead[i]));
  return ir;
}

bool Location::irMap(int luminosity) {
  if (luminosity > 720)
    return true;
  return false;
}

void Location::printInfrared() {
  Location::updateInfrared();
  //Serial.println("INFRARED: ");
  Serial.print("[");
  Serial.print((ir[0]) ? "#" : "_");
  Serial.print((ir[1]) ? "#" : "_");
  Serial.print((ir[2]) ? "#" : "_");
  Serial.println("]");

  Serial.print("[");
  Serial.print((ir[3]) ? "#" : "_");
  Serial.print((ir[4]) ? "#" : "_");
  Serial.print((ir[5]) ? "#" : "_");
  Serial.println("]");
}

void Location::printRawInfrared() {
  Location::updateInfrared();
  //Serial.println("Raw INFRARED: ");
  Serial.print("[");
  Serial.print(irRaw[0]);  Serial.print("\t");
  Serial.print(irRaw[1]);  Serial.print("\t");
  Serial.print(irRaw[2]);  Serial.print("\t");
  Serial.println("]");

  Serial.print("[");
  Serial.print(irRaw[3]);  Serial.print("\t");
  Serial.print(irRaw[4]);  Serial.print("\t");
  Serial.print(irRaw[5]);  Serial.print("\t");
  Serial.println("]");
}

