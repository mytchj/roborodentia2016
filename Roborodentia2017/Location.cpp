// Robot Localization via sonar
#include <Arduino.h>
#include "Location.h"
#include "config.h"

// IRsensors
static const int irRead[IRCOUNT] = {A2, A3, A4, A5, A6, A7};
volatile unsigned long int Location::encoderCount = 0;
int Location::irRaw[IRCOUNT];
boolean Location::ir[IRCOUNT];
  
void Location::Init(void) {
  pinMode(ENCODERPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODERPIN), encoderISR, CHANGE);
  // IR Sensors
  for (int i = 0; i < IRCOUNT; i++)
    pinMode(irRead[i], INPUT);
}

void Location::encoderISR() {
  Location::encoderCount++;
}

unsigned long Location::getEncoder() {  
  return Location::encoderCount;
}

void Location::resetEncoder() {
  Location::encoderCount = 0;
}

int Location::printEncoderCount() {  
#if DEBUG_ENABLED
  Serial.println(Location::encoderCount);
#endif
  return Location::encoderCount;
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

#if DEBUG_ENABLED

void Location::printInfrared() {
  Location::updateInfrared();
  Serial.println("INFRARED: ");
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
  Serial.println("Raw INFRARED: ");
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

#endif
