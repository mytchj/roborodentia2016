// Robot Localization via sonar
#include <Arduino.h>
#include "NewPing.h"
#include "Location.h"
#include "config.h"

// IRsensors
static const int irRead[IRCOUNT] = {A0, A1, A2, A3, A4, A5};
int Location::irRaw[IRCOUNT];
boolean Location::ir[IRCOUNT];

#if DEBUG_ENABLED
static const int joystick[JOYCOUNT] = {A8, A9, A7};
int Location::joyXYB[JOYCOUNT];
#endif

// Points of Interest
locationCoordinates myLoc = {0, 0};
static const locationCoordinates pickupPegs[2] = {{0, TWOFEET}, {0, TWOFEET}};
static const locationCoordinates scoringPegs[2] = {{FOURFEET, 0}, {-FOURFEET, 0}};
static const locationCoordinates barrier = {-TWOFEET, 0};


// Static Variables
uint8_t Location::cm[SONAR_NUM];         // Where the ping distances are stored.
NewPing Location::sonar[SONAR_NUM] = {
  NewPing(22, 23, MAX_DISTANCE), // trig, echo
  NewPing(24, 25, MAX_DISTANCE),
  NewPing(26, 27, MAX_DISTANCE),
  NewPing(28, 29, MAX_DISTANCE)
};
  
void Location::Init(void) {
#if DEBUG_ENABLED
  // Joystick Controls
  for (int i = 0; i < JOYCOUNT; i++)
    pinMode(joystick[i], INPUT);  
#endif

  // IR Sensors
  for (int i = 0; i < IRCOUNT; i++)
    pinMode(irRead[i], INPUT);
}

uint8_t* Location::updateSonar(void) {
  for (uint8_t i = 0; i < SONAR_NUM; i++) // Loop through all the sensors.
    cm[i] = sonar[i].ping() / US_ROUNDTRIP_CM;
  return (uint8_t*)cm;
}

boolean* Location::updateInfrared() {
  for (int i = 0; i < IRCOUNT; i++)
    ir[i] = irMap(irRaw[i] = analogRead(irRead[i]));
  return ir;
}

bool Location::irMap(int luminosity) {
  if (luminosity > 800)
    return true;
  return false;
}

#if DEBUG_ENABLED

void Location::printSonar() {
  Location::updateSonar();
  Serial.print("SONAR: ");
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    if (cm[i] != NO_ECHO) {
      Serial.print(i);
      Serial.print("=");
      Serial.print(cm[i]);
      Serial.print("cm ");
    }
  }
  Serial.println();
}

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

uint8_t Location::joystickDirection(int num) {
  if (num > DEADZONE)
    return FORWARD;
  if (num < -DEADZONE)
    return BACKWARD;
  return BRAKE;
}

int* Location::updateJoystick() {
  for (int i = 0; i < JOYCOUNT; i++)
    joyXYB[i] = (analogRead(joystick[i]) - 512)/2;
  return joyXYB;
}

void Location::printJoystickPosition() {
  Serial.print("Joy: ");
  Serial.print("X = ");
  Serial.print(joyXYB[0]);
  Serial.print(" Y = ");
  Serial.print(joyXYB[1]);
  Serial.print(" B = ");
  Serial.print(joyXYB[2]);
  Serial.println(); 
}

#endif
