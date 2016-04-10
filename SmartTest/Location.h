#ifndef Location_h
#define Location_h

#include <Arduino.h>
#include "NewPing.h"
#include "config.h"

typedef struct {int x; int y;} locationCoordinates;

class Location {
  private:  
#if DEBUG_ENABLED
    static int joyXYB[JOYCOUNT];
#endif
    static int irRaw[IRCOUNT];
    static boolean ir[IRCOUNT];
    static uint8_t cm[SONAR_NUM];

  public:
    static locationCoordinates myLoc;
    static void Init();
    static NewPing sonar[];
#if DEBUG_ENABLED
    static int* updateJoystick();
    static uint8_t joystickDirection(int);
    
    static void printJoystickPosition();
    static void printSonar();
    static void printInfrared();
    static void printRawInfrared();
#endif
    static boolean* updateInfrared();
    static bool irMap(int);
    static uint8_t* updateSonarQuick();
    static uint8_t* updateSonar();
    static void echoCheck();
};

#endif
