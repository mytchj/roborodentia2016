#ifndef Location_h
#define Location_h

#include <Arduino.h>
#include "config.h"

class Location {
  private:
    static int irRaw[IRCOUNT];
    static boolean ir[IRCOUNT];

  public:
  volatile static long int encoderCount;
    static void Init();
#if DEBUG_ENABLED
    static void printInfrared();
    static void printRawInfrared();
#endif
    static boolean* updateInfrared();
    static bool irMap(int);
    static void encoderISR();
    static int printEncoderCount();
    static int getEncoder();
};

#endif
