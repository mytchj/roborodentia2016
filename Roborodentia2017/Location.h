#ifndef Location_h
#define Location_h

#include <Arduino.h>
#include "config.h"

class Location {
  private:
    static int irRaw[IRCOUNT];
    static boolean ir[IRCOUNT];

  public:
  volatile static unsigned long int encoderCountx;
  volatile static unsigned long int encoderCounty;
    static void Init();
#if DEBUG_ENABLED
    static void printInfrared();
    static void printRawInfrared();
#endif
    static boolean* updateInfrared();
    static bool irMap(int);
    static void encoderISRx();
    static void encoderISRy();
    static void printEncoderCount();
    static unsigned long int getEncoderx();
    static unsigned long int getEncodery();
    static void resetEncoders();
};

#endif
