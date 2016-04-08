/* Global */
#define DEBUG_ENABLED 1

/* Main Stuff */
// Codes for the motor function.
#define JOYCOUNT 3
#define POTCOUNT 3
#define SERVOCOUNT 3
#define MOTORCOUNT 4
#define IRCOUNT 6

#define DEADZONE 20

/* Location.cpp stuff */
#define TWOFEET 61 // centimetre
#define FOURFEET 122 // centimetre

#define SONAR_NUM 4 // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

#define FULL_SPEED 255
#define HALF_SPEED 160
#define MEH_SPEED 120
#define SLOW_SPEED 80

#define BUTTONPIN 35
#define LEDPIN 52

#define XDIR 0
#define YDIR 1

// From AF_Motor (wheels)
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// For IRSAVE
#define FRONTLEFT 0x01
#define FRONTRIGHT 0x02
#define BACKLEFT 0x04
#define BACKRIGHT 0x08

// For DCMotors
#define DRIVE true
#define REVERSE false

#define OPEN 1
#define CLOSE 2
#define STOP 3

//Margin of error for distance to target
#define MARGIN 3

#define SONAR_AVERAGING_PRECISION 7
#define FUDGING_DISTANCE 15

#define BUTTON_PRESS() joyXYB[2]==-256

#define XCLOSE() ((Location::myLoc.x >= toLoc.x - MARGIN) && (Location::myLoc.x <= toLoc.x + MARGIN))
#define YCLOSE() ((Location::myLoc.y <= toLoc.y - MARGIN) && (Location::myLoc.y <= toLoc.y + MARGIN))
