/* Global */
#define DEBUG_ENABLED 1

/* Main Stuff */
// Codes for the motor function.
#define POTCOUNT 3
#define SERVOCOUNT 3
#define MOTORCOUNT 4
#define IRCOUNT 6

/* Location.cpp stuff */
#define FULL_SPEED 255
#define HALF_SPEED 160
#define MEH_SPEED 120
#define SLOW_SPEED 80

#define BUTTONPIN 35
#define LBUTTON 32
#define RBUTTON 33
#define LEDPIN 52
#define ENCODERPIN 20

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
#define GOLEFT 0x05
#define GORIGHT 0x0A

// For DCMotors
#define DRIVE true
#define REVERSE false

#define OPEN 1
#define CLOSE 2
#define STOP 3
