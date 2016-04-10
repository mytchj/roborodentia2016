#include <Math.h>
#include <Servo.h>
#include <AFMotor.h>
#include "HMotor.h"
#include "Location.h"
#include "config.h"

// Motor & Servo Globals
Servo hservo[2];
Servo cservo[3];
AF_DCMotor motor1(4);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor *slide[SERVOCOUNT] = {&motor1, &motor2, &motor3};

static int startTime;
static uint8_t* distance; 
static boolean* ir; //[IRCOUNT];    
static uint8_t clampStatus[SERVOCOUNT] = {OPEN, OPEN, OPEN};

// Usage example
  //distance = Location::updateSonar();
  //ir = Location::updateInfrared();
  

#if DEBUG_ENABLED
static bool joyStickEnabled = false;
static int* joyXYB; //[JOYCOUNT];
static char incomingByte;
#endif 


HMotor hmotor1(39, 38, 2);
HMotor hmotor2(40, 41, 45);
HMotor hmotor3(51, 50, 46);
HMotor hmotor4(49, 48, 44);
HMotor *hm[MOTORCOUNT] = {&hmotor1, &hmotor2, &hmotor3, &hmotor4};

void setup() {
#if DEBUG_ENABLED
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.println("Initializing");
#endif

  pinMode(BUTTONPIN, INPUT);
  pinMode(LEDPIN, OUTPUT);
  Location::Init();

  //while (!digitalRead(BUTTONPIN));

  startTime = millis();
  
  lift(1, RELEASE);
  lift(0, RELEASE);
  lift(2, RELEASE);

#if DEBUG_ENABLED
  Serial.println("Initialized");
#endif
}

void loop() {
  if (digitalRead(BUTTONPIN) && millis() - startTime > 10000) {    
    #if DEBUG_ENABLED
      Serial.println("Restarting");
    #endif
    restart();
  }
  /*
  if (millis() - startTime > 179000) // Stop 1 second shy of 3 minutes
    endMatch();
*/
#if DEBUG_ENABLED

  if (joyStickEnabled) {
    joyXYB = Location::updateJoystick();
    joyStickDrive();
  }
  
  if (serialRead())
    serialDo();

#else

  dirDrive(XDIR, BACKWARD, FULL_SPEED);
  delay(500);
  dirDrive(XDIR, BACKWARD, HALF_SPEED);
  delay(500);
  dirDrive();
  delay(500);
  dirDrive(XDIR, FORWARD, FULL_SPEED);
  delay(500);
  dirDrive(XDIR, FORWARD, HALF_SPEED);
  delay(500);


#endif    

}

void restart() {
  digitalWrite(LEDPIN, LOW);
  asm volatile("  jmp 0"); // do not trust the jump
}

void endMatch() {
  #if DEBUG_ENABLED
    Serial.println("Alert: Match Over");
  #endif
  dirDrive();
  hservoDetach();
  for (;;) {
    toggleLED();
    if (digitalRead(BUTTONPIN)) {    
    #if DEBUG_ENABLED
      Serial.println("Alert: Restarting");
    #endif
    restart();
  }
  }
}

void hump(int direction) {
  if (direction < BRAKE) {
    hservoAttach();
    hservo[(direction + 1) % 2].writeMicroseconds(1300);
    hservo[direction % 2].writeMicroseconds(1700); 
  } else {
    hservoDetach();
  }
}

void lift(uint8_t selector, uint8_t pos) {
  slide[selector]->setSpeed(FULL_SPEED);
  slide[selector]->run(pos);
  if (pos == RELEASE)
    slide[selector]->setSpeed(FULL_SPEED);
}

// select state manually
void clamp(uint8_t selector, uint8_t state) {
  if (state == CLOSE) {
    cservoAttach(selector);
    cservo[selector].write(0);
    clampStatus[selector] = CLOSE;
  } else if (state == OPEN) {
    cservoAttach(selector);
    cservo[selector].write(90);
    clampStatus[selector] = OPEN;
  } else if (state == STOP) {
    cservoDetach(selector);
    clampStatus[selector] = STOP;
  }
}

void cservoAttach(uint8_t selector) {
  if (selector == 0)
    cservo[0].attach(9);
  else if (selector == 1)
    cservo[1].attach(10);
  else if (selector == 2)
    cservo[2].attach(13);
}

void cservoAttach() {
  cservoAttach(0);
  cservoAttach(1);
  cservoAttach(2);
}

void cservoDetach(uint8_t selector) {
  cservo[selector].detach();
}

void cservoDetach() {
  cservoDetach(0);
  cservoDetach(1);
  cservoDetach(2);
}

void hservoAttach() {
  hservo[0].attach(40);
  hservo[1].attach(41);
}

void hservoDetach() {
  hservo[0].detach();
  hservo[1].detach();
}

void avoidDrive(uint8_t* dist) {
  for (int i = 0; i < MOTORCOUNT / 2; i++)                                                                                                                                          
    if((dist[i]) < 5 && (dist[i] > 1)) {
      hm[(i + MOTORCOUNT + 1) % MOTORCOUNT]->drive(FULL_SPEED, FORWARD);
      hm[(i + MOTORCOUNT - 1) % MOTORCOUNT]->drive(FULL_SPEED, FORWARD);
    } else {
      hm[(i + MOTORCOUNT + 1) % MOTORCOUNT]->drive(0, RELEASE);
      hm[(i + MOTORCOUNT - 1) % MOTORCOUNT]->drive(0, RELEASE);
    }
}

void spinDrive(int degrees, bool direction) {
  hm[0]->drive(FULL_SPEED, direction ? FORWARD : BACKWARD);                    
  hm[1]->drive(FULL_SPEED, direction ? FORWARD : BACKWARD);
  hm[2]->drive(FULL_SPEED, direction ? BACKWARD : FORWARD);
  hm[3]->drive(FULL_SPEED, direction ? BACKWARD : FORWARD);
  
  delay(degrees);
  for (int i = 0; i < MOTORCOUNT; i++) {
    hm[i]->drive(0, RELEASE);
  }
}

void singleDrive(uint8_t num, uint8_t direction) {
  hm[num]->drive(FULL_SPEED, direction);
}

void singleDrive(uint8_t num, uint8_t direction, uint8_t speed) {
  hm[num]->drive(speed, direction);
}

void dirDrive(bool axis, uint8_t direction, uint8_t speed) {
  hm[axis ? 0 : 1]->drive(speed, direction);
  hm[axis ? 2 : 3]->drive(speed, direction);
}

void dirDrive(bool axis, uint8_t direction) {
  dirDrive(axis, direction, FULL_SPEED);
}

void dirDrive(bool axis) {
  dirDrive(axis, RELEASE);
}

void dirDrive() {
  hm[0]->drive(0, RELEASE);
  hm[1]->drive(0, RELEASE);
  hm[2]->drive(0, RELEASE);
  hm[3]->drive(0, RELEASE);
}

#if DEBUG_ENABLED

void joyStickDrive() {
  for (int i = 0; i < MOTORCOUNT; i++)                                                                                                                                          
    hm[i]->drive(abs(abs(joyXYB[i%2]) - 1), Location::joystickDirection(joyXYB[i%2]));
}

bool serialRead() {
  incomingByte = 0;
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    Serial.print("I received: ");
    Serial.println(incomingByte);
  }
  return incomingByte;
}
#endif

void wheelSpeedTest() {
  for (int i = 255; i > 0; i--) {
    dirDrive(YDIR, FORWARD, i);
    delay(10);
  }
  for (int i = 0; i < 255; i++) {
    dirDrive(YDIR, BACKWARD, i); 
    delay(10);
  }
  dirDrive(YDIR, RELEASE); 

  for (int i = 255; i > 0; i--) {
    dirDrive(XDIR, FORWARD, i);
    delay(10);
  }
  for (int i = 0; i < 255; i++) {
    dirDrive(XDIR, BACKWARD, i); 
    delay(10);
  }
  dirDrive(XDIR, RELEASE); 

}

void toggleLED() {
  static int status = LOW;
  digitalWrite(LEDPIN, status = !status);
}

void withdraw() {
  // Get out of the way of middle clamp
  clamp(0, CLOSE);
  clamp(2, CLOSE);

  //Grab middle
  clamp(1, CLOSE);
  delay(100);
  lift(1, FORWARD);

  
  lift(0, BACKWARD);
  lift(2, BACKWARD);
  clamp(0, OPEN);
  clamp(2, OPEN);
  delay(1000);
  clamp(0, CLOSE);
  clamp(2, CLOSE);
  delay(100);
  lift(0, FORWARD);
  lift(2, FORWARD);
  
  
}

void deposit() {
  
}

void laps(uint8_t numberOfLaps) {
  while (numberOfLaps--) {
    Serial.println("FollowForward");
    followLines(FORWARD);
    followLines(FORWARD);
    Serial.println("Withdraw");
    withdraw();
    
    Serial.println("FollowBackward");
    followLines(BACKWARD);
    followLines(BACKWARD);
    Serial.println("Deposit");
    deposit();
  }
}

void testLift() {
  uint8_t i = 0;
  while (i < POTCOUNT) {
    lift(i, FORWARD);
    delay(1000);
    lift(i, RELEASE);
    delay(1000);
    lift(i, BACKWARD);
    delay(1000);
    lift(i, RELEASE);
    delay(1000);
    i++;
  }
  
}

#if DEBUG_ENABLED

void serialDo() {
  switch (incomingByte) {
    case '`': hump(FORWARD);      break;
    case '1': hump(BACKWARD);     break;
    case '2': hump(BRAKE);        break;
    case '6': testLift();         break;
    case '9': laps(1);             break;

    case 'a': clamp(1, OPEN);            break;
    case 's': clamp(1, CLOSE);            break;
    case 'd': clamp(1, STOP);            break;
    
    case 'z': Location::printInfrared();            break;
    case 'x': Location::printRawInfrared();         break;
    case 'c': Location::printSonar();               break;
    case 'v': updateSonar();               break;

    case 'm': singleDrive(0, FORWARD, SLOW_SPEED + 60); break;  
    
    case 'q': followLines(FORWARD, 0);                        break;
    case 'w': followLines(BACKWARD, 0);                        break;
    case '[': spinDrive(175, true);              break;
    case ']': spinDrive(175, false);              break;
    case 'r': sonarDrive(XDIR, FORWARD);              break;
    case 't': sonarDrive(XDIR, BACKWARD);              break;
    
    case 'y': sonarDrive(YDIR, FORWARD);              break;
    case 'u': sonarDrive(YDIR, BACKWARD);              break;
    case 'p': dirDrive(XDIR, FORWARD, FULL_SPEED);  break;

  }
}

void toggleJoystickControl() {
  joyStickEnabled = !joyStickEnabled;
}
#endif


// Runs the front and back
void followLines(uint8_t dir, int irSave) {
  int i = 5000;
  dirDrive(XDIR, dir, HALF_SPEED);
  while(i--) {
    ir = Location::updateInfrared();

    //THIS CHECKS THE FRONT IR's
    if (ir[0] | ir[1] | ir[2]) { // Any of the front are on
      if (ir[1])
        dirDrive(XDIR, dir, HALF_SPEED);
      else if (ir[0]){
        singleDrive(2, FORWARD , SLOW_SPEED); //dir == FORWARD ? FORWARD : BACKWARD
        irSave |= FRONTLEFT;
        irSave &= ~FRONTRIGHT;
      }
      else if (ir[2]){
        singleDrive(2, BACKWARD, SLOW_SPEED); //dir == FORWARD ? BACKWARD : FORWARD
        irSave |= FRONTRIGHT;
        irSave &= ~FRONTLEFT;
      }
      else {
        dirDrive();
      }
    }
    else{
      if(irSave & FRONTLEFT)
        singleDrive(2, FORWARD, MEH_SPEED);
      else if(irSave & FRONTRIGHT)
        singleDrive(2, BACKWARD, MEH_SPEED);
      else{
        dirDrive();
      }   
      dirDrive(XDIR, dir, HALF_SPEED); //otherwise stop
    }

    //THIS CHECKS THE BACK IR's
    if (ir[3] | ir[4] | ir[5]) { // Any of the back are on
      if (ir[4])
           dirDrive(XDIR, dir, HALF_SPEED);
      else if (ir[3]){
        singleDrive(0, FORWARD, SLOW_SPEED); //dir == FORWARD ? FORWARD : BACKWARD
        irSave |= BACKLEFT;
        irSave &= ~BACKRIGHT;
        }
      else if (ir[5]){
        singleDrive(0, BACKWARD, SLOW_SPEED);//dir == FORWARD ? BACKWARD : FORWARDc
        irSave |= BACKRIGHT;
        irSave &= ~BACKLEFT;
        }
      else {
        dirDrive();
        Serial.println("Illegal state detected");
      }
    }
    else {
      if(irSave & BACKLEFT)
        singleDrive(0, FORWARD, MEH_SPEED);
      else if(irSave & BACKRIGHT)
        singleDrive(0, BACKWARD, MEH_SPEED);
      else{
        dirDrive();
      }
      dirDrive(XDIR, dir, HALF_SPEED); //otherwise stop
    }
  }
  dirDrive(); //stop when done
}

void sonarDrive(int axis, int dir) {
  dirDrive();
  updateSonar();
  while (distance[axis + (dir - 1) * 2] > 1) {
    if (distance[axis + (dir - 1) * 2] > 5)
      dirDrive(axis, dir, FULL_SPEED);
    else
      dirDrive(axis, dir, HALF_SPEED);
    updateSonar();
  }
  dirDrive(); //stop when done
}

void updateSonar() {
  distance = Location::updateSonar();
}

void updateLocation() {
  updateSonar();
  
  // TODO this code incorrectly assumes a certain orientation
  // because the robot turns, the coordinate space will become shifted, dislocating fixed points
  Location::myLoc.x = distance[0] - distance[2];
  Location::myLoc.y = distance[1] - distance[3];
}

uint8_t moveTo(locationCoordinates toLoc) {
  updateLocation();
  dirDrive(); //stop when done
  
  while (XCLOSE() || YCLOSE()) {
    if (XCLOSE())
      dirDrive(XDIR, Location::myLoc.x > toLoc.x);
    else
      dirDrive(XDIR);
      
    if (YCLOSE())
      dirDrive(YDIR, Location::myLoc.y > toLoc.y);
    else
      dirDrive(YDIR);
      
    updateLocation();
  }
  dirDrive(); //stop when done
  
  return 0;
}
