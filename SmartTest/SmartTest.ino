#include <Math.h>
#include <AFMotor.h>
#include "HMotor.h"
#include "Location.h"
#include "config.h"

// Motor & Servo Globals
AF_DCMotor slide(4);

static int startTime;
static boolean* ir; //[IRCOUNT];

#if DEBUG_ENABLED
static char incomingByte;
#endif 

HMotor hmotor1(39, 38, 2);
HMotor hmotor2(40, 41, 45);
HMotor hmotor3(51, 50, 46);
HMotor hmotor4(49, 48, 44);
HMotor *hm[MOTORCOUNT] = {&hmotor1, &hmotor2, &hmotor3, &hmotor4};

void setup() {
#if DEBUG_ENABLED
  Serial.begin(115200);
  Serial.println("Initializing");
#endif

  Serial1.begin(115200); // talk to other Arduino

  pinMode(BUTTONPIN, INPUT);
  pinMode(LBUTTON, INPUT_PULLUP);
  pinMode(RBUTTON, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
  Location::Init();

  // Wait for start
  //while (!digitalRead(BUTTONPIN));
  
  lift(RELEASE);

#if DEBUG_ENABLED
  Serial.println("Initialized");
#endif
}

void loop() {
  game();
  if (digitalRead(BUTTONPIN) && millis() - startTime > 10000) {    
    #if DEBUG_ENABLED
      Serial.println("Restarting");
    #endif
    startTime = millis();
  }

#if DEBUG_ENABLED
  if (serialRead())
    serialDo();
#endif    
}

int checkButton()  {
  int button = 0;
  if(!digitalRead(LBUTTON))
    button |= 1;
  if(!digitalRead(RBUTTON))
    button |= 2;
  return button; 
}

void lift(uint8_t pos) {
  slide.setSpeed(FULL_SPEED);
  slide.run(pos);
}

void spinDrive(int degrees, bool direction) {
  hm[0]->drive(FULL_SPEED, direction ? FORWARD : BACKWARD);                    
  hm[1]->drive(FULL_SPEED, direction ? FORWARD : BACKWARD);
  hm[2]->drive(FULL_SPEED, direction ? BACKWARD : FORWARD);
  hm[3]->drive(FULL_SPEED, direction ? BACKWARD : FORWARD);
  
  delay(degrees);
  for (int i = 0; i < MOTORCOUNT; i++)
    hm[i]->drive(0, RELEASE);
}

void singleDrive(uint8_t num, uint8_t direction) {
  hm[num]->drive(FULL_SPEED, direction);
}

void singleDrive(uint8_t num, uint8_t dir, uint8_t speed) {
  hm[num]->drive(speed, dir);
}

void dirDrive(bool axis, uint8_t dir, uint8_t speed) {
  hm[axis ? 0 : 1]->drive(speed, dir);
  hm[axis ? 2 : 3]->drive(speed, dir);
}

void dirDrive(bool axis, uint8_t dir) {
  dirDrive(axis, dir, FULL_SPEED);
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

void turnDrive(uint8_t dir) {
  turnDrive(dir, false);
}

void turnDrive(uint8_t dir, bool ninety){
  turnDrive(dir, ninety, 5000);
}

void turnDrive(uint8_t dir, bool ninety, unsigned int long ticks){
  Location::resetEncoder();
  int long enC = Location::getEncoder();
  if (dir == FRONTLEFT) {
    hm[0]->drive(200, BACKWARD);
    hm[1]->drive(MEH_SPEED, FORWARD);
    hm[2]->drive(FULL_SPEED, BACKWARD);
    hm[3]->drive(MEH_SPEED, BACKWARD);
    while(Location::getEncoder() - enC < ticks);
  }
  else if (dir == FRONTRIGHT) {
    hm[0]->drive(FULL_SPEED - 25, FORWARD);
    hm[1]->drive(FULL_SPEED, BACKWARD);
    hm[2]->drive(FULL_SPEED - 25, FORWARD);
    while(Location::getEncoder() - enC < ticks);
  }
  
  if(ninety)
    followLines(FORWARD, GORIGHT);
  dirDrive();
}

#if DEBUG_ENABLED
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

void liftAll(unsigned int time) {
  lift(FORWARD);
  delay(time);
  lift(RELEASE);
}

void get() {
  Serial1.write(OPEN << 2);
  delay(1200);
  lift(BACKWARD);  // move down
  delay(1200);
  Serial1.write(CLOSE << 2);
  delay(1200);
  lift(FORWARD);   // move up
  delay(1000);
  lift(RELEASE);   // Release
}

void put() {
  lift(BACKWARD);  // move down
  delay(400);
  Serial1.write(OPEN << 2);
  delay(100);
  lift(FORWARD);   // move up
  delay(1200);
  Serial1.write(STOP << 2);
  lift(RELEASE);   // Release
}

void game() {
  startTime = millis();
  int i = 5;
  while(!digitalRead(BUTTONPIN));

  // Prepare
  Serial1.write(0b01111111);

  // Go to center (first action)
  followLines(FORWARD);
  get();
  
  turnDrive(FRONTRIGHT, false, 5000);
  liftAll(100);
  followLines(FORWARD, GORIGHT);
  put();
  
  while(i--) {
    turnDrive(FRONTRIGHT, true, 4500);
    followLines(FORWARD, GORIGHT, false);
  
    liftAll(100);
    dirDrive(YDIR, FORWARD, HALF_SPEED);
  
    liftAll(150);
    followLines(FORWARD, GORIGHT, false, true);
    liftAll(100);
    followLines(FORWARD, GOLEFT, false, true);
    followLines(FORWARD);
  
    get();
      
    turnDrive(FRONTLEFT);
    followLines(FORWARD, GOLEFT, false, true);
    followLines(FORWARD);
  
    put();
  }

  turnDrive(FRONTRIGHT, true);
  followLines(FORWARD, GORIGHT, false);
  dirDrive(YDIR, FORWARD, HALF_SPEED);
  delay(200);
  followLines(FORWARD, GORIGHT, false, true);
  delay(200);

  followLines(FORWARD, GOLEFT, false, true);
  delay(100);
  followLines(FORWARD, GORIGHT, false, true);
  delay(100);
  followLines(FORWARD, GOLEFT, false, true);
  followLines(FORWARD);

  get();
  
  // try uturn
  turnDrive(FRONTRIGHT);
  liftAll(300);
  turnDrive(FRONTRIGHT, true);
  followLines(FORWARD, GORIGHT, false);  
  followLines(FORWARD, GOLEFT, false);
  followLines(FORWARD, GORIGHT);

  put();
}

#if DEBUG_ENABLED

void serialDo() {
  switch (incomingByte) {
    case 'n': game(); break;
  }
}

#endif

void followLines(uint8_t dir) {
  followLines(dir, 0);
}

void followLines(uint8_t dir, byte irSave){
  followLines(dir, irSave, true);
}

void followLines(uint8_t dir, byte irSave, bool drive){
  followLines(dir, irSave, drive, false);
}

// Runs the front and back
void followLines(uint8_t dir, byte irSave, bool drive, bool checkBack) {
  int speed;
  if(drive)
    dirDrive(XDIR, dir, HALF_SPEED);
 
  do{
    ir = Location::updateInfrared();
    if(drive){
      speed = FULL_SPEED;
        lift(FORWARD);
    }
    else
      speed = 0;

    //THIS CHECKS THE FRONT IR's
    if (ir[0] | ir[1] | ir[2]) { // Any of the front are on
      if (!ir[2]){
        singleDrive(2, BACKWARD , SLOW_SPEED);
        irSave |= FRONTLEFT;
        irSave &= ~FRONTRIGHT;
      }
      else if (!ir[0]){
        singleDrive(2, FORWARD, SLOW_SPEED);
        irSave |= FRONTRIGHT;
        irSave &= ~FRONTLEFT;
      }
      else{
        dirDrive(XDIR, dir, speed);
      }
    }
    else{
      if(irSave & FRONTLEFT)
        singleDrive(2, BACKWARD, MEH_SPEED);
      else if(irSave & FRONTRIGHT)
        singleDrive(2, FORWARD, MEH_SPEED);
      if(drive)
        dirDrive(XDIR, dir, MEH_SPEED);
    }

    //THIS CHECKS THE BACK IR's
    if (ir[3] | ir[4] | ir[5]) { // Any of the back are on
      if (!ir[5]){
        singleDrive(0, BACKWARD, SLOW_SPEED);
        irSave |= BACKLEFT;
        irSave &= ~BACKRIGHT;
      } 
      else if (!ir[3]){
        singleDrive(0, FORWARD, SLOW_SPEED);
        irSave |= BACKRIGHT;
        irSave &= ~BACKLEFT;
      } 
      else{
        dirDrive(XDIR, dir, speed);
      }
    }
    else {
      if(irSave & BACKLEFT)
        singleDrive(0, BACKWARD, MEH_SPEED + 25);
      else if(irSave & BACKRIGHT)
        singleDrive(0, FORWARD, MEH_SPEED + 25);
      if(drive)
        dirDrive(XDIR, dir, MEH_SPEED); //otherwise stop
    }
    if (!drive && ir[1]) {
      if(!checkBack)
        break;
      else if(checkBack & ir[4])
        break;
    }
  } while(!drive | !checkButton());

  if(drive){
    dirDrive(XDIR, dir, MEH_SPEED);
    liftAll(100);
  }

  dirDrive(); //stop when done
}

void shiftOver(uint8_t dir, unsigned int long ticks) {
  Location::resetEncoder();
  int long enC = Location::getEncoder();

  dirDrive(YDIR, dir, SLOW_SPEED);
  while(Location::getEncoder() - enC < ticks);
  dirDrive();
}

