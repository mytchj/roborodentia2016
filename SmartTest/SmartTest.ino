#include <Math.h>
#include <AFMotor.h>
#include "HMotor.h"
#include "Location.h"
#include "config.h"

// Motor & Servo Globals
AF_DCMotor motor1(2);
AF_DCMotor motor2(3);
AF_DCMotor motor3(4);
AF_DCMotor *slide[SERVOCOUNT] = {&motor1, &motor2, &motor3};

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
  
  lift(0, RELEASE);
  lift(1, RELEASE);
  lift(2, RELEASE);

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

  /*
  if (millis() - startTime > 179000) // Stop 1 second shy of 3 minutes
    endMatch();
  */

#if DEBUG_ENABLED
  if (serialRead())
    serialDo();
#else
  game();
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

void endMatch() {
  #if DEBUG_ENABLED
    Serial.println("Alert: Match Over");
  #endif
  dirDrive();
  Serial1.write(0b0011111111);
  toggleLED();
  while (!digitalRead(BUTTONPIN)) {
    #if DEBUG_ENABLED
      Serial.println("Alert: Restarting");
    #endif
    digitalWrite(LEDPIN, LOW);
    startTime = millis();
  }
}

void lift(uint8_t selector, uint8_t pos) {
  slide[selector]->setSpeed(FULL_SPEED);
  slide[selector]->run(pos);
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

void turnDrive(uint8_t dir, bool ninety, int long ticks){
  int long enC = Location::getEncoder();
  if (dir == FRONTLEFT) {
    hm[0]->drive(200, BACKWARD);
    hm[1]->drive(MEH_SPEED, FORWARD);
    hm[2]->drive(FULL_SPEED, BACKWARD);
    hm[3]->drive(MEH_SPEED, BACKWARD);
    while(Location::getEncoder() - enC < ticks);
  }
  else if (dir == FRONTRIGHT) {
    hm[0]->drive(HALF_SPEED + 20, FORWARD);
    hm[1]->drive(HALF_SPEED + 10, BACKWARD);
    hm[2]->drive(HALF_SPEED + 20, FORWARD);
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

void liftAll(unsigned int time) {
  lift(0, FORWARD);
  lift(1, FORWARD);
  lift(2, FORWARD);
  
  delay(time);
  
  lift(0, RELEASE);
  lift(1, RELEASE);
  lift(2, RELEASE);
}

void get(uint8_t num) {
  Serial1.write(OPEN << (num * 2));
  delay(1200);
  lift(num, BACKWARD);  // move down
  delay(800);
  Serial1.write(CLOSE << (num * 2));
  delay(1200);
  lift(num, FORWARD);   // move up
  delay(1000);
  lift(num, RELEASE);   // Release
}

void put(uint8_t num) {
  lift(num, BACKWARD);  // move down
  delay(200);
  Serial1.write(OPEN << (num * 2));
  delay(250);
  lift(num, FORWARD);   // move up
  delay(1200);
  Serial1.write(STOP << (num * 2));
  lift(num, RELEASE);   // Release
}

void game() {
  startTime = millis();
  int i = 2;
  while(!digitalRead(BUTTONPIN));

  // Prepare
  Serial1.write(0b01111111);
  liftAll(1000);
  Serial1.write(0b01111111);

  // Go to center (first action)
  followLines(FORWARD);
  get(1);

  turnDrive(FRONTRIGHT);
  liftAll(100);
  followLines(FORWARD, GORIGHT);
  put(1);

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
  
    get(1);
      
    turnDrive(FRONTLEFT);
    followLines(FORWARD, GOLEFT, false, true);
    followLines(FORWARD);
  
    put(1);
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

  get(1);
  
  //try uturn
  turnDrive(FRONTRIGHT);
  delay(100);
  turnDrive(FRONTRIGHT);
  followLines(FORWARD, GORIGHT, false);  
  
  followLines(FORWARD, GOLEFT, false);
  followLines(FORWARD, GORIGHT);

  put(1);
}

#if DEBUG_ENABLED

void serialDo() {
  switch (incomingByte) {
    case 'n': game(); break;
    case 'c': Location::printInfrared(); break;
    case 'v': Location::printRawInfrared(); break;
    case '1': followLines(FORWARD, GOLEFT); break;
    case '2': Location::printEncoderCount(); break;
    case '3': dirDrive(YDIR, FORWARD); break;
    case '4': dirDrive(); break;
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

    Serial.println(
    ir = Location::updateInfrared();
    if(drive){
      speed = FULL_SPEED;
        lift(0, FORWARD);
        lift(1, FORWARD);
        lift(2, FORWARD);
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
        //singleDrive(2, BRAKE);
        //singleDrive(2, RELEASE);
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
