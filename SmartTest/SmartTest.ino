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
static uint8_t clampStatus[SERVOCOUNT] = {STOP, STOP, STOP};

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
}

// select state manually
void clamp(uint8_t selector, uint8_t state) {
  if (state == CLOSE) {
    cservoAttach(selector);
    cservo[selector].write(0);
    clampStatus[selector] = CLOSE;
  } else if (state == OPEN) {
    cservoAttach(selector);
    cservo[selector].write(69);
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

void turnDrive(uint8_t dir, bool ninety){
  int sonarDistance;
  int timeCap;
    if (dir == FRONTLEFT) { 
    hm[0]->drive(200, BACKWARD);
    hm[1]->drive(MEH_SPEED, FORWARD);
    hm[2]->drive(FULL_SPEED, BACKWARD);
    hm[3]->drive(MEH_SPEED, BACKWARD);
  }
  else if (dir == FRONTRIGHT) {
    hm[0]->drive(HALF_SPEED + 20, FORWARD);
    hm[1]->drive(HALF_SPEED, BACKWARD);
    hm[2]->drive(HALF_SPEED + 20, FORWARD);
    if(!ninety)
      delay(200);
  }
  if(!ninety)
    delay(1100);
  else{
    timeCap = millis();
    delay(500);
    do{
      updateSonar();
      sonarDistance = distance[0] + distance[2];
    }while(!(sonarDistance == 90) & (timeCap - millis()) < 600);
    singleDrive(0, BRAKE);
    singleDrive(1, BRAKE);
    singleDrive(2, BRAKE);
  }
  dirDrive();
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

void withdraw(uint8_t num) {
  // Get others out of the way
  lift(num, BACKWARD);
  lift((num + 1) % POTCOUNT, FORWARD);
  lift((num + 2) % POTCOUNT, FORWARD);
  delay(690);

  // Grab
  clamp(num, CLOSE);
  delay(690);
  lift(num, FORWARD);
  delay(690);

  // Release Everyone
  lift((num + 1) % POTCOUNT, RELEASE);
  lift((num + 2) % POTCOUNT, RELEASE);  
  lift(num, RELEASE);
}

void deposit(uint8_t num) {
  lift(num, BACKWARD);
  clamp(num, OPEN);
  delay(690);
  lift(num, FORWARD);
  delay(690 / 2);
  clamp(num, CLOSE);
  delay(690 / 2);
}

void game() {
  while(!digitalRead(BUTTONPIN));
    Serial.println("FIGHT!");

  turnDrive(FRONTRIGHT);
  delay(100);
  followLines(FORWARD, GORIGHT);

  delay(1000);

  turnDrive(FRONTRIGHT);
  delay(100);
  followLines(FORWARD, GORIGHT, false);
  delay(50);
  followLines(FORWARD, GOLEFT, false);
  delay(50);
  followLines(FORWARD, GORIGHT);

  delay(1000);

  turnDrive(FRONTLEFT);
  delay(100);
  followLines(FORWARD, GOLEFT, false);
  followLines(FORWARD, GORIGHT, false);
  followLines(FORWARD, GOLEFT);

  delay(1000);

  turnDrive(FRONTRIGHT);
  delay(100);
  followLines(FORWARD, GORIGHT, false);
  delay(50);
  followLines(FORWARD, GOLEFT, false);
  delay(50);
  followLines(FORWARD, GORIGHT);

  //try uturn
  turnDrive(FRONTRIGHT);
  delay(100);
  turnDrive(FRONTRIGHT);
  followLines(FORWARD, GORIGHT, false);
}

void laps(uint8_t numberOfLaps) {
  while (numberOfLaps--) {
    Serial.println("FollowForward");
    followLines(FORWARD, 0);
    followLines(FORWARD, 0);
    Serial.println("Withdraw");
    withdraw();
    
    Serial.println("FollowBackward");
    followLines(BACKWARD, 0);
    followLines(BACKWARD, 0);
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

    case 'm' : turnDrive(FRONTLEFT); break;
    case 'q': followLines(FORWARD, GORIGHT);                        break;
    case 'w': followLines(BACKWARD, GORIGHT);                        break;
    case 'e': followLines(FORWARD, GOLEFT, false); break;

    case 'i': turnDrive(FRONTRIGHT); break;
    case 'g': followLines(FORWARD, GORIGHT, false); break;
    
    
	case 'n': game(); break;  
    
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

void followLines(uint8_t dir) {
  followLines(dir, 0);
}

void followLines(uint8_t dir, byte irSave){
  followLines(dir, irSave, true);
}

// Runs the front and back
void followLines(uint8_t dir, byte irSave, bool drive) {
  int dist = SLOW_SPEED, speed;
  if(drive)
    dirDrive(XDIR, dir, HALF_SPEED);
  do{
    ir = Location::updateInfrared();
    if(drive){
      speed = (dist << 1) + 60;
      if(speed > FULL_SPEED)
        speed = FULL_SPEED;
      else if(dist < 50)
        speed = MEH_SPEED - 20;
    }
    else
      speed = 0;

      Serial.println(speed);
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
      else
        dirDrive(XDIR, dir, speed);
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
      else
        dirDrive(XDIR, dir, speed);
    }
    else {
      if(irSave & BACKLEFT)
        singleDrive(0, BACKWARD, MEH_SPEED + 25);
      else if(irSave & BACKRIGHT)
        singleDrive(0, FORWARD, MEH_SPEED + 25);
      if(drive)
        dirDrive(XDIR, dir, MEH_SPEED); //otherwise stop
    }
    if(drive)
      updateSonar();
    else
      if(ir[1] & ir[2])
        break;
  } while(!drive | ((dist = distance[dir == BACKWARD ? 0 : 2]) > 5 | dist == 0));
  //Serial.println(dist);
  if(drive)
    delay(50);
  dirDrive(); //stop when done
}

void shiftOver(uint8_t axis, uint8_t dir, uint8_t farther) {
  updateSonar();
  uint8_t where = distance[FIND_SONAR] - farther;
  while (distance[FIND_SONAR] != where) {
    stabilize(axis, dir, where);
    updateSonar();
  }
}

void stabilize(uint8_t axis, uint8_t dir, uint8_t where) {
  uint8_t fS = FIND_SONAR;
  updateSonar();
  while (distance[fS] != where) {
    Serial.print(distance[fS]);
    Serial.print("   ");
    Serial.println(where);
    if (distance[fS] > where)
      dirDrive(axis, dir, SLOW_SPEED);
    else
      dirDrive(axis, dir ^ 0x03, SLOW_SPEED);
    updateSonar();
  }
  dirDrive();
  Serial.print(distance[fS]);
}

void sonarDrive(uint8_t axis, uint8_t dir) {
  int x;
  dirDrive();
  updateSonar();


  while ((x = distance[FIND_SONAR]) > 3) {

    if (x > 25)
      dirDrive(axis, dir, FULL_SPEED);
    else
      dirDrive(axis, dir, SLOW_SPEED);
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
