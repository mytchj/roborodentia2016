#include "HMotor.h"
#include "Location.h"
#include "config.h"
#include "Adafruit_BNO055.h"

// shrink tolerance window of gyro from +-2 to +-1 to perfect drive through walls

HMotor hmotor1(22, 23, 2);
HMotor hmotor2(50, 51, 4);
HMotor hmotor3(52, 53, 5);
HMotor hmotor4(24, 25, 3);
HMotor *hm[MOTORCOUNT] = {&hmotor1, &hmotor2, &hmotor3, &hmotor4};
Adafruit_BNO055 bno = Adafruit_BNO055();

using namespace imu;
int mode = START;
int start_pos = 0;  // start_pos must be positive always, never negative or it will freak out
int tilt = 0;
boolean umph = false;

void setup() {
#if DEBUG_ENABLED
  Serial.begin(115200);
  Serial.println("Initializing");
#endif

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  pinMode(JOYBUTTON, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
  Location::Init();
  start_pos = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();

  #if DEBUG_ENABLED
    Serial.println("Initialized");
  #endif
}

void test() {
  drive(100, 0, 0, 0);
  delay(3000);
  drive(0, 100, 0, 0);
  delay(3000);
  drive(0, 0, 100, 0);
  delay(3000);
  drive(0, 0, 0, 100);
  delay(3000);  
}

void test2(){
  int isLift, acceleration;
  Serial.println("Ready?");
  delay(3000);

  Serial.println("GO!");
  for(int i = 0; i < 5000; i++){
    

      

    //Serial.println(isLifted());
    //delay(100);
  }
  
}

void loop() {
  startToMid();
  midToGap1();
  gap1ToPickup();
  pickupToGap2();
  gap2ToScore();
  score();
}

void startToMid() {
  Location::resetEncoders();

  // move to mid wall
  while(Location::getEncodery() < 6500) {
    irMode(-250);
  }
  // stop when at wall
  brake();
  delay(1500);    // TODO: Flip Multiplier Here
}

void midToGap1() {
  int isLift;
  Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // move away from wall and line up with gap1
  encoderModeY(FORWARD, 550);

  // change rotation by 90 degrees to face scoring pegs
  start_pos += 270;
  for (int i = 0; i < 1250; i++)
    gyroMode(0, 0);  

  tilt = euler.y();
  
  // move from mid to wall (through gap)
  Location::resetEncoders();
  while(Location::getEncodery() < 9000) {
    isLift = isLifted();
    if(abs(isLift) > 0){
      gyroMode(isLifted(), isLifted());
      umph = true;
    }
    else
      gyroMode(0, 150); 
    
    Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 

    int accelY = abs(acceleration.y());
    if(accelY > 5){
      Serial.println(accelY);    
      if(Location::getEncodery() > 8000)   // TODO: inegrate this with above while loop instead of break?
        break;
      else{
        umph = true;
        for(int i = 0; i < 300; i++)
          gyroMode(120, 120);
      }
    }
  }
  
  brake();
  if(umph){
    for(int i = 0; i < 700; i++)
      gyroMode(0, 150);
  }
  brake();
}

void gap1ToPickup() {
 /*
  * Backup to get space to drop
  */
  Location::resetEncoders();
  while(Location::getEncodery() < 300){
    gyroMode(-120, -100);
  }
  
 /*
  * Drive to the scoring pegs
  */
  encoderModeX(BACKWARD, 2100);
  brake();                                                // straightening out here???!!
  for (int i = 0; i < 2300; i++) {
    irMode(0, BACKWARD, BACKWARD);
  }
  brake();
  
  //reset things
  umph = false;
}

void pickupToGap2() {
 /*
  * Move Left to the gap
  */
  encoderModeX(BACKWARD, 1900);               //G2  
}

void gap2ToScore() {
  int isLift;
  Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  tilt = euler.y();

/*
* Move back to start
*/
  // move from wall to mid (through gap)
  Location::resetEncoders();
  while(Location::getEncodery() < 8400) {
    isLift = isLifted();
    Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 
    int accelY = abs(acceleration.y());
    if(abs(isLift) > 0){
      gyroMode(isLifted(), isLifted());
      umph = true;
    }
    else if(accelY > 5){ 
        umph = true;
        for(int i = 0; i < 300; i++)
          gyroMode(-120, -120);
    }
    else{
      gyroMode(0, -120);
    }
  }
  brake();    
  
  if(umph){
    for(int i = 0; i < 200; i++)
      gyroMode(0, -120);
  }  
}

void score() {
 /* 
  * change rotation by 90 degrees to face scoring pegs
  */
  start_pos += 90;
  for (int i = 0; i < 1200; i++)
    gyroMode(0, 0);
  
  for (int i = 0; i < 1500; i++) {
    irMode(0, BACKWARD, BACKWARD);
  }
  brake();
  
 /*
  * Pickup Rings
  */
  encoderModeY(FORWARD, 1000);
  delay(2000);  
}

void serialDo() {
  switch (Serial.read()) {
    case 'c': Location::printInfrared(); break;
    case 'v': Location::printRawInfrared(); break;
    
   // case '9': dirDrive(); break;
  }
}

void encoderModeX(int direction, int distance) {
    Location::resetEncoders();
    if (direction == FORWARD)
      drive(0, ENCODER_SPEED, 0, ENCODER_SPEED);
    else
      drive(0, -ENCODER_SPEED, 0, -ENCODER_SPEED);
    while(Location::getEncoderx() < distance) {
      Location::printEncoderCount();
    }
    brake();
    Serial.print("ENC = ");
    Location::printEncoderCount();
    delay(100);
}

void encoderModeY(int direction, int distance) {
    Location::resetEncoders();
    if (direction == FORWARD)
      drive(ENCODER_SPEED, 0, ENCODER_SPEED, 0);
    else
      drive(-ENCODER_SPEED, 0, -ENCODER_SPEED, 0);
    while(Location::getEncodery() < distance) {
      Location::printEncoderCount();
    }
    brake();
    Serial.print("ENC = ");
    Location::printEncoderCount();
    delay(100);
}

void gyroMode(int x, int y) {
  Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int rotation = abs(start_pos + euler.x());
  rotation = rotation % 360;
  
  if(rotation > 180 && rotation < 358){
    rotation = map(rotation, 180, 360, 130, 60);
    drive(y + rotation, x + rotation, y - rotation, x - rotation);
  }
  else if(rotation > 2 && rotation < 180){
    rotation = map(rotation, 0, 180, 60, 130);
    drive(y - rotation, x - rotation, y + rotation, x + rotation);
  }
  else {
    rotation = 0;
    drive(y, x, y, x);
  }
}

void irMode(int y) {
  irMode(y, 0, 0);
}


void irMode(int y, int seekF, int seekB) {
  int xF = 0, xB = 0;
  boolean* ir = Location::updateInfrared();

  // Front IR's with xF

  if(ir[0] == true)
    xF = -IR_SPEED;
  else if(ir[2] == true)
    xF = IR_SPEED;
  else{
    if(seekF == FORWARD)
      xF = IR_SPEED;
    else if(seekF == BACKWARD)
      xF = -IR_SPEED;
    else
      xF = 0;

  }

  // Back IR's wth xB
  if(ir[3] == true)
    xB = -IR_SPEED;
  else if(ir[5] == true)
    xB = IR_SPEED;
  else
    if(seekB == FORWARD)
      xB = IR_SPEED;
    else if(seekB == BACKWARD)
      xB = -IR_SPEED;
    else
      xB = 0;

  //If a double black is read then ignore it and make the motors move the same
  if(ir[3] == ir[4] && ir[4] == ir[5] && ir[4] == true){
    xB = xF;
  }
  if(ir[0] == ir[1] && ir[1] == ir[2] && ir[1] == true){
    xF = xB;
  }

  if (y == 0 && (xF == 0 ^ xB == 0)) // stationary needs more power to make it move if only one is off
    drive(0, xF * 2, 0, xB * 2);
  else // moving 
    drive(y, xF, y, xB);
}

void drive(int m1, int m2, int m3, int m4){
  hm[0]->drive(m1, m1 > 0 ? FORWARD : BACKWARD);
  hm[1]->drive(m2, m2 > 0 ? FORWARD : BACKWARD);
  hm[2]->drive(m3, m3 > 0 ? FORWARD : BACKWARD);
  hm[3]->drive(m4, m4 > 0 ? FORWARD : BACKWARD);
  
}

int isLifted(){
  Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  if(euler.y() - tilt < -1.5)
    return -130;
  else if(euler.y() - tilt > 1.5)
    return 130;
  else
    return 0;
}

void brake(){
  hm[0]->drive(0, BRAKE);
  hm[1]->drive(0, BRAKE);
  hm[2]->drive(0, BRAKE);
  hm[3]->drive(0, BRAKE);
}
