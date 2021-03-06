#include <Servo.h> 
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
Servo grabber[3];
Servo dumper[2];
//Servo flapper;
using namespace imu;
int start_pos = 0;  // start_pos must be positive always, never negative or it will freak out
double tilt = 0;
boolean umph = false;

void setup() {
  #if DEBUG_ENABLED
  Serial.begin(115200);
  Serial.println("Initializing");
  #endif

  grabber[LEFT].attach(9);
  grabber[MIDDLE].attach(10);
  grabber[RIGHT].attach(8);
  dumper[LEFT].attach(11);
  dumper[RIGHT].attach(12);
  //flapper.attach(13);
  // 3000 is down, 0 is up
  grabber[LEFT].writeMicroseconds(3000);
  grabber[MIDDLE].writeMicroseconds(3000);
  grabber[RIGHT].writeMicroseconds(3000);
  dumper[LEFT].writeMicroseconds(3000);
  dumper[RIGHT].writeMicroseconds(3000);
  //flapper.writeMicroseconds(3000);
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  Location::Init();
  start_pos = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();

  #if DEBUG_ENABLED
  Serial.println("Initialized");
  #endif
}



void loop() {
  getRings();
  
  while(Location::getEncodery() < 1500) irMode(-150);
  start_pos += 90;
    for (int i = 0; i < 1350; i++)
      gyroMode(0, 0);  

  while(!irMode(0, BACKWARD, BACKWARD));
  brake();

  boolean* ir;
  do{
    irMode(255);
    ir = Location::updateInfrared();
  }while(!(ir[0] & ir[2]));
  brake();

  //Location::resetEncoders();
 // while(Location::getEncodery() < 100) irMode(120);

  pickup(&grabber[RIGHT], 200);
  brake();
  for(int i = 0; i < 200; i++){
    irMode(0);
  }
  deposit();
  
  delay(1000);

  do{
    irMode(-255);
    ir = Location::updateInfrared();
  }while(!(ir[3] & ir[5]));
  brake();
  
  Location::resetEncoders();
  while(Location::getEncodery() < 1500) irMode(-150);
  brake();

    
  while(Location::getEncodery() < 1500) irMode(-150);
  start_pos -= 90;
    for (int i = 0; i < 1350; i++)
      gyroMode(0, 0);  

  brake();
  while(!irMode(0, BACKWARD, BACKWARD));
  brake();

  dumper[LEFT].writeMicroseconds(880);
  dumper[RIGHT].writeMicroseconds(880);

  grabber[LEFT].writeMicroseconds(3000);
  grabber[MIDDLE].writeMicroseconds(3000);
  grabber[RIGHT].writeMicroseconds(3000);

  
  encoderModeX(BACKWARD, 40);
  do{
    irMode(110);
    ir = Location::updateInfrared();
  }while(!(ir[0] & ir[2]));
  brake();

  Location::resetEncoders();
  while(Location::getEncodery() < 100) irMode(-120);
  


  /*
  startToMid();
  midToGap1();
  gap1ToScore();
  deposit();
  scoreToGap2();
  gap2ToPickup();
  */
  //delay(3000);
}

void getRings() {
  encoderModeX(BACKWARD, 40);
  Location::resetEncoders();
  while(Location::getEncodery() < 200) irMode(-120);
  //encoderModeY(BACKWARD, 300);
  brake();
  dumper[LEFT].writeMicroseconds(880);
  dumper[RIGHT].writeMicroseconds(880);
  waitForPickup();
  waitForPickup();
 /*
  * Pickup Rings
  */
  encoderModeY(FORWARD, 400);
  pickup(&grabber[MIDDLE]);
  waitForPickup();
  encoderModeY(BACKWARD, 500);
  brake();

  dumper[LEFT].writeMicroseconds(0);  // 0 is up
  dumper[RIGHT].writeMicroseconds(0);  // 0 is up
  delay(1000);
  dumper[LEFT].writeMicroseconds(880);
  dumper[RIGHT].writeMicroseconds(880);
  
  delay(1000);
  encoderModeX(FORWARD, 400);
  brake();    
  for (int i = 0; i < 400; i++)
    gyroMode(0, 0);
  encoderModeY(FORWARD, 500);
  pickup(&grabber[RIGHT], 400);
  waitForPickup();
  encoderModeY(BACKWARD, 500);

  dumper[LEFT].writeMicroseconds(0);  // 0 is up
  dumper[RIGHT].writeMicroseconds(0);  // 0 is up
  delay(1000);
  dumper[LEFT].writeMicroseconds(880);
  dumper[RIGHT].writeMicroseconds(880);
  
  encoderModeX(BACKWARD, 1100);
  brake();
  for (int i = 0; i < 400; i++)
    gyroMode(0, 0);
  encoderModeY(FORWARD, 500);
  pickup(&grabber[LEFT]);
  waitForPickup();
  encoderModeY(BACKWARD, 500);
  
  dumper[LEFT].writeMicroseconds(0);  // 0 is up
  dumper[RIGHT].writeMicroseconds(0);  // 0 is up
  
  delay(1000);
 
  for (int i = 0; i < 1500; i++) {
    irMode(0, FORWARD, FORWARD);
  }  
}


void startToMid() {
  Location::resetEncoders();
  
  // move to mid wall
  while(Location::getEncodery() < 500)  irMode(-70);
  while(Location::getEncodery() < 1500) irMode(-150);
  while(Location::getEncodery() < 3200) irMode(-250);
  while(Location::getEncodery() < 6000) irMode(-80);
  // stop when at wall
  brake();
}

void midToGap1() {
  int isLift;
  // move away from wall and line up with gap1
  encoderModeY(FORWARD, 175);

  // change rotation by 90 degrees to face scoring pegs
  start_pos += 270;
  for (int i = 0; i < 1350; i++)
    gyroMode(0, 0);  

  //flapper.writeMicroseconds(3000);
  encoderModeX(BACKWARD, 175);

  tilt = bno.getVector(Adafruit_BNO055::VECTOR_EULER).y();
  
  // move from mid to wall (through gap)
  Location::resetEncoders();
  while(Location::getEncodery() < 9000) {
    isLift = isLifted();
    if(abs(isLift) > 0) {
      gyroMode(isLift, isLift);
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
        for(int i = 0; i < 500; i++)
          gyroMode(120, 120);
      }
    }
  }
  
  brake();
  if(umph){
    for(int i = 0; i < 300; i++)
      gyroMode(-120, 150);
    //reset things for next run
    umph = false;
  }
  brake();
}

void gap1ToScore() {
 /*
  * Drive to the scoring pegs
  */
  encoderModeX(BACKWARD, 800);
  for (int i = 0; i < 2500 && !irMode(0, BACKWARD, BACKWARD); i++);
  encoderModeX(FORWARD, 50);
  brake();  
}

void deposit() {
  encoderModeY(BACKWARD, 330);
  brake();
  waitForPickup();
  dumper[LEFT].writeMicroseconds(1130);  // 0 is up
  dumper[RIGHT].writeMicroseconds(1130);  // 0 is up
  waitForPickup();
  
  encoderModeY(FORWARD, 30);
  encoderModeY(BACKWARD, 30);
  encoderModeY(FORWARD, 30);
  encoderModeY(BACKWARD, 30);
  encoderModeY(FORWARD, 30);
  encoderModeY(BACKWARD, 30);
/*
  for(int i = 0; i < 400; i++)
    gyroMode(0, 0);
  encoderModeY(BACKWARD, 300);
*/
}

void scoreToGap2() {
 /*
  * Move Left to the gap
  */
  encoderModeX(BACKWARD, 1900);               //G2
  //grabber[LEFT].writeMicroseconds(3000);
  //grabber[MIDDLE].writeMicroseconds(3000);
  //grabber[RIGHT].writeMicroseconds(3000);
  //dumper[LEFT].writeMicroseconds(3000);  // 0 is up
  //dumper[RIGHT].writeMicroseconds(3000);  // 0 is up
}

void gap2ToPickup() {
  int isLift;
  Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  tilt = euler.y();

/*
* Move back to start
*/
  // move from wall to mid (through gap)
  Location::resetEncoders();
  while(Location::getEncodery() < 6000) {
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
    //reset things for next run
    umph = false;
  }  

 /* 
  * change rotation by 90 degrees to face supply pegs
  */
  start_pos += 90;
  for (int i = 0; i < 1200; i++)
    gyroMode(0, 0);

  for (int i = 0; i < 400 && !irMode(-100, BACKWARD, BACKWARD); i++);
  for (int i = 0; i < 3000 && !irMode(0, BACKWARD, BACKWARD); i++);
  encoderModeX(FORWARD, 50);
  for (int i = 0; i < 3600; i++)
    gyroMode(0, 0);
  for (int i = 0; i < 2000 && !irMode(50); i++);
  brake();
  
}

void waitForPickup() {
  delay(1000);
}


void encoderModeX(int direction, int distance) {
    Location::resetEncoders();
    if (direction == FORWARD)
      drive(0, ENCODER_SPEED, 0, ENCODER_SPEED);
    else
      drive(0, -ENCODER_SPEED, 0, -ENCODER_SPEED);
    while(Location::getEncoderx() < distance) {
      //Location::printEncoderCount();
    }
    brake();
}

void encoderModeY(int direction, int distance) {
    Location::resetEncoders();
    if (direction == FORWARD)
      drive(ENCODER_SPEED, 0, ENCODER_SPEED, 0);
    else
      drive(-ENCODER_SPEED, 0, -ENCODER_SPEED, 0);
    while(Location::getEncodery() < distance) {
      //Location::printEncoderCount();
    }
    brake();
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

int irMode(int y) {
  return irMode(y, 0, 0);
}


int irMode(int y, int seekF, int seekB) {
  int xF = 0, xB = 0;
  boolean* ir = Location::updateInfrared();

  // Front IR's with xF
  if(ir[0] == true)
    xF = -IR_SPEED;
  else if(ir[2] == true)
    xF = IR_SPEED;
  else if(ir[1] == false) {
    if(seekF == FORWARD)
      xF = IR_SPEED;
    else if(seekF == BACKWARD)
      xF = -IR_SPEED;
  }

  // Back IR's wth xB
  if(ir[3] == true)
    xB = -IR_SPEED;
  else if(ir[5] == true)
    xB = IR_SPEED;
  else if(ir[4] == false) {
    if(seekB == FORWARD)
      xB = IR_SPEED;
    else if(seekB == BACKWARD)
      xB = -IR_SPEED;
  }

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

  return (ir[0] | ir[1] | ir[2]) & (ir[3] | ir[4] | ir[5]);
}

void drive(int m1, int m2, int m3, int m4){
  hm[0]->drive(m1, m1 > 0 ? FORWARD : BACKWARD);
  hm[1]->drive(m2, m2 > 0 ? FORWARD : BACKWARD);
  hm[2]->drive(m3, m3 > 0 ? FORWARD : BACKWARD);
  hm[3]->drive(m4, m4 > 0 ? FORWARD : BACKWARD);
}

int isLifted(){
  Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  if(euler.y() - tilt < -2)
    return -130;
  else if(euler.y() - tilt > 2)
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

void dropoff(Servo* servo) {
  servo->writeMicroseconds(2000); //3000 is down
  // Takes 500 ms to react
}

void pickup(Servo* servo) {
  servo->writeMicroseconds(900);  // 0 is up
  // Takes 500 ms to react
}

void pickup(Servo* servo, int add){
  servo->writeMicroseconds(900 - add);  // 0 is up
}

