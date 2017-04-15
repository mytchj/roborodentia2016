#include "HMotor.h"
#include "Location.h"
#include "config.h"
#include "Adafruit_BNO055.h"

HMotor hmotor1(22, 23, 4);
HMotor hmotor2(52, 53, 3);
HMotor hmotor3(50, 51, 2);
HMotor hmotor4(25, 24, 5);
HMotor *hm[MOTORCOUNT] = {&hmotor1, &hmotor2, &hmotor3, &hmotor4};
Adafruit_BNO055 bno = Adafruit_BNO055();

using namespace imu;
int mode = 0;

int start_pos;

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
  Location::Init();
  start_pos = bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();

  #if DEBUG_ENABLED
    Serial.println("Initialized");
  #endif
}

void loop() {
  if(digitalRead(JOYBUTTON) == 0){ // joystick button pressed down
    Serial.print("Changing Mode to: ");
    mode = (mode + 1) % 4;
    switch((mode)){
      case START:
        Serial.println("Start State");
      break;
      case IR:
        Serial.println("IR Reading");
      break;
      case GYRO:
        Serial.println("Gyroscope");
      break;
      case ENCOD:
        Serial.println("Encoder");
        Location::resetEncoder();
      break;
      default:
        Serial.println("No State Found");
      break;
    }
    delay(1000);
  }
  joystickDo();
}

void serialDo() {
  switch (Serial.read()) {
    case 'c': Location::printInfrared(); break;
    case 'v': Location::printRawInfrared(); break;
    
   // case '9': dirDrive(); break;
  }
}

void joystickDo() {
  int x = analogRead(A0) - 512;
  int y = analogRead(A1) - 512;
  int xF = 0, xB = 0;

  //joystick deadzone
  if(abs(x) < 100)
    x = 0;
  if (abs(y) < 100)
    y = 0;
    
  Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /*
   * For simple driving
   */
  if(mode == START) {
      drive(y, x, y, x);
  }
  
  /*
   * For Driving with IR
   */
  else if(mode == IR){
    Location* l = new Location();
  
    // Front IR's with xF
    if(l->ir[0] == true)
      xF = -180;
    else if(l->ir[2] == true)
      xF = 180;
    else
      xF = 0;
  
    // Front IR's with xF
    if(l->ir[3] == true)
      xB = -180;
    else if(l->ir[5] == true)
      xB = 180;
    else
      xB = 0;

    l->printInfrared();
    drive(0, xF, 0, xB);
  }
  
  /*
   * For Rotation of using the Gyroscope
   */
  else if(mode == GYRO){ 
    int rotation = abs(start_pos - euler.x());
  
    if(rotation > 180 && rotation < 355){
      rotation = 360 - rotation;
      rotation = map(rotation, -180, 180, 80, 180);
      drive(y + rotation, x + rotation, y - rotation, x - rotation);
    }
    else if(rotation > 5 && rotation < 180){
      rotation = map(rotation, -180, 180, 80, 180);
      drive(y - rotation, x - rotation, y + rotation, x + rotation);
    }
    else
      drive(y, x, y, x);
    Serial.print("ROT = ");
    Serial.println(rotation);
  }

  /*
   * For Walking a precise number of steps using motor encoders
   */
  else if (mode == ENCOD) {
    if (x) {
      Location::resetEncoder();
      if (x > 0)
        drive(0, HALF_SPEED, 0, HALF_SPEED);
      if (x < 0)
        drive(0, -HALF_SPEED, 0, -HALF_SPEED);
      while(Location::getEncoder() < 1042)
        Location::printEncoderCount();
      drive(0, 0, 0, 0);
      Serial.print("ENC = ");
      Location::printEncoderCount();
      delay(400);
    }
  }
}

void drive(int m1, int m2, int m3, int m4){
  hm[0]->drive(m1, m1 > 0 ? FORWARD : BACKWARD);
  hm[1]->drive(m2, m2 > 0 ? FORWARD : BACKWARD);
  hm[2]->drive(m3, m3 > 0 ? FORWARD : BACKWARD);
  hm[3]->drive(m4, m4 > 0 ? FORWARD : BACKWARD);
  
}

void brake(){
  hm[0]->drive(0, BRAKE);
  hm[1]->drive(0, BRAKE);
  hm[2]->drive(0, BRAKE);
  hm[3]->drive(0, BRAKE);
}
/*
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

*/
