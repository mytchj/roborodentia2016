#include "HMotor.h"
#include "Location.h"
#include "config.h"
#include "Adafruit_BNO055.h"

HMotor hmotor1(22, 23, 2);
HMotor hmotor2(50, 51, 4);
HMotor hmotor3(52, 53, 5);
HMotor hmotor4(24, 25, 3);
HMotor *hm[MOTORCOUNT] = {&hmotor1, &hmotor2, &hmotor3, &hmotor4};
Adafruit_BNO055 bno = Adafruit_BNO055();

using namespace imu;
int mode = START;
int start_pos = 0;  // start_pos must be positive always, never negative or it will freak out

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

void loop() {
  //Location::printInfrared();
  //irMode(120);
  justDoIt();
  //test();
  //delay(500);
}


void justDoIt() {
  //6000 is mid to left Y
  //1000 is start to gap Y
    Location::resetEncoders();

    // move to mid wall
    while(Location::getEncodery() < 6500) {
      irMode(-250);
    }
    // stop when at wall
    brake();

    // DO STUFF
    delay(1500);

    // move away from wall and line up with gap
    encoderModeY(FORWARD, 742);

    // change rotation by 90 degrees to face scoring pegs
    start_pos += 270;
    for (int i = 0; i < 1200; i++)
      gyroMode(0, 0);

    // move from mid to wall (through gap)
    Location::resetEncoders();
    while(Location::getEncodery() < 9100) {
      gyroMode(0, 120);
    }
    brake();

    // move closer to middle
    encoderModeX(BACKWARD, 2300);
    brake();
    for (int i = 0; i < 3200; i++) {
      irMode(0, BACKWARD, BACKWARD);
    }
    brake();

    // thats all folks.
    digitalWrite(LEDPIN, HIGH);
    delay(3000);

    // move closer to middle
    encoderModeX(BACKWARD, 2300);
    
    // move from mid to wall (through gap)
    Location::resetEncoders();
    while(Location::getEncodery() < 8400) {
      gyroMode(0, -120);
    }
    brake();
    
    // change rotation by 90 degrees to face scoring pegs
    start_pos += 90;
    for (int i = 0; i < 1000; i++)
      gyroMode(0, 0);

    for (int i = 0; i < 2000; i++) {
      irMode(0, BACKWARD, BACKWARD);
    }
    brake();

    encoderModeY(FORWARD, 1000);
    
    // thats all folks.
    digitalWrite(LEDPIN, LOW);
    delay(3000);
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

  if(digitalRead(JOYBUTTON) == 0){ // joystick button pressed down
    Serial.print("Changing Mode to: ");
    mode = (mode + 1) % 4;
    switch((mode)){
      case START:
        digitalWrite(LEDPIN, LOW);
        Serial.println("Start State");
      break;
      case IR:
        digitalWrite(LEDPIN, LOW);
        Serial.println("IR Reading");
      break;
      case GYRO:
        digitalWrite(LEDPIN, LOW);
        Serial.println("Gyroscope");
      break;
      case ENCOD:
        digitalWrite(LEDPIN, LOW);
        Serial.println("Encoder");
        Location::resetEncoders();
      break;
      break;
      default:
        digitalWrite(LEDPIN, HIGH);
        Serial.println("No State Found");
      break;
    }
  }

  //joystick deadzone
  if(abs(x) < 100)
    x = 0;
  if (abs(y) < 100)
    y = 0;
    
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
    irMode(y);
  }
  
  /*
   * For Rotation of using the Gyroscope
   */
  else if(mode == GYRO) { 
    gyroMode(x, y);
  }

  /*
   * For Walking a precise number of steps using motor encoders
   */
  else if (mode == ENCOD) {
   
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

  Serial.print("eu = ");
  Serial.print(euler.x());
  
  Serial.print("  \tROT = ");
  Serial.print(rotation);
  
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
    
  Serial.print("  \tCOR = ");
  Serial.println(rotation);
}

void irMode(int y) {
  irMode(y, 0, 0);
}


void irMode(int y, int seekF, int seekB) {
  int xF = 0, xB = 0;
  boolean* ir = Location::updateInfrared();

  // Front IR's with xF
  if(ir[0] == ir[1] && ir[1] == ir[2] && ir[1] == true){
    xF = 0;
  }
  else if(ir[0] == true)
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

  // Front IR's with xF
  if(ir[3] == ir[4] && ir[4] == ir[5] && ir[4] == true){
    xF = 0;
  }
  else if(ir[3] == true)
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

void brake(){
  hm[0]->drive(0, BRAKE);
  hm[1]->drive(0, BRAKE);
  hm[2]->drive(0, BRAKE);
  hm[3]->drive(0, BRAKE);
}
