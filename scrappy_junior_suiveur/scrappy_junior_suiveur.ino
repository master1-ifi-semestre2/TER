/****************************************************************************
   Sonar Robot

   Authors:  
      - BENSOUSSAN Chloé
      - BOUKOU Grâce
      - GRÉAU Alexandre      
      - WILHELM Andreina
          
   Permissions: MIT licence
      
*****************************************************************************/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
#include <TimerOne.h>
#include <VirtualWire.h>

#define FORWARD_ 0
#define BACKWARD_ 2
#define LEFT_ -1
#define RIGHT_ 1


/* Communication */
const int receive_pin = 17;               
const uint8_t myId = 1; // follower

typedef struct {
  int id;
  int value;
} Message;

Message msg;
byte msgSize = sizeof(msg);


/* Measurements */
const float safetyDistance = 27; // according with the speed, expressed in cm
const float robotWidth = 20; // and the height is 12 cm


/* Movement */
const int motorSpeed = 200;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Motor 1 -> right Motor 2 -> left
Adafruit_DCMotor *motorRight = AFMS.getMotor(1);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(2);


/*
 * Moves the wheels
 */
void navigate(){
  int moveDir = msg.value;

  // move forward  
  if(moveDir == FORWARD_) { 
    Serial.print("avant  ");
    motorRight->run(FORWARD);
    motorLeft->run(FORWARD);
  }
  // move backward
  else if(moveDir == BACKWARD_) {
    Serial.print("arriere  ");
    motorRight->run(BACKWARD);
    motorLeft->run(BACKWARD);
  }
  // move left
  else if(moveDir == LEFT_) { 
    Serial.print("gauche  ");
    motorRight->run(RELEASE);
    motorLeft->run(FORWARD);
  }
  // move right
  else if(moveDir == RIGHT_) {
    Serial.print("droite  ");
    motorRight->run(FORWARD);
    motorLeft->run(RELEASE);
  }
}

 

/*
 * Initial setup
 */
void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // Setup the receiver for communication
  vw_set_rx_pin(receive_pin);
  vw_setup(2000); 
  vw_rx_start(); 

  // Set initial message
  msg.id = myId;
  msg.value = FORWARD_;

  // Right wheel
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorRight->setSpeed(motorSpeed);
  motorRight->run(FORWARD);
  // turn on motor
  motorRight->run(RELEASE);
  
  // Left wheel
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorLeft->setSpeed(motorSpeed);
  motorLeft->run(FORWARD);
  // turn on motor
  motorLeft->run(RELEASE);
}


/*
 * It executes navigate and sends the message
 */
void loop()
{
  // check if a message has been received and stores it in the corresponding structure
  if (vw_get_message((byte *) &msg, &msgSize))// && formationMode) // Non-blocking
  {
    Serial.println(""); 
    Serial.print("Id: ");
    Serial.print(msg.id);
    Serial.print("  Value: ");
    Serial.println(msg.value); 
  }

  navigate();
}
