/****************************************************************************
   Sonar Robot

   Authors:  
      - BENSOUSSAN Chloé
      - BOUKOU Grâce
      - GRÉAU Alexandre      
      - WILHELM Andreina
          
   Permissions: MIT licence
      
*****************************************************************************/

#include <Adafruit_MotorShield.h>
#include <RH_ASK.h>
#include <SPI.h>

#define FORWARD_ 0
#define BACKWARD_ 2
#define LEFT_ -1
#define RIGHT_ 1
#define STOP_ 3
#define MAX_SPEED 255


/* Ultrasonic sensors */      
const uint8_t echoPin_front_right = 2;  // echo signal (receives)
const uint8_t echoPin_front_left = 3; 

const uint8_t trigPin_front_right = 4;  // trigger signal (sends)
const uint8_t trigPin_front_left = 5; 


/* Communication */
RH_ASK driver(2000, 14, 15);
int msg;
int msgSize = sizeof(msg);


/* Measurements */
const float safetyDistance = 20; // according with the speed. Expressed in cm
const float robotWidth = 20;  // expressed in cm
const float margin = safetyDistance / 2; // margin of movement regarding the Master robot. The follower should always be between the marging and the safetyDistance


/* Movement */
boolean isDistanceSet = false;
volatile int currentState = STOP_; // initial state = stop
const int motorSpeed = 60; // from 0 (off) to 255 (max speed)

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Motor 1 -> left / Motor 2 -> right
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);


/*
 * Motors setup and movement
 */  
void setSpeed_(const int mSpeed) {
  motorLeft->setSpeed(mSpeed);
  motorRight->setSpeed(mSpeed);
}

void moveForward() {
  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);
}

void moveBackward() {
  motorLeft->run(BACKWARD);
  motorRight->run(BACKWARD);

  // turn on back led
  // turnOnLed(ledPin_back);
}

void moveLeft() {
  motorLeft->run(BACKWARD);
  motorRight->run(FORWARD);

  // turn on left led
  // turnOnLed(ledPin_left);
}

void moveRight() {
  motorLeft->run(FORWARD);
  motorRight->run(BACKWARD);

  // turn on right led
  // turnOnLed(ledPin_right);
}

void dontMove() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}


/*
 * Receiver setup
 */
void setupReceiver() {
  if (!driver.init())
         Serial.println("init failed");
}


/*
 * Calculates the distance with the information obtained from the sensors  
 */
float calculDistance(uint8_t trigPin, uint8_t echoPin){
  uint32_t duration; // duration of the round trip
  float cm;  // distance of the obstacle

  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  // Start trigger signal

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
 
  // convert the time into a distance
  cm = (float)((duration<<4)+duration)/1000.0; // cm = 17 * duration/1000
  return cm;
}


/*
 * Moves the robot
 */
void navigate() 
{
  float cm_front_left;  // distance to the Master
  float cm_front_right;  
  float max_distance = safetyDistance + margin;
  float ideal_distance = safetyDistance + margin/2;
  
  cm_front_left = calculDistance(trigPin_front_left, echoPin_front_left);
  cm_front_right = calculDistance(trigPin_front_right, echoPin_front_right);
  
  Serial.print(cm_front_left);
  Serial.print(" - ");
  Serial.print(cm_front_right);
  Serial.println("");

  if(cm_front_left < safetyDistance && cm_front_right < safetyDistance) {
    currentState = BACKWARD_;
    isDistanceSet = false;    
  } 
  else if (!isDistanceSet) {
    if(cm_front_left < ideal_distance && cm_front_right < ideal_distance) {
      currentState = BACKWARD_;
    } 
    else if ((cm_front_left > max_distance && cm_front_right > max_distance) ||
            ((ideal_distance - 2 < cm_front_left && cm_front_left < ideal_distance + 2) && 
            (ideal_distance - 2 < cm_front_right && cm_front_right < ideal_distance + 2))) {
      currentState = STOP_;
      isDistanceSet = true;  
    }
  }  

  switch(currentState) {
    // move forward  
    case FORWARD_:  
      Serial.println("avant");
      moveForward();
      break;

    // move backward
    case BACKWARD_:
      Serial.println("arriere");
      moveBackward();
      break;

    // move left
    case LEFT_: 
      Serial.println("gauche"); 
      moveLeft();
      break;

    // move right
    case RIGHT_:
      Serial.println("droite");
      moveRight();
      break;

    case STOP_:
      Serial.println("stop");
      dontMove();
      break;
  }
}

 

/*
 * Initial setup
 */
void setup() {
  // initialize serial communication
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz

  setupReceiver();
  setSpeed_(motorSpeed);
}


/*
 * It's the the function that will be called at each tick time. It executes navigate and sends the message
 */
void loop()
{  
  if (driver.recv((uint8_t *)&msg, (uint8_t *)&msgSize)) // Non-blocking
    {
        Serial.print("I received: ");
        Serial.print(msg);
        Serial.println("");
        currentState = msg;
    }
    else{
      Serial.println("Rien!");
   }
  navigate();
}
