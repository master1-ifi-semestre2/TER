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


/* Ultrasonic sensors */      
const uint8_t echoPin_RIGHT = 2;  // echo signal (receives)
const uint8_t echoPin_right = 3;
const uint8_t echoPin_front_right = 4;
const uint8_t echoPin_front_left = 5; 
const uint8_t echoPin_left = 6;
const uint8_t echoPin_LEFT = 7; 

const uint8_t trigPin_RIGHT = 8;  // trigger signal (sends)
const uint8_t trigPin_right = 9;
const uint8_t trigPin_front_right = 10;
const uint8_t trigPin_front_left = 11; 
const uint8_t trigPin_left = 12;
const uint8_t trigPin_LEFT = 13;


/* Communication */
const int transmit_pin = 14;    
const uint8_t myId = 0; // boss

typedef struct {
  int id;
  int value;
} Message;

Message msg;


/* Measurements */
const float safetyDistance = 30; // according with the speed, expressed in cm
const float robotWidth = 20; // and the height is 12 cm
const float marge = safetyDistance / 3;


/* LEDs
 * long side : pin
 * short side : ground
 * resistor : 100 Ohm
 */
const uint8_t ledPin_left = 15;
const uint8_t ledPin_back = 16;
const uint8_t ledPin_right = 17;


/* Movement */
const int motorSpeed = 200; // from 0 (off) to 255 (max speed)


/*
 * Determines where to move
 */
int objectDetected = 0; // the side where the object is detected
boolean searchingObject = false;
int tick = 0;
int randomDir = FORWARD_;
boolean stop_ = false;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Motor 4 -> left / Motor 2 -> right
Adafruit_DCMotor *motorLeft = AFMS.getMotor(4);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);


/*
 * Motors setup and movement
 */ 
void setupMotors() {
  // Left wheel
  motorLeft->setSpeed(motorSpeed);
  motorLeft->run(FORWARD);
  // turn on motor
  motorLeft->run(RELEASE);
  
  // Right wheel
  motorRight->setSpeed(motorSpeed);
  motorRight->run(FORWARD);
  // turn on motor
  motorRight->run(RELEASE);
}

void moveForward() {
  motorRight->run(FORWARD);
  motorLeft->run(FORWARD);
  msg.value = FORWARD_; 
}

void moveBackward() {
  motorRight->run(BACKWARD);
  motorLeft->run(BACKWARD);

  // turn on back led
  // turnOnLed(ledPin_back);

  msg.value = BACKWARD_;
}

void moveLeft() {
  motorLeft->run(RELEASE);
  motorRight->run(FORWARD);

  // turn on left led
  // turnOnLed(ledPin_left);

  msg.value = LEFT_;
}

void moveRight() {
  motorLeft->run(FORWARD);
  motorRight->run(RELEASE);

  // turn on right led
  // turnOnLed(ledPin_right);

  msg.value = RIGHT_;
}

void dontMove() {
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
}

/*
 * Transmitter setup and send message
 */
void setupTransmitter() {
  // Setup the transmitter
  vw_set_tx_pin(transmit_pin);
  vw_set_ptt_inverted(true);
  vw_setup(2000);

  // Set initial message
  msg.id = myId;
  msg.value = FORWARD_;
}

/* Sends message on how to move */
void sendMessage() {
  vw_send((byte*) &msg, sizeof(msg));
  vw_wait_tx(); // we wait until the message is sent
  //Serial.println("Message send !");
  delay(10);
}


/*
 * LEDs setup
 */ 
void setupLeds() {
  pinMode(ledPin_left, OUTPUT);
  pinMode(ledPin_back, OUTPUT);
  pinMode(ledPin_right, OUTPUT);  
}

void turnOffAllLeds() { 
  digitalWrite(ledPin_left, LOW);
  digitalWrite(ledPin_back, LOW);
  digitalWrite(ledPin_right, LOW);
  delay(100);
}

void turnOnLed(uint8_t ledPin) {
  digitalWrite(ledPin, HIGH);  
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


void initValue() {
  objectDetected = 0;
  searchingObject = false;
  tick = 0;
}

/*
 * Determines where to move
 */
int explore(float cm_LEFT, float cm_left, float cm_front_left, float cm_front_right, float cm_right, float cm_RIGHT) {  
  Serial.print("object detected (-1 == left) ");
  Serial.println(objectDetected);
  Serial.print(cm_LEFT);
  Serial.print(" - ");
  Serial.print(cm_left);
  Serial.print(" - ");
  Serial.print(cm_front_left);
  Serial.print(" - ");
  Serial.print(cm_front_right);
  Serial.print(" - ");
  Serial.print(cm_right);
  Serial.print(" - ");
  Serial.println(cm_RIGHT);
   
 if (tick > 203){
  initValue();
  stop_ = false;
 }
 else if (tick == 200) {
      Serial.println("random");
      randomDir = -objectDetected;
      //initValue();
      stop_ = true;
      return randomDir;
  }
  
 if (searchingObject && objectDetected == LEFT_){
    // Turn until robot is parallel to the object
    if (cm_LEFT < safetyDistance ||  cm_left < safetyDistance){
         searchingObject = false;
    }
    else {
        // Serial.println(" Tourne à droite pour retrouver l'object ");
         return RIGHT_;
    }
 }
 else if (searchingObject && objectDetected == RIGHT_){
    // Turn until robot is parallel to the object
    if (cm_RIGHT < safetyDistance ||  cm_right < safetyDistance){
         searchingObject = false;
    }
    else {
        // Serial.println(" Tourne à gauche pour retrouver l'object ");
         return LEFT_;
    }
 }

 if (objectDetected == LEFT_) {
      // If there is already an object detected
      if(cm_LEFT > safetyDistance && cm_left > safetyDistance) {
        // If there is an object detected on the left and he has passed it, then he has to turn left
        return LEFT_;
      } 
      if (cm_LEFT < safetyDistance - marge ||  cm_left < safetyDistance - marge || cm_front_right < safetyDistance || cm_front_left < safetyDistance){
        return RIGHT_;
      } 
    return FORWARD_; 
  } 
  else if (objectDetected == RIGHT_) {
      // If there is already an object detected
      if(cm_RIGHT > safetyDistance && cm_right > safetyDistance) {
        // If there is an object detected on the left and he has passed it, then he has to turn right
        return RIGHT_;
      } 
      if (cm_RIGHT < safetyDistance - marge ||  cm_right < safetyDistance - marge || cm_front_right < safetyDistance || cm_front_left < safetyDistance){
          return LEFT_;
      } 
    return FORWARD_; 
  }
  else {
    // There's no object detected
    if (tick < 4) {
      return randomDir; 
    }
    
    if(cm_LEFT < safetyDistance || cm_left < safetyDistance) {
        // Object detected on the left side
         objectDetected = LEFT_;
     }
     else if(cm_RIGHT < safetyDistance || cm_right < safetyDistance) {
        // Object detected on the right side
         objectDetected = RIGHT_;
     }
     else if(cm_front_left < safetyDistance && cm_front_right < safetyDistance) {
        // Object detected in front of him
        
        if(cm_LEFT < safetyDistance || cm_left < safetyDistance) {
            // There is an object on the left side 
            objectDetected = LEFT_;
            return RIGHT_;
        }
        else if(cm_RIGHT < safetyDistance || cm_right < safetyDistance) {
            // There is an object on the right side
            objectDetected = RIGHT_;
            return LEFT_;
        }
        else {
            // Compare both sides
            if  (cm_right > cm_left) {
                objectDetected = LEFT_;
                searchingObject = true;
                return RIGHT_;
            }
            else {
              //Serial.println(" - RIGHT");
              objectDetected = RIGHT_;
              searchingObject = true;
              return LEFT_;
            }  
        }       
     }
     return FORWARD_; 
  }
} 



/*
 * Moves the wheels
 */
void navigate(){
  int resultatExplore;
  
  float cm_front_left;  // distance of the obstacle
  float cm_front_right;
  float cm_left;
  float cm_LEFT;
  float cm_right;
  float cm_RIGHT;

  
  noInterrupts();
  cm_front_left = calculDistance(trigPin_front_left, echoPin_front_left);
  cm_front_right = calculDistance(trigPin_front_right, echoPin_front_right);
  cm_left = calculDistance(trigPin_left, echoPin_left);
  cm_LEFT = calculDistance(trigPin_LEFT, echoPin_LEFT);
  cm_right = calculDistance(trigPin_right, echoPin_right);
  cm_RIGHT = calculDistance(trigPin_RIGHT, echoPin_RIGHT);

  resultatExplore = explore(cm_LEFT, cm_left, cm_front_left, cm_front_right, cm_right, cm_RIGHT);
  interrupts();

  tick++;

  if (stop_){
    dontMove();
    return;
  }
  
  // turn off leds
  // turnOffAllLeds();
  
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);

  switch(resultatExplore) {
    // move forward  
    case FORWARD_:  
      Serial.print("avant  ");
      Serial.println(tick);
      moveForward();
      break;

    // move backward
    case BACKWARD_:
      Serial.print("arriere  ");
      Serial.println(tick);
      moveBackward();
      break;

    // move left
    case LEFT_: 
      Serial.print("gauche  ");
      Serial.println(tick); 
      moveLeft();
      break;

    // move right
    case RIGHT_:
      Serial.print("droite  ");
      Serial.println(tick);
      moveRight();
      break;
  }
}


/*
 * Initial setup
 */
void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  setupTransmitter();
  setupMotors();
  //setupLeds();
}


/*
 * It's the the function that will be called at each tick time. It executes navigate and sends the message
 */
void loop()
{
  navigate();
  sendMessage();
}
