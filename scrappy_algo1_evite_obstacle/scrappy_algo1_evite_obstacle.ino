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


/* LEDs
 * long side : pin
 * short side : ground
 * resistor : 100 Ohm
 */
const uint8_t ledPin_left = 15;
const uint8_t ledPin_back = 16;
const uint8_t ledPin_right = 17;


/* Measurements */
const float safetyDistance = 20; // according with the speed. Expressed in cm
const float robotWidth = 20;  // expressed in cm


/* Movement */
volatile int currentState = FORWARD_; // initial state = forward
const int motorSpeed = 200; // from 0 (off) to 255 (max speed)

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Motor 1 -> left / Motor 2 -> right
Adafruit_DCMotor *motorLeft = AFMS.getMotor(4);
Adafruit_DCMotor *motorRight = AFMS.getMotor(2);


/* Communication */
RH_ASK transmitter(2000, 15, 14);  // transmitter pin 14, receiver pin 15 (but receiver is not used)
int msg = currentState;


/*
 * Motors setup and movement
 */ 
void setupMotors() {
  // Left wheel
  motorLeft->setSpeed(motorSpeed);
  motorLeft->run(FORWARD);
  motorLeft->run(RELEASE);
  
  // Right wheel
  motorRight->setSpeed(motorSpeed + 10);
  motorRight->run(FORWARD);
  motorRight->run(RELEASE);
}

void moveForward() {
  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);
  msg = FORWARD_; 
}

void moveBackward() {
  motorLeft->run(BACKWARD);
  motorRight->run(BACKWARD);

  // turn on back led
  // turnOnLed(ledPin_back);

  msg = BACKWARD_;
}

void moveLeft() {
  motorLeft->run(BACKWARD);
  motorRight->run(FORWARD);

  // turn on left led
  // turnOnLed(ledPin_left);

  msg = LEFT_;
}

void moveRight() {
  motorLeft->run(FORWARD);
  motorRight->run(BACKWARD);

  // turn on right led
  // turnOnLed(ledPin_right);

  msg = RIGHT_;
}


/*
 * Transmitter setup and send message
 */
void setupTransmitter() {
  if (!transmitter.init())
         Serial.println("Init failed");
}

/* Sends the movement to be followed (the int indicating the direction) */
void sendMessage() {
  transmitter.send((uint8_t *) &msg, sizeof(msg));
  transmitter.waitPacketSent();
  delay(20);
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
float calculDistance(uint8_t trigPin, uint8_t echoPin) {
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
 * Determines where to move to avoid obstacles
 */
int explore(float cm_front_left, float cm_front_right, float cm_left, float cm_right, float cm_LEFT, float cm_RIGHT) {

  /* Print distance of each sensor - for debugging
  Serial.print(cm_LEFT);
  Serial.print(" - ");
  Serial.print(cm_left);
  Serial.print(" - ");
  Serial.print(cm_front_left);
  Serial.print(" - ");
  Serial.print(cm_front_right);
  Serial.print(" - ");
  Serial.println(cm_right);
  Serial.print(" - ");
  Serial.println(cm_RIGHT);
  */

   // if there is enough space everywhere in front of him then go forward
   if ((cm_front_right > robotWidth + safetyDistance) && (cm_front_left > robotWidth + safetyDistance) && cm_left > safetyDistance && cm_right > safetyDistance) {    
         return FORWARD_;
   }
   // if there is not enough space in front of him, but there's enough to turn then turn right or left (where there is more space)
   else if (cm_front_left > safetyDistance || cm_front_right > safetyDistance) {
        if (cm_left > cm_right || cm_LEFT > cm_RIGHT) {
             return LEFT_;
        }
        else {
             return RIGHT_;
        }   
   }
   // if there's not enough space anywhere then go backward
   else {
        return BACKWARD_;
   }
} 


/*
 * Moves the robot
 */
void navigate()
{
  int resultatExplore;

  // distances from all sides
  float cm_front_left;
  float cm_front_right;
  float cm_left;
  float cm_right;
  float cm_LEFT;
  float cm_RIGHT;  
  
  noInterrupts();
  cm_front_left = calculDistance(trigPin_front_left, echoPin_front_left);
  cm_front_right = calculDistance(trigPin_front_right, echoPin_front_right);
  cm_left = calculDistance(trigPin_left, echoPin_left);
  cm_right = calculDistance(trigPin_right, echoPin_right);
  cm_LEFT = calculDistance(trigPin_LEFT, echoPin_LEFT);
  cm_RIGHT = calculDistance(trigPin_RIGHT, echoPin_RIGHT);  

  resultatExplore=explore(cm_front_left, cm_front_right, cm_left, cm_right, cm_LEFT, cm_RIGHT);  
 
  interrupts();

  // if it turns then don't stop turning until it finds free space in front of him
  if ((currentState == RIGHT_ || currentState == LEFT_) && (resultatExplore == LEFT_ || resultatExplore == RIGHT_)) {
    resultatExplore = currentState;
  }

  // turn off leds
  // turnOffAllLeds();
  
  currentState = resultatExplore;
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);

  switch(resultatExplore) {
    // move forward  
    case FORWARD_:  
      //Serial.println("avant");
      moveForward();
      break;

    // move backward
    case BACKWARD_:
      //Serial.println("arriere");
      moveBackward();
      break;

    // move left
    case LEFT_: 
      //Serial.println("gauche"); 
      moveLeft();
      break;

    // move right
    case RIGHT_:
      //Serial.println("droite");
      moveRight();
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
  
  setupTransmitter();
  setupMotors();
  //setupLeds();
}


/*
 * It's the function that will be called at each tick time. It executes navigate and sends the message
 */
void loop()
{
  navigate();
  sendMessage();
}
