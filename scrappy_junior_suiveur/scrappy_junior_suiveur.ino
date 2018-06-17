/****************************************************************************
   Sonar Robot

   Authors:  
      - BENSOUSSAN Chloé
      - BOUKOU Grâce
      - GRÉAU Alexandre      
      - WILHELM Andreina
          
   Permissions: MIT licence
      
*****************************************************************************/

#include <RH_ASK.h>
#include <SPI.h>

//#include <Wire.h>
#include <Adafruit_MotorShield.h>
//#include <Adafruit_PWMServoDriver.h>
//#include <TimerOne.h>
//#include <VirtualWire.h>


#define FORWARD_ 0
#define BACKWARD_ 2
#define LEFT_ -1
#define RIGHT_ 1
#define STOP_ 3


/* Ultrasonic sensors */      
//const uint8_t echoPin = 2;  // echo signal (receives)
//const uint8_t trigPin = 3;  // trigger signal (sends)


/* Communication */
RH_ASK driver(2000, 14, 15);
int msg;
int msgSize = sizeof(msg);

/*const int receive_pin = 7;
const uint8_t myId = 1; // follower

typedef struct {
  int id;
  int value;
} Message;

Message msg;
byte msgSize = sizeof(msg);
*/

/* Measurements */
const float safetyDistance = 30; // according with the speed. Expressed in cm
const float robotWidth = 20;  // expressed in cm


/* Movement */
volatile int currentState = STOP_; // initial state = stop
const int motorSpeed = 100; // from 0 (off) to 255 (max speed)

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
 // TODO: Don't start the wheels by default
void setupMotors() {
  // Left wheel
  motorLeft->setSpeed(motorSpeed);
  //motorLeft->run(FORWARD);
  // turn on motor
  //motorLeft->run(RELEASE);
  
  // Right wheel
  motorRight->setSpeed(motorSpeed - 30);
  //motorRight->run(FORWARD);
  // turn on motor
  //motorRight->run(RELEASE);
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
  /*// Setup the receiver
  vw_set_rx_pin(receive_pin);
  vw_setup(2000); 
  vw_rx_start(); 

  // Set initial default message
  //msg.id = myId;
 // msg.value = FORWARD_;   // TODO: See if it's better to change it so it doesn't move until it actually receives something
 */
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
  //float cm_front;  // distance of Master
  //noInterrupts();
  //cm_front = calculDistance(trigPin, echoPin);
  //interrupts();
  //Serial.print(cm_front);

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
  setupMotors();
}


/*
 * It's the the function that will be called at each tick time. It executes navigate and sends the message
 */
void loop()
{
  // check if a message has been received and stores it in the corresponding structure
 /* if (vw_get_message((byte *) &msg, &msgSize)) // Non-blocking
  {
    Serial.println(""); 
    Serial.print("Id: ");
    Serial.print(msg.id);
    Serial.print("  Value: ");
    Serial.println(msg.value); 
    currentState = msg.value;
  }*/  
  
  if (driver.recv((uint8_t *)&msg, (uint8_t *)&msgSize)) // Non-blocking
    {
        // Message with a good checksum received, dump it.
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
