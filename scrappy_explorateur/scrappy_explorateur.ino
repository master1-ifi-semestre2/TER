
/****************************************************************************
   Sonar Robot

   Authors:  
      - WILHELM Andreina
      - BENSOUSSAN Chloé
      - GRÉAU Alexandre
      - BOUKOU Grâce
       
   Permissions: MIT licence
      
*****************************************************************************/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
#include <TimerOne.h>
#include <VirtualWire.h>

/* Ultrasonic sensors */
const uint8_t trigPin_front_left = 3; // trigger signal (sends)
const uint8_t echoPin_front_left = 2; // echo signal (receives)

const uint8_t trigPin_left = 9;
const uint8_t echoPin_left = 8;

const uint8_t trigPin_front_right = 6;
const uint8_t echoPin_front_right = 7;

const uint8_t trigPin_right = 4;
const uint8_t echoPin_right = 5;


/* Communication */
const int transmit_pin = 12;    
const uint8_t myId = 0; // boss

typedef struct {
  int id;
  int value;
} Message;

Message msg;


/* Measurements */
//const int r = 22.5 * 1.866; // distance between the center of the robot and the sensors
//const float teta = 30;
const float safetyDistance = 20; // according with the speed, expressed in cm
const float robotWidth = 20; // and the height is 12 cm


/* LEDs
 * long side : pin
 * short side : ground
 * resistor : 100 Ohm
 */
const uint8_t ledPin_left = 14;
const uint8_t ledPin_back = 15;
const uint8_t ledPin_right = 16;

/* Movement */
volatile int currentState = 5; // initial state = forward
const int motorSpeed = 200;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

/* Motor 1 -> right Motor 2 -> left */
Adafruit_DCMotor *motorRight = AFMS.getMotor(1);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(2);



/*
 * Calculates the distance with the information obtained from the sensors  
 */
float calculDistance(uint8_t trigPin,uint8_t echoPin){
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
 * Determines where to move
 */
int sideDetected = 5;
int explore(float cm_front_left, float cm_front_right, float cm_left, float cm_right) {
      
  Serial.print(cm_left);
  Serial.print("  -  ");
  Serial.print(cm_front_left);
  Serial.print("  -  ");
  Serial.print(cm_front_right);
  Serial.print("  -  ");
  Serial.println(cm_right);

   // if there is enough space everywhere then go forward
   if ((cm_front_right > robotWidth + safetyDistance) && (cm_front_left > robotWidth + safetyDistance) && cm_left > robotWidth && cm_right > robotWidth) {     
         //Serial.println("↑");
         return 0;
   }


   
   // if there is not enough space in front of it, but there's enough to turn then turn right or left (where there is more space)
   else if (cm_front_left > robotWidth || cm_front_right > robotWidth) {
        if (cm_left > cm_right){
             //Serial.println("←");
             sideDetected = 1;
             return -1;
        }
        else {
             //Serial.println("➝");
             sideDetected = -1;
             return 1;
        }   
   }
   // if there's not enough space anywhere then go backward
   else {
        //Serial.println("↓");
        return 2;
   }
} 



/*
 * Moves the wheels
 */
void navigate()
{
  int resultatExplore;
  
  float cm_front_left;  // distance of the obstacle
  float cm_front_right;
  float cm_left;
  float cm_right;
  
  noInterrupts();
  cm_front_left = calculDistance(trigPin_front_left,echoPin_front_left);
  cm_front_right = calculDistance(trigPin_front_right,echoPin_front_right);
  cm_left = calculDistance(trigPin_left, echoPin_left);
  cm_right = calculDistance(trigPin_right, echoPin_right);

  resultatExplore=explore(cm_front_left, cm_front_right, cm_left, cm_right);
  interrupts();
   
  // if it turns then don't stop turning until it finds free space in front of it
  if ((currentState == 1 || currentState == -1) && (resultatExplore == 1 || resultatExplore == -1)) {
    resultatExplore = currentState;
  }

  // turn off leds
  digitalWrite(ledPin_left, LOW);
  digitalWrite(ledPin_back, LOW);
  digitalWrite(ledPin_right, LOW);
  delay(100);
  
  currentState = resultatExplore;
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);

  // move forward  
  if(resultatExplore == 0) { 
    Serial.println("Marche avant");
    motorRight->run(FORWARD);
    motorLeft->run(FORWARD);

    msg.value = 0; 
  }
  // move backward
  else if(resultatExplore == 2) {
    Serial.println("Marche arriere");
    motorRight->run(BACKWARD);
    motorLeft->run(BACKWARD);

    // turn on backward led
    digitalWrite(ledPin_back, HIGH);

    msg.value = 2;
  }
  // move left
  else if(resultatExplore == -1) { 
    Serial.println("gauche"); 
    motorRight->run(BACKWARD);
    motorLeft->run(FORWARD);

    // turn on left led
    digitalWrite(ledPin_left, HIGH);

    msg.value = -1;
  }
  // move right
  else if(resultatExplore == 1) {
    Serial.println("droite");
    motorRight->run(FORWARD);
    motorLeft->run(BACKWARD);

    // turn on right led
    digitalWrite(ledPin_right, HIGH);

    msg.value = 1;
  }
}



/*
 * Sends message on how to move to other robot
 */
void send_message() {
  vw_send((byte*) &msg, sizeof(msg));
  vw_wait_tx(); // On attend la fin de l'envoi
  Serial.println("Message send !");
  delay(300);
}
 

/*
 * Initial setup
 */
void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // Setup the transmitter for communication
  vw_set_tx_pin(transmit_pin);
  vw_set_ptt_inverted(true);
  vw_setup(2000);

  // Set initial message
  msg.id = myId;
  msg.value = 0;

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

  // setup leds
  pinMode(ledPin_left, OUTPUT);
  pinMode(ledPin_back, OUTPUT);
  pinMode(ledPin_right, OUTPUT);
}


/*
 * It executes navigate and sends the message
 */
void loop()
{
  navigate();
  //send_message();
}
