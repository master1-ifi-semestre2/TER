
/****************************************************************************
   Sonar Robot

   Authors:  
      - BENSOUSSAN Chloé
      - BOUKOU Grâce
      - GRÉAU Alexandre      
      - WILHELM Andreina
          
   Permissions: MIT licence
      
*****************************************************************************/



// CHECK TYPES IN EVERY FILE 




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
const uint8_t trigPin_LEFT = 10;
const uint8_t echoPin_LEFT = 11;

const uint8_t trigPin_left = 5;
const uint8_t echoPin_left = 4;

const uint8_t trigPin_front_left = 2; // trigger signal (sends)
const uint8_t echoPin_front_left = 3; // echo signal (receives)

const uint8_t trigPin_front_right = 12;
const uint8_t echoPin_front_right = 13;

const uint8_t trigPin_right = 7;
const uint8_t echoPin_right = 6;

const uint8_t trigPin_RIGHT = 8;
const uint8_t echoPin_RIGHT = 9;



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
const float safetyDistance = 27; // according with the speed, expressed in cm
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
volatile int currentState = FORWARD_; // initial state = forward
const int motorSpeed = 110;


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Motor 1 -> right Motor 2 -> left
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
int objectDetected = 0; // 0 false, !0 true
boolean searchingObject = false;
int tick = 0;

int explore(float cm_LEFT, float cm_left, float cm_front_left, float cm_front_right, float cm_right, float cm_RIGHT) {  
  Serial.println(cm_LEFT);
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

  if (tick < 3){
    return FORWARD_; 
  }

 if (searchingObject == true && objectDetected == LEFT_){
    if (cm_LEFT < safetyDistance ||  cm_left < safetyDistance){
         searchingObject = false;
    }
    else {
         Serial.println(" Tourne à droite pour retrouver l'object ");
         return RIGHT_;
    }
 }
 else if (searchingObject == true && objectDetected == RIGHT_){
     if (cm_RIGHT < safetyDistance ||  cm_right < safetyDistance){
         searchingObject = false;
    }
    else {
         Serial.println(" Tourne à gauche pour retrouver l'object ");
         return LEFT_;
    }
 }

 if (objectDetected == LEFT_) {
      
      // Si il a déjà détecté un objet
      if(cm_LEFT > safetyDistance && cm_left > safetyDistance) {
        /* Object detected in left side, and he has pass the object, then he has to turn in left */
        Serial.println("Pass the object LEFT");
        //turn = true;
        return LEFT_;
      } 
      if (cm_LEFT < safetyDistance - 10 ||  cm_left < safetyDistance - 10 || cm_front_right < safetyDistance || cm_front_left < safetyDistance){
          Serial.println("Trop près de l'object ***** ou obstacle devant ***** tourne un peu a droite");
        //turn = true;
        return RIGHT_;
      } 
    Serial.println("Tout droit object detected");
    return FORWARD_; 
  } 
  else if (objectDetected == RIGHT_) {
      
      // Si il a déjà détecté un objet
      if(cm_RIGHT > safetyDistance && cm_right > safetyDistance) {
        /* Object detected in left side, and he has pass the object, then he has to turn in left */
        Serial.println("Pass the object RIGHT");
        return RIGHT_;
      } 
      if (cm_RIGHT < safetyDistance - 10 ||  cm_right < safetyDistance - 10 || cm_front_right < safetyDistance || cm_front_left < safetyDistance){
          Serial.println("Trop près de l'object ***** ou obstacle devant ***** tourne un peu a gauche");
          return LEFT_;
      } 
    Serial.println("Tout droit object detected");
    return FORWARD_; 
  }
    
  else {
    /* Pas d'object détecté !!*/

    if(cm_LEFT < safetyDistance || cm_left < safetyDistance) {
        /* Detected an object in left side */
        Serial.println("object detected LEFT");
         objectDetected = LEFT_;
     }
     else if(cm_RIGHT < safetyDistance || cm_right < safetyDistance) {
        /* Detected an object in right side */
        Serial.println("object detected RIGHT");
         objectDetected = RIGHT_;
     }
     else if(cm_front_left < safetyDistance || cm_front_right < safetyDistance) {
        /* He detects an object in front of him */
        Serial.print("object detected FRONT");

        if(cm_LEFT < safetyDistance || cm_left < safetyDistance) {
            /* There is an object on left side */
            Serial.println(" - LEFT");
            objectDetected = LEFT_;
            return RIGHT_;
        }
        else if(cm_RIGHT < safetyDistance || cm_right < safetyDistance) {
            /* There is an object on right side */
            Serial.println(" - RIGHT");
            objectDetected = RIGHT_;
            return LEFT_;
        }

        else {
            /* Compare both side */
            if  (cm_right > cm_left){
                Serial.println(" - LEFT");
                objectDetected = LEFT_;
                searchingObject = true;
                return RIGHT_;
            }
            else {
              Serial.println(" - RIGHT");
              objectDetected = RIGHT_;
              searchingObject = true;
              return LEFT_;
            }  
        }       
     }
     Serial.println("Tout droit");
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
  cm_front_left = calculDistance(trigPin_front_left,echoPin_front_left);
  cm_front_right = calculDistance(trigPin_front_right,echoPin_front_right);
  cm_left = calculDistance(trigPin_left, echoPin_left);
  cm_LEFT = calculDistance(trigPin_LEFT, echoPin_LEFT);
  cm_right = calculDistance(trigPin_right, echoPin_right);
  cm_RIGHT = calculDistance(trigPin_RIGHT, echoPin_RIGHT);

  resultatExplore = explore(cm_LEFT, cm_left, cm_front_left, cm_front_right, cm_right, cm_RIGHT);
  interrupts();

  tick++;
  Serial.print("                                                    tick ");
  Serial.println(tick);
  
  // turn off leds
  digitalWrite(ledPin_left, LOW);
  digitalWrite(ledPin_back, LOW);
  digitalWrite(ledPin_right, LOW);
  delay(100);
  
  currentState = resultatExplore;
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);

  // move forward  
  if(resultatExplore == FORWARD_) { 
    //Serial.println("Marche avant");
    motorRight->run(FORWARD);
    motorLeft->run(FORWARD);

    msg.value = FORWARD_; 
  }
  // move backward
  else if(resultatExplore == BACKWARD_) {
    //Serial.println("Marche arriere");
    motorRight->run(BACKWARD);
    motorLeft->run(BACKWARD);

    // turn on backward led
    digitalWrite(ledPin_back, HIGH);

    msg.value = BACKWARD_;
  }
  // move left
  else if(resultatExplore == LEFT_) { 
    //Serial.println("gauche"); 
    motorRight->run(RELEASE);
    motorLeft->run(FORWARD);

    // turn on left led
    digitalWrite(ledPin_left, HIGH);

    msg.value = LEFT_;
  }
  // move right
  else if(resultatExplore == RIGHT_) {
    //Serial.println("droite");
    motorRight->run(FORWARD);
    motorLeft->run(RELEASE);

    // turn on right led
    digitalWrite(ledPin_right, HIGH);

    msg.value = RIGHT_;
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
