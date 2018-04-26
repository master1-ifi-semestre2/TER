
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
const uint8_t trigPin_front_left = 3; // trigger signal (sends)
const uint8_t echoPin_front_left = 2; // echo signal (receives)

const uint8_t trigPin_left = 9;
const uint8_t echoPin_left = 8;

const uint8_t trigPin_front_right = 6;
const uint8_t echoPin_front_right = 7;

const uint8_t trigPin_LEFT = 4;
const uint8_t echoPin_LEFT = 5;


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
volatile int currentState = FORWARD_; // initial state = forward
const int motorSpeed = 90;


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
//int randomMoveTime = 0;
//bool turn = false;

int explore(float cm_front_left, float cm_front_right, float cm_left, float cm_LEFT) {  
  Serial.print(cm_left);
  Serial.print("  -  ");
  Serial.print(cm_front_left);
  Serial.print("  -  ");
  Serial.print(cm_front_right);
  Serial.print("  -  (debout) ");
  Serial.println(cm_LEFT);


  /* Modifier algo explorateur !
  
  TODO : tourner Jusqu'a que left_diagonal trouve l'objet */


  // this is the way you would stop going around the object.... ??
  //randomMoveTime++;
  
  /*if(randomMoveTime == 50) {
    randomMoveTime = 0;
    objectDetected = 0;
  }*/
  
  // if there is enough space everywhere in front then go forward
  if ((cm_front_right > robotWidth + safetyDistance) && (cm_front_left > robotWidth + safetyDistance)) 
  {  
    if (objectDetected) {
      
      // Si il a déjà détecté un objet
      if(objectDetected == LEFT_ && cm_LEFT > safetyDistance && cm_left > safetyDistance/* && turn == false*/) {
        /* Object detected in left side, and he has pass the object, then he has to turn in left */
        Serial.println("Pass the object LEFT");
        //turn = true;
        return LEFT_;
      } 
      else if (objectDetected == LEFT_ && (cm_LEFT < safetyDistance - 10 ||  cm_left < safetyDistance - 10 || cm_front_right < safetyDistance - 10 || cm_front_left < safetyDistance - 10)){
          Serial.println("Trop près de l'object ***** ou obstacle devant ***** tourne un peu a droite");
        //turn = true;
        return RIGHT_;
      }
    }
    else if(cm_LEFT < safetyDistance || cm_left < safetyDistance) {//randomMoveTime != 0) {    // == 0 when it does the random movement... (which is not really random)
      /* He checkes both side to see if there is an object*/
      //if (cm_left < safetyDistance) {
        Serial.println("object detected LEFT");
        objectDetected = LEFT_;
     // }   
    } 
    //Serial.println("↑");
    /* There is no object or he has just faund one */
    //turn = false;
    return FORWARD_;   
  } 
  // if there is not enough space to go forward, but there's enough to turn then turn right or left (where there is more space)
  /*else if (cm_front_left > robotWidth || cm_front_right > robotWidth) 
  {
    if (cm_left > cm_right) {
      //Serial.println("←");
      Serial.println("object detected FRONT -> side RIGHT");
      objectDetected = RIGHT_;
      return LEFT_;
    }
    else {
      //Serial.println("➝");
      Serial.println("object detected FRONT -> side LEFT");
      objectDetected = LEFT_;
      return RIGHT_;
    }   
  }
  // if there's not enough space anywhere then go backward
  else {
    //Serial.println("↓");
    return BACKWARD_;
  }*/
  }








int tick = 0;

/*
 * Moves the wheels
 */
void navigate()
{
  int resultatExplore;
  
  float cm_front_left;  // distance of the obstacle
  float cm_front_right;
  float cm_left;
  float cm_LEFT;
  
  noInterrupts();
  cm_front_left = calculDistance(trigPin_front_left,echoPin_front_left);
  cm_front_right = calculDistance(trigPin_front_right,echoPin_front_right);
  cm_left = calculDistance(trigPin_left, echoPin_left);
  cm_LEFT = calculDistance(trigPin_LEFT, echoPin_LEFT);

  resultatExplore = explore(cm_front_left, cm_front_right, cm_left, cm_LEFT);
  interrupts();

  tick++;
  Serial.print("                                     tick ");
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
