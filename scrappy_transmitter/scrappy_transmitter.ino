
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

typedef struct {
  int id;
  int value;
} Message;

Message message;
byte messageSize = sizeof(message);

const uint8_t myId = 0; // boss

const uint8_t trigPin_front_left = 3; //envoie de signal
const uint8_t echoPin_front_left = 2; //reçoit le signal

const uint8_t trigPin_left = 9;
const uint8_t echoPin_left = 8;

const uint8_t trigPin_front_right = 6;
const uint8_t echoPin_front_right = 7;

const uint8_t trigPin_right = 4;
const uint8_t echoPin_right = 5;

const int transmit_pin = 12;

//const int r = 22.5 * 1.866; // Distance entre le centre du robot et capteurs
//const float teta = 30;
const float safetyDistance = 20; // cm en fonction de la vitesse
const float robotWidth = 20; // Hauteur 12 cm

/* Etat initial quelconque */
volatile int currentState = 5;


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motorRight = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *motorLeft = AFMS.getMotor(2);

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

int explore(float cm_front_left, float cm_front_right, float cm_left, float cm_right){
    
 /* Serial.print("G = ");
  Serial.print(cm_left);
  Serial.print("  F = ");
  Serial.print(cm_front_left);
  Serial.print(" - ");
  Serial.print(cm_front_right);
  Serial.print("  D = ");
  Serial.println(cm_right);*/

     if (((cm_front_right > robotWidth + safetyDistance) 
      || (cm_front_left > robotWidth + safetyDistance))
      && (cm_left > robotWidth) 
      && (cm_right > robotWidth)) {
        // Si il y a de la place de partout, avancer
         Serial.println("↑");
         return 0;
   }
   else if (cm_front_left > robotWidth || cm_front_right > robotWidth){
      // Si il n'y a plus de place devant mais reste de la place pour tourner, tourne dans un des côtés 
        if (cm_left > cm_right){
             Serial.println("←");
             return -1;
        }
        else {
             Serial.println("➝");
             return 1;
        }   
   }
   else {
        // sinon reculer
        Serial.println("↓");
        return 2;
   }
 } 

void navigate()
{
  float cm_front_left;  // distance of the obstacle
  float cm_front_right;
  float cm_left;
  float cm_right;
  
  int resultatExplore;
  
  noInterrupts();
  cm_front_left = calculDistance(trigPin_front_left,echoPin_front_left);
  cm_front_right = calculDistance(trigPin_front_right,echoPin_front_right);
  cm_left = calculDistance(trigPin_left, echoPin_left);
  cm_right = calculDistance(trigPin_right, echoPin_right);

  resultatExplore=explore(cm_front_left, cm_front_right, cm_left, cm_right);
  
  interrupts();
   
  // S'il tourne, ne pas s'arrêter jusqu'a qu'il trouve de la place devant
  if ((currentState == 1 || currentState == -1) && (resultatExplore == 1 || resultatExplore == -1)){
    resultatExplore = currentState;
  }
   if(resultatExplore != currentState){
    currentState = resultatExplore;
    motorRight->run(RELEASE);
    motorLeft->run(RELEASE);
  
    if(resultatExplore == 0){ 
      // marche avant  

      Serial.println("Marche avant");
      motorRight->run(FORWARD);
      motorLeft->run(FORWARD);

      message.value = 0;
      
    }
    else if(resultatExplore == 2){
      // marche arrière 
      Serial.println("Marche arriere");
      motorRight->run(BACKWARD);
      motorLeft->run(BACKWARD);

      message.value = 2;
    }
    else if(resultatExplore == -1){ 
      // tourner à gauche
      Serial.println("gauche"); 
      motorRight->run(BACKWARD);
      motorLeft->run(FORWARD);

      message.value = -1;
    }
    else if(resultatExplore == 1){
      // tourner à droite 
      Serial.println("droite");
      motorRight->run(FORWARD);
      motorLeft->run(BACKWARD);

      message.value = 1;
    }
  } 
}

void send_message(){
  vw_send((byte*) &message, sizeof(message));
  vw_wait_tx(); // On attend la fin de l'envoi
  //Serial.println("Message send !");
  delay(300);
}

void setup() {
 
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  /* Set up de l'émetteur */
  vw_set_tx_pin(transmit_pin);
  vw_set_ptt_inverted(true);
  vw_setup(2000);

  message.id = myId;
  message.value = 0;
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorRight->setSpeed(80);
  motorRight->run(FORWARD);
  // turn on motor
  motorRight->run(RELEASE);
  
  //demarre le moteur numero 2
  motorLeft->setSpeed(80);
  motorLeft->run(FORWARD);
  // turn on motor
  motorLeft->run(RELEASE);
  
}


void loop()
{
  navigate();
  send_message();
  
}
