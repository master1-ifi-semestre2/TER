/****************************************************************************
   Sonar Robot

   Authors:  
      - WILHELM Andreina
      - BENSOUSSAN Chloé
      - GRÉAU Alexandre
      - BOUKOU Grâce
       
   Permissions: MIT licence
   
   Remarks:
      
*****************************************************************************/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
#include <TimerOne.h>

/*
  Gauche : capteur 1
  Milieu : capteur 2
  Droite : capteur 3
  */

const uint8_t trigPin1 = 3; //envoie de signal
const uint8_t echoPin1 = 2; //reçoit le signal
const uint8_t trigPin2 = 5;
const uint8_t echoPin2 = 4;
const uint8_t trigPin3 = 6;
const uint8_t echoPin3 = 7;

const int r = 22.5 * 1.866; // Distance entre le centre du robot et capteurs
const float teta = 30;
const float safetyDistance = 20; // cm en fonction de la vitesse
const float robotWidth = 20; 
float h1; //espace libre à gauche 
float h2; // espace libre à droite 
float d1; // distances obstacles/capteurs
float d2;
float d3;
float l1;
float l2;
float l3;

/* Etat initial = avancer */
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

int explore(float cm1,float cm2,float cm3){
     
  /*Serial.print("Capteur gauche : ");                               
  Serial.print(cm1);
  Serial.print(" cm");
  Serial.println();
  Serial.print("Capteur milieu : ");      
  Serial.print(cm2);
  Serial.print(" cm");
  Serial.println();
  Serial.print("Capteur droite : ");      
  Serial.print(cm3);
  Serial.print(" cm");
  Serial.println();*/
  
  d1=cm1;
  d2=cm2;
  d3=cm3;
  
  //demi distance des capteurs 1 & 3 + 21cm
  h1 = d1 / 2 + 21;
  h2 = d3 / 2 + 21;
  
  l1 = d1 * 0.866 - 5.63;
  l2 = (d3 - d1) * 0.866;
  l3 =  (d2 - d3) * 0.866 - 5.63;
  
  if((d2 > robotWidth + safetyDistance)
    && (h2 > (robotWidth + safetyDistance)) 
    && (h1 > (robotWidth + safetyDistance))) {
    /* Si il y a plus de 40 cm devant lui et de chaque coté */
      Serial.print("↑");
      Serial.println();
      return 0;
  }
  else if (d2 > robotWidth){
    /* Si il ne peut plus avancer et reste de l'espace entre les 2, choisir le côté où il y a le plus d'espace */
      if(d1 > d3){
        Serial.print("← 30°, gauche = ");
        Serial.print(d1);
        Serial.print("  droite = ");
        Serial.print(d3);
        Serial.println();
        return -1;
      }
      else {
         Serial.print("➝ 30°, gauche = ");
        Serial.print(d1);
        Serial.print("  droite = ");
        Serial.print(d3);
         Serial.println();
         return 1;
      }
  }
  else {
      /* Faire marche arrière */
      Serial.print("↓");
      Serial.println();
      return 2;
    }
 } 

void navigate()
{
  float cm1;  // distance of the obstacle
  float cm2;
  float cm3;
  int resultatExplore;
  
   noInterrupts();
  cm1 = calculDistance(trigPin1,echoPin1);
  cm2 = calculDistance(trigPin2,echoPin2);
  cm3 = calculDistance(trigPin3,echoPin3);

  resultatExplore=explore(cm1,cm2,cm3);
  
   interrupts();
  Serial.print("Explore retourne : ");
  Serial.print(resultatExplore);
  Serial.println();
  
  if(resultatExplore != currentState){
    currentState = resultatExplore;
    motorRight->run(RELEASE);
    motorLeft->run(RELEASE);
  
    if(resultatExplore == 0){ 
      /* marche avant */  
      motorRight->run(FORWARD);//run(BACKWARD);
      motorLeft->run(FORWARD);
    }
    else if(resultatExplore == 2){
      /* marche arrière */ 
      motorRight->run(BACKWARD);
      motorLeft->run(BACKWARD);
    }
    else if(resultatExplore == -1){ 
      /* tourner à gauche */
      motorRight->run(BACKWARD);
      motorLeft->run(FORWARD);
    }
    else if(resultatExplore == 1){
      /* tourner à droite */
      motorRight->run(FORWARD);
      motorLeft->run(BACKWARD);
    }
  }
}

 
void setup() {
  
  // initialize serial communication:
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
 
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorRight->setSpeed(100);
  motorRight->run(FORWARD);
  // turn on motor
  motorRight->run(RELEASE);
  
  //demarre le moteur numero 2
  motorLeft->setSpeed(100);
  motorLeft->run(FORWARD);
  // turn on motor
  motorLeft->run(RELEASE);
  //a completer avec temps correspondant en milliseconde voir la frequ a donner 
  //PIN 10 ET 9 inutilisable
  Timer1.initialize(1000000);  
  //attacher la methode calcul de distance , a noter periode non obligatoire.
  Timer1.attachInterrupt(navigate);
 
}
 
void loop(){

}
