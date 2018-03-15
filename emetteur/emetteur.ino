

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

const uint8_t trigPin_left = 3;
const uint8_t echoPin_left = 2;

const uint8_t trigPin_front = 5;
const uint8_t echoPin_front = 4;

const uint8_t trigPin_right = 7;
const uint8_t echoPin_right = 6;

const int transmit_pin = 12;

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


void setup() {

  // initialize serial communication:
  Serial.begin(9600);

  /*Set up de l'émetteur*/
  vw_set_tx_pin(transmit_pin);
  vw_setup(2000);
  
}
 
void loop(){
  float cm;
  /*Serial.print("← ");
  cm = calculDistance(trigPin_left, echoPin_left);
  Serial.print(cm);

  Serial.print("   ↑ ");
  cm = calculDistance(trigPin_front, echoPin_front);
  Serial.print(cm);

  Serial.print("   ➝ ");
  cm = calculDistance(trigPin_right, echoPin_right);
  Serial.print(cm);

  Serial.println("");
  Serial.println("");*/
   const char msg[8] = "hello !"; // Tableau qui contient notre message
   
   
    vw_send((uint8_t *)msg, strlen(msg)+1); // On envoie le message 
    vw_wait_tx(); // On attend la fin de l'envoi
    Serial.println("Message send !"); // On signal la fin de l'envoi
    delay(100); // Et on attend 1s pour éviter que deux messages se superpose
}
