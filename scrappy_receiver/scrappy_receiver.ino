
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
const int receive_pin = 11;
const uint8_t myId = 1; // follower
bool formationMode = true;                // USE IT OR NOT

typedef struct {
  int id;
  int value;
} Message;

Message msg;
byte msgSize = sizeof(msg);


/* Measurements */
//const int r = 22.5 * 1.866; // distance between the center of the robot and the sensors
//const float teta = 30;
const float safetyDistance = 20; // according with the speed, expressed in cm
const float robotWidth = 20; // and the height is 12 cm


/* LED */
const uint8_t ledPin_back = 13;


/* Movement */
volatile int currentState = 5; // initial state = forward
const int motorSpeed = 0;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *motorRight = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *motorLeft = AFMS.getMotor(2);


/*float h_left; //espace libre à gauche 
float h_right; // espace libre à droite */

/*float d_front_left; // distances obstacles/capteurs
float d_front_right;
float d_left;
float d_right;*/

/*float l_front_left;
float l_front_right;
float l_left;
float l_right;*/

/*
 * Calculates the distance with the information obtained from the sensors  
 */
float calculDistance(uint8_t trigPin,uint8_t echoPin) {
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
int explore(float cm_front_left, float cm_front_right, float cm_left, float cm_right) {
    
  /*Serial.print("devant gauche = ");
  Serial.print(cm_front_left);
  Serial.print("    devant droite = ");
  Serial.println(cm_front_right);
  Serial.print("  gauche = ");
  Serial.print(cm_left);
  Serial.print("  droit = ");
  Serial.println(cm_right);*/

   // if there is enough space everywhere then go forward
   if ((cm_front_right > robotWidth + safetyDistance) && (cm_front_left > robotWidth + safetyDistance) && cm_left > robotWidth && cm_right > robotWidth) {     
         Serial.println("↑");
         return 0;
   }
   // if there is not enough space in front of it, but there's enough to turn then turn right or left (where there is more space)
   else if (cm_front_left > robotWidth || cm_front_right > robotWidth) {
        if (cm_left > cm_right){
             Serial.println("←");
             return -1;
        }
        else {
             Serial.println("➝");
             return 1;
        }   
   }
   // if there's not enough space anywhere then go backward
   else {
        Serial.println("↓");
        return 2;
   }
} 



/*
 * Moves the wheels
 */
void navigate()
{
  int resultatExplore;

  if(formationMode) 
  {
    //if (msg.id != myId) { // for when it has both an emitter and receiver, we want make sure we are not reading the message that we are sending
      resultatExplore = msg.value;
      //delay(100);
    //}
  } else {
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
  }

  // if it turns then don't stop turning until it finds free space in front of it
  if ((currentState == 1 || currentState == -1) && (resultatExplore == 1 || resultatExplore == -1)) {
    resultatExplore = currentState;
  }

  // turn off leds
  digitalWrite(ledPin_back, LOW);
  delay(100);   // SEE IF THIS DELAY AFFECTS FOLLOWING THE LEADER WHEN TURNING
  
  currentState = resultatExplore;
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);

  // move forward  
  if(resultatExplore == 0) { 
    motorRight->run(FORWARD);
    motorLeft->run(FORWARD);
    Serial.println("  forward");
  }
  // move backward
  else if(resultatExplore == 2) {
    motorRight->run(BACKWARD);
    motorLeft->run(BACKWARD);
    
    // turn on backward led
    digitalWrite(ledPin_back, HIGH);
    
    Serial.println("  backward");
  }
  // move left
  else if(resultatExplore == -1) { 
    motorRight->run(BACKWARD);
    motorLeft->run(FORWARD);
    Serial.println("  left");
  }
  // move right
  else if(resultatExplore == 1) {
    motorRight->run(FORWARD);
    motorLeft->run(BACKWARD);
    Serial.println("  right");
  }
}



/*
 * Initial setup
 */
void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Setup the receiver for communication
  vw_set_rx_pin(receive_pin);
  vw_setup(2000); 
  vw_rx_start();  

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
  pinMode(ledPin_back, OUTPUT);
  
  //a completer avec temps correspondant en milliseconde voir la frequ a donner 
  //Timer1.initialize(1000000);  
  //attacher la methode calcul de distance , a noter periode non obligatoire.
  //Timer1.attachInterrupt(navigate);
}



/*
 * It reads the message received and execute navigate
 */
void loop()
{
    // check if a message has been received and stores it in the corresponding structure
    if (vw_get_message((byte *) &msg, &msgSize) && formationMode) // Non-blocking
    {
      Serial.print("Id: ");
      Serial.print(msg.id);
      Serial.print("  Value: ");
      Serial.print(msg.value); 
    }

    navigate();
}
