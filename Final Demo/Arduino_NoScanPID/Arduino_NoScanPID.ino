/*
 * Demo 1
 * Filename: Demo1_Feedback_Function_Implementation.ino
 *
 * Purpose: Accurately move a two-wheeled robot to a desired position via
            rotation and forward movement functions where a desired set angle (deg)
            and set distance (cm) are inputs to the functions. Use of appropriate PID
            controllers implemented to ensure accurate movement.
 * Class: EENG350, SEED Lab
 * Creators: Connor Deny (PI motor control) & Sean West (function implementation)
 * Co-Creator: Brian Burr (I2C)
 * Date: 03/09/2023
 *
 */

// include libraries
#include <Encoder.h>
#include <Wire.h>

// pin connections encoderL(A,B) encoderR(C,D), motor1, and motor2
const int encoderPinA = 2;   // CLK encoder 1
const int encoderPinB = 5;   // DAT pin

const int encoderPinC = 3;   // CLK encoder 2
const int encoderPinD = 6;   // DAT

const int motorEnable = 4;   // Tri-state di sable both motor chanels when LOW (enable)
const int M1DIR = 7;         // Motor polarity 1 (direction)
const int M1PWM = 9;         // Motor volt 1 "speed"
const int M2DIR = 8;         // Motor polarity 1 (direction)
const int M2PWM = 10;

// setup encoder class to be used
Encoder encoderL(encoderPinA, encoderPinB);
Encoder encoderR(encoderPinC, encoderPinD);

// PI controller variables
//int updateFrequency = 100;               // independent update frequency per function?
const int counts_per_rotation = 3200;    // the motor I took home was 3,200 but it should be 1,600 counts?
unsigned long int previousMillis = 0;    // use to determine delta_t

// positional variables
int arucoPosition = 2;                   // aruco position: 0,1,2, or 3
const float motorSetPositionThreshold_deg = 5.0;  // allowable angular difference from set point allowed

// movement function global variables
bool isForwardMoveDone = 0;
bool isRotationDone = 0;
const float wheelRadius_cm = 7.75;
const int maxSpeed = 190;                 // limit maximum motor speed
float motorPosition_degL = 0.0;           // current position of motor in degrees
float motorPosition_degR = 0.0;           // current position of motor in degrees

//I2C Stuff and states---------------------------------------------------------------------------------------------------

typedef enum State {
  IDLE,
  SCAN,
  ADJUST,
  MOVE,
  STOP,
  PAUSE
};

State currentState;

float angleError = 0; // Angle error, (-) value means marker is to left, (+) means marker to right
float distanceError = 0; // Distance error in centimeters, (+) error is distance to marker, - error should not happen

#define FOLLOWER_ADDRESS 0x04
byte data[32]; // buffer to store i2c message into

typedef union I2C_Packet_t {
  byte floatArrayNums[4];
  float floatNum;
};

const int BUSY = 11; //busy pin

//-----------------------------------------------------------------------------------------------------------------------

void setup() {

  // assign digital motor pins as outputs
  // note, encoder pins do not need to be set since the encoder.h library handles this
  pinMode(motorEnable, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M2PWM, OUTPUT);

  pinMode(BUSY, OUTPUT);
  
  currentState = IDLE;

  digitalWrite(motorEnable, 1);
  digitalWrite(BUSY, HIGH);
  
  analogWrite(M1PWM, 0);
  analogWrite(M2PWM, 0);

  // Set baud rate
  Serial.begin(115200);

  delay(500);
  Wire.begin(FOLLOWER_ADDRESS);
  Wire.onReceive(receiveData);

  Serial.println("init...");
}

// loop conditions to execute sequence once
bool customMovement = true;


int timer = 0;

unsigned long int lastTime = 0;


void loop() {

  timer += 1;

  digitalWrite(BUSY, 1);

  //Serial.println(currentState);
  switch(currentState) {
    
    case SCAN:
      Serial.print(millis());
      Serial.print(" - ");
      Serial.println(lastTime);
      Serial.println(millis()-lastTime);
      if (((millis() - lastTime) > 500) || ((millis() - lastTime) < 0)) {
        
        digitalWrite(BUSY, 0);
        rotateBot(45);
        digitalWrite(BUSY, 1);
        lastTime = millis();
      }

      digitalWrite(BUSY, 1);
      
    break;
    
    case ADJUST:
      //Serial.println(currentState);
     // Serial.println(angleError);
      digitalWrite(BUSY, 0);
      //Write Motors to 0
      analogWrite(M2PWM, 0);
      analogWrite(M1PWM, 0); 

      rotateBot(angleError);
      Serial.println("rotate done");
      currentState = PAUSE;
      Serial.println(currentState);
      digitalWrite(BUSY, 1);
    break;
    
    case MOVE:
      Serial.println(currentState);
      digitalWrite(BUSY, 0);
      //Write Motors to 0
      analogWrite(M2PWM, 0);
      analogWrite(M1PWM, 0); 

      forwardBot(distanceError);
      
      currentState = PAUSE;
      digitalWrite(BUSY, 1);
    break;
    case IDLE:
    case STOP:
    case PAUSE:
      digitalWrite(BUSY, 1);
      //Write Motors to 0
      analogWrite(M2PWM, 0);
      analogWrite(M1PWM, 0);
    break;
  }

}

float forwardBot(float x) {

  encoderL.write(0);
  encoderR.write(0);

  // PD controller variables
  const float Kp = 1.8;                       // proportional control
  const float Ki = 0.0;                      //seconds, PWM counts, degrees
  const float Kd = 20.0;                      //derivative control ms*PWM/degree
  int updateFrequency = 100;               // 0.1 seconds per update
  const int counts_per_rotation = 3200;    // the motor I took home was 3,200 but it should be 1,600 counts?
  unsigned long int previousMillis = millis();    // use to determine delta_t
  
  //PI Forward
  float errorF = 0.0;                       // in deg
  float previousErrorF = 0.0;
  int PI_pwmOutF = 0;                       // PWM control 0-255
  
  //Angle Correction
  float errorA = 0.0;                       // in deg
  float previousErrorA = 0.0;
  float integratorA = 0.0;                    // in PWM
  int PI_pwmOutA = 0;                       // PWM control 0-255
   
  const float endThresh = 5.0;  // allowable angular difference from set point allowed
  
  // reset motor position vals
  const float wheelCmm = (3.14159*7.6)/360;
  float forwardSetPosition_deg = x / wheelCmm;

  const float rotationConst = 4.85;
  float angleSetPosition_deg = 0;
  
  motorPosition_degL = 0.0;           // current position of motor in degrees
  motorPosition_degR = 0.0;           // current position of motor in degrees

  int count = 0;
  while (count < 10) {
    int delta_t = (millis() - previousMillis);  // time since last PI controller execution
    if (delta_t < 0) {previousMillis = millis();}
    // set previous millis to wait 0.1 seconds for next PI control loop
  
    // Control motor every 0.1 seconds to move as necessary
    if (delta_t >= updateFrequency) {
      previousMillis = millis();
      float fullRotation = 360.0;  // make calculations easier below
      
      // read current encoder position and convert to deg
      int encoderReadingL = encoderL.read();
      int encoderReadingR = -1 * encoderR.read();
      motorPosition_degL = ((float) encoderReadingL * fullRotation) / (float) counts_per_rotation;    
      motorPosition_degR = ((float) encoderReadingR * fullRotation) / (float) counts_per_rotation;

      //-----------FORWARD------------------
      //calculate error
      previousErrorF = errorF;
      errorF = forwardSetPosition_deg - (motorPosition_degL + motorPosition_degR);
      
      // calculate derivative error and determine PWM output to motor
      float errorDxF = (errorF - previousErrorF) / (float) delta_t;
      PI_pwmOutF = (Kp * errorF) + (Kd * errorDxF);  // PWM value to control motor speed

      //----------ANGLE-----------------
      //calculate error
      previousErrorA = errorA;
      errorA = angleSetPosition_deg - (motorPosition_degL - motorPosition_degR);
      
      // calculate derivative and integral error and determine PWM output to motor
      float errorDxA = (errorA - previousErrorA) / (float) delta_t;
      integratorA += Ki * errorA * (float) delta_t / 1000;
      integratorA = constrain(integratorA, -50, 50);  //prevent integrator windup.
      PI_pwmOutA = (Kp * errorA) + (Kd * errorDxA) + (Ki * integratorA);  // PWM value to control motor speed


      //--------WRITING------------
      int M2Out = constrain(PI_pwmOutF, -1*maxSpeed, maxSpeed) - PI_pwmOutA;
      int M1Out = constrain(PI_pwmOutF, -1*maxSpeed, maxSpeed) + PI_pwmOutA;
  
      if (M2Out > 0) {
        digitalWrite(M2DIR, HIGH);
      } else {
        digitalWrite(M2DIR, LOW);
      }
  
      if (M1Out > 0) {
        digitalWrite(M1DIR, HIGH);
      } else {
        digitalWrite(M1DIR, LOW);
      }
  
      //Write Motors
      analogWrite(M2PWM, constrain(abs(M2Out), 0, 255));
      analogWrite(M1PWM, constrain(abs(M1Out), 0, 255)); 

      if ((errorF < endThresh)) {
        count++;
      }
      
    }
    
  } // end while loop
  analogWrite(M1PWM, 0);
  analogWrite(M2PWM, 0);
  return errorA; // return to main loop

} // end function



float rotateBot(float phi) {
  Serial.println("Entered RotateBot");
  encoderL.write(0);
  encoderR.write(0);
  
  // PD controller variables
  const float Kp = 1.8;                       // proportional control
  const float Kd = 20.0;                      //derivative control ms*PWM/degree
  int updateFrequency = 100;               // 0.1 seconds per update
  const int counts_per_rotation = 3200;    // the motor I took home was 3,200 but it should be 1,600 counts?
  unsigned long int previousMillis = millis();    // use to determine delta_t
  
  //PI Forward
  float errorF = 0.0;                       // in deg
  float previousErrorF = 0.0;
  int PI_pwmOutF = 0;                       // PWM control 0-255
  
  //Angle Correction
  float errorA = 0.0;                       // in deg
  float previousErrorA = 0.0;
  int PI_pwmOutA = 0;                       // PWM control 0-255
   
  const float endThresh = 3.0;  // allowable angular difference from set point allowed
  
  // reset motor position vals
  const float wheelCmm = (3.14159*7.8)/360;
  float forwardSetPosition_deg = 0.0;

  const float rotationConst = 4.85;
  float angleSetPosition_deg = phi * rotationConst;
  
  motorPosition_degL = 0.0;           // current position of motor in degrees
  motorPosition_degR = 0.0;           // current position of motor in degrees

  int count = 0;
  int timeout = 0;
  while ((count < 10) && (timeout < 10)) {
    int delta_t = (millis() - previousMillis);  // time since last PI controller execution
    if (delta_t < 0) {previousMillis = millis();}
    // set previous millis to wait 0.1 seconds for next PI control loop
  
    // Control motor every 0.1 seconds to move as necessary
    if (delta_t >= updateFrequency) {
      previousMillis = millis();
      float fullRotation = 360.0;  // make calculations easier below
      
      // read current encoder position and convert to deg
      int encoderReadingL = encoderL.read();
      int encoderReadingR = -1 * encoderR.read();
      motorPosition_degL = ((float) encoderReadingL * fullRotation) / (float) counts_per_rotation;    
      motorPosition_degR = ((float) encoderReadingR * fullRotation) / (float) counts_per_rotation;

      //-----------FORWARD------------------
      //calculate error
      previousErrorF = errorF;
      errorF = forwardSetPosition_deg - (motorPosition_degL + motorPosition_degR);
      
      // calculate derivative error and determine PWM output to motor
      float errorDxF = (errorF - previousErrorF) / (float) delta_t;
      PI_pwmOutF = (Kp * errorF) + (Kd * errorDxF);  // PWM value to control motor speed

      //----------ANGLE-----------------
      //calculate error
      previousErrorA = errorA;
      errorA = angleSetPosition_deg - (motorPosition_degL - motorPosition_degR);

      if ((errorA-previousErrorA) < 1) {
        timeout++;
      }
      
      // calculate derivative error and determine PWM output to motor
      float errorDxA = (errorA - previousErrorA) / (float) delta_t;
      PI_pwmOutA = (Kp * errorA) + (Kd * errorDxA);  // PWM value to control motor speed


      //--------WRITING------------
      int M2Out = PI_pwmOutF - constrain(PI_pwmOutA, -1*maxSpeed, maxSpeed);
      int M1Out = PI_pwmOutF + constrain(PI_pwmOutA, -1*maxSpeed, maxSpeed);
  
      if (M2Out > 0) {
        digitalWrite(M2DIR, HIGH);
      } else {
        digitalWrite(M2DIR, LOW);
      }
  
      if (M1Out > 0) {
        digitalWrite(M1DIR, HIGH);
      } else {
        digitalWrite(M1DIR, LOW);
      }
  
      //Write Motors
      analogWrite(M2PWM, constrain(abs(M2Out), 0, 255));
      analogWrite(M1PWM, constrain(abs(M1Out), 0, 255));

      if ((errorA < endThresh)) {
        count++;
      }
      
    }
    
  } // end while loop
  analogWrite(M1PWM, 0);
  analogWrite(M2PWM, 0);
  return errorA; // return to main loop

} // end function

void receiveData(int byteCount) {
  int i = 0; //counter of num bytes recieved
  while(Wire.available()) { //continuously grab data
    data[i] = Wire.read();
    i++; //i iterates up to num bytes recieved
  }
  Serial.println("I2C");
  //Serial.println(i);
  int offset = data[0];
  switch(offset) {
    case 0:
      currentState = PAUSE;
    break;
    case 1:
      currentState = SCAN;
    break;
    case 2:
      //update angle error, figure out typdef union stuff
          {
      I2C_Packet_t pack;
      pack.floatArrayNums[0] = data[2];
      pack.floatArrayNums[1] = data[3];
      pack.floatArrayNums[2] = data[4];
      pack.floatArrayNums[3] = data[5];
      angleError = pack.floatNum;
      
      //Serial.println(angleError);
    }
    break;
    case 3:
      {
      I2C_Packet_t pack;
      pack.floatArrayNums[0] = data[2];
      pack.floatArrayNums[1] = data[3];
      pack.floatArrayNums[2] = data[4];
      pack.floatArrayNums[3] = data[5];
      //Serial.println(distanceError);
      distanceError = pack.floatNum;
    }
    //update distance error
    break;
    case 4:
      currentState = ADJUST; //angle error fix
    break;
    case 5:
      currentState = MOVE; // distance error fix
    break;
    case 6:
      currentState = STOP; // execution finished
    break;

  }
  Serial.println(currentState);

}
