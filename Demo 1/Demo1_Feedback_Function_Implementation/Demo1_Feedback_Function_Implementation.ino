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
const int maxSpeed = 128;                 // limit maximum motor speed
float motorPosition_degL = 0.0;           // current position of motor in degrees
float motorPosition_degR = 0.0;           // current position of motor in degrees



void setup() {

  // assign digital motor pins as outputs
  // note, encoder pins do not need to be set since the encoder.h library handles this
  pinMode(motorEnable, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M2PWM, OUTPUT);

  digitalWrite(motorEnable, 1);

  // Set baud rate
  Serial.begin(115200);

  delay(100);

  Serial.println("init...");
}

// loop conditions to execute sequence once
bool customMovement = true;

void loop() {

  // execute custom sequence of forward and rotational movements
  if (customMovement) {
    Serial.println(rotateBot(90));
    delay(200);
    //forwardBot(213.36);
    Serial.println("Done");
    customMovement = false;
  }

  //fsm implementation? maybe for later demo?
}

float forwardBot(float x) {

  encoderL.write(0);
  encoderR.write(0);

  // PD controller variables
  const float Kp = 1.8;                       // proportional control
  const float Kd = 20.0;                      //derivative control ms*PWM/degree
  int updateFrequency = 100;               // 0.1 seconds per update
  const int counts_per_rotation = 3200;    // the motor I took home was 3,200 but it should be 1,600 counts?
  unsigned long int previousMillis = 0;    // use to determine delta_t
  
  //PI Forward
  float errorF = 0.0;                       // in deg
  float previousErrorF = 0.0;
  int PI_pwmOutF = 0;                       // PWM control 0-255
  
  //Angle Correction
  float errorA = 0.0;                       // in deg
  float previousErrorA = 0.0;
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
      
      // calculate derivative error and determine PWM output to motor
      float errorDxA = (errorA - previousErrorA) / (float) delta_t;
      PI_pwmOutA = (Kp * errorA) + (Kd * errorDxA);  // PWM value to control motor speed

      // Encoder count check derivative contorl
      float errorCounts = motorPosition_degL - motorPosition_degR;
      float PI_pwmOutV = Kd * errorCounts;


      //--------WRITING------------
      int M2Out = PI_pwmOutF - PI_pwmOutA;
      int M1Out = PI_pwmOutF + PI_pwmOutA;

      /*
      if (motorPosition_degL > motorPosition_degR + 5) {
        M1Out = M1Out + PI_pwmOutV;      
      } else if (motorPosition_degR > motorPosition_degL + 5) {
        M2Out = M2Out + PI_pwmOutV;
      }
      */
      
      
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
      analogWrite(M2PWM, constrain(abs(M2Out), 0, maxSpeed));
      analogWrite(M1PWM, constrain(abs(M1Out), 0, maxSpeed));

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

  encoderL.write(0);
  encoderR.write(0);

  // PD controller variables
  const float Kp = 1.8;                       // proportional control
  const float Kd = 20.0;                      //derivative control ms*PWM/degree
  int updateFrequency = 100;               // 0.1 seconds per update
  const int counts_per_rotation = 3200;    // the motor I took home was 3,200 but it should be 1,600 counts?
  unsigned long int previousMillis = 0;    // use to determine delta_t
  
  //PI Forward
  float errorF = 0.0;                       // in deg
  float previousErrorF = 0.0;
  int PI_pwmOutF = 0;                       // PWM control 0-255
  
  //Angle Correction
  float errorA = 0.0;                       // in deg
  float previousErrorA = 0.0;
  int PI_pwmOutA = 0;                       // PWM control 0-255
   
  const float endThresh = 5.0;  // allowable angular difference from set point allowed
  
  // reset motor position vals
  const float wheelCmm = (3.14159*7.6)/360;
  float forwardSetPosition_deg = 0.0;

  const float rotationConst = 4.85;
  float angleSetPosition_deg = phi * rotationConst;
  
  motorPosition_degL = 0.0;           // current position of motor in degrees
  motorPosition_degR = 0.0;           // current position of motor in degrees

  int count = 0;
  while (count < 10) {
    int delta_t = (millis() - previousMillis);  // time since last PI controller execution
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
      
      // calculate derivative error and determine PWM output to motor
      float errorDxA = (errorA - previousErrorA) / (float) delta_t;
      PI_pwmOutA = (Kp * errorA) + (Kd * errorDxA);  // PWM value to control motor speed


      //--------WRITING------------
      int M2Out = PI_pwmOutF - PI_pwmOutA;
      int M1Out = PI_pwmOutF + PI_pwmOutA;
  
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
      analogWrite(M2PWM, constrain(abs(M2Out), 0, maxSpeed));
      analogWrite(M1PWM, constrain(abs(M1Out), 0, maxSpeed));

      if ((errorA < endThresh)) {
        count++;
      }
      
    }
    
  } // end while loop
  analogWrite(M1PWM, 0);
  analogWrite(M2PWM, 0);
  return errorA; // return to main loop

} // end function
