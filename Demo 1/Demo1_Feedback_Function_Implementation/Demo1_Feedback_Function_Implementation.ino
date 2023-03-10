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

//I2C stuff

#define FOLLOWER_ADDRESS 0x04
int data[32]; // buffer to store i2c message into

typedef union I2C_Packet_t {
  byte floatArrayNums[4];
  float floatNum;
} ;

void setup() {

  // assign digital motor pins as outputs
  // note, encoder pins do not need to be set since the encoder.h library handles this
  pinMode(motorEnable, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M2PWM, OUTPUT);

  Wire.begin(FOLLOWER_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  digitalWrite(motorEnable, 1);

  // Set baud rate
  Serial.begin(115200);

  delay(100);

  Serial.println("forward,angle");
}

// loop conditions to execute sequence once
bool customMovement = true;

void loop() {

  // execute custom sequence of forward and rotational movements
  if (customMovement) {
    forwardMoveBot(100.0);
    delay(200);
    rotateBot(180.0);
    delay(200);
    forwardMoveBot(100.0);
    customMovement == false;
  }

  //fsm implementation? maybe for later demo?
}

/**
 * This function moves the robot forward to a set distance in cm
 *
 * @param distance_cm Set the distance desired in cm
 */
void forwardMoveBot(float distance_cm) {

  // reset encoder counts to zero out relative starting position
  // reset bool var that checks if function completed
  encoderL.write(0);
  encoderR.write(0);
  isForwardMoveDone = 0;
  previousMillis = 0;

  // set position calculation
  // reset motor position vals
  const float wheelCmm = (2*3.14159*7.75)/360;    //circumference of our wheels in cm/deg.
  float motorSetPosition_deg = distance_cm / wheelCmm;        // desired set position based on aruco position, in deg
  motorPosition_degL = 0.0;                 // current position of motor in degrees
  motorPosition_degR = 0.0;                 // current position of motor in degrees
  
  // PI controller variables
  const int Kp = 1.0;                       // proportional control
  const float Ki = 0;                       // integrator control
  int updateFrequency = 100;                // 0.1 seconds per PI update

  // Angle Correction
  float errorA = 0.0;                       // in deg
  float KpA = 0.5;
  int PI_pwmOutA = 0;                       // PWM control 0-255

  // PI Forward
  float errorF = 0.0;                       // in deg
  float integralErrorF = 0.0;               // in deg * seconds
  int PI_pwmOutF = 0;                       // PWM control 0-255
 
  while (isForwardMoveDone == false) {
    int delta_t = (millis() - previousMillis);  // time since last PI controller execution
    
    // Control motor every 0.1 seconds to move as necessary
    if (delta_t >= updateFrequency) {
      previousMillis = millis();
      float fullRotation = 360.0;  // make calculations easier below

      // read current encoder position and convert to deg
      int encoderReadingL = encoderL.read();   
      motorPosition_degL = ((float) encoderReadingL * fullRotation) / (float) counts_per_rotation;

      // Other Motor
      int encoderReadingR = -1 * encoderR.read();   
      motorPosition_degR = ((float) encoderReadingR * fullRotation) / (float) counts_per_rotation;

      //Calculate Error and record
      float errorF = motorSetPosition_deg - motorPosition_degL;

      // calculate integral error and determine PWM output to motor
      integralErrorF += errorF * ((float) delta_t / 1000.0);        // assuming delta_t is calculated in seconds, gives deg * sec
      PI_pwmOutF = (Kp * errorF) + (Ki * integralErrorF);  // PWM value to control motor speed

      //Angle Correction
      float errorA = motorPosition_degL - motorPosition_degR;
      PI_pwmOutA = KpA * errorA;

      int M2Out = PI_pwmOutF + PI_pwmOutA;
      int M1Out = PI_pwmOutF - PI_pwmOutA;

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
      
      //if motor reached setpoint, within threshold,turn off
      if ((PI_pwmOutF < 5)&&(PI_pwmOutA < 5)) {
        analogWrite(M1PWM, 0);
        analogWrite(M2PWM, 0);
        integralErrorF = 0.0;           // reset integral error
        isForwardMoveDone = true;
      }  
    } // end PI if statement

    if (Serial.available()) {
      motorSetPosition_deg = Serial.parseFloat();
    }

    Serial.print(PI_pwmOutF);
    Serial.print(",");
    Serial.println(PI_pwmOutA);
    } // end while loop

    return; // return to main loop
}

/**
 * This function rotates the robot to a set phi in degrees
 *
 * @param setDegreesRot Set the angle phi (pos or neg) to rotate
 */
void rotateBot(float setDegreesRot) {

  /* *!* delete me before submitting *!*
  Idea: Rotate both wheels at aprox same velocity until desired rotation achieved
   */

  // reset encoder counts to zero out relative starting position
  // reset bool var that checks if function completed
  encoderL.write(0);
  encoderR.write(0);
  isRotationDone = 0;
  previousMillis = 0;

  //PI Forward
  float errorF = 0.0;                       // in deg
  float integralErrorF = 0.0;               // in deg * seconds
  int PI_pwmOutF = 0;                       // PWM control 0-255
  
  //Angle Correction
  float errorA = 0.0;                       // in deg
  float KpA = 0;
  int PI_pwmOutA = 0;                       // PWM control 0-255

  // PI controller variables
  const float Kp = 3.4;                     // proportional control
  const float Ki = 3.0;                     // integrator control
  int updateFrequency = 50;                 // 0.05 seconds per update

  // PID controll for velocity
  // checking to make sure that encoder counts are stepping together (or actually calcualte velocity?)
  const float KpV = 0.0;
  const float KiV = 0.0;
  const float KdV = 0.0;
  
  // set desired rotation encoder position
  //float motorSetPosition_deg = (setDegreesRot * counts_per_rot) * (360.0 / counts_per_rotation);
  // reset motor position vals
  const float rotationConst = 2.3;
  float motorSetPosition_deg = setDegreesRot * rotationConst;
  motorPosition_degL = 0.0;           // current position of motor in degrees
  motorPosition_degR = 0.0;           // current position of motor in degrees

  while (isRotationDone == false) {

    // execute every update frequency
    int delta_t = millis() - previousMillis;
    if (delta_t >= updateFrequency) {

      previousMillis = millis();
      float fullRotation = 360.0;  // make calculations easier below

      // read current encoder position and convert to deg
      int encoderReadingL = encoderL.read();   
      motorPosition_degL = ((float) encoderReadingL * fullRotation) / (float) counts_per_rotation;

      // Other Motor
      int encoderReadingR = -1 * encoderR.read();   
      motorPosition_degR = ((float) encoderReadingR * fullRotation) / (float) counts_per_rotation;
      
      //float errorF = 0.0;
      if (motorSetPosition_deg >= 0) {
        errorF = motorSetPosition_deg - motorPosition_degL;
      } else {
        errorF = motorSetPosition_deg + motorPosition_degR;
      } 
      //Calculate Error and record
      //errorF = motorSetPosition_deg + motorPosition_degR;

      // calculate integral error and determine PWM output to motor
      integralErrorF += Ki*errorF * ((float) delta_t / 500.0);        // assuming delta_t is calculated in seconds, gives deg * sec
      integralErrorF = constrain(integralErrorF, -30, 30);
      PI_pwmOutF = (Kp * errorF) + (integralErrorF);  // PWM value to control motor speed

      //Angle Correction
      float errorA = motorPosition_degL + motorPosition_degR;
      PI_pwmOutA = KpA * errorA;

      // velocity correction with derivative control
      //float errorV = motorPosition_degL + motorPosition_degR;
      //PI_pwmOutV = KpV * errorV;

      int M2Out = -1*PI_pwmOutF;
      int M1Out = PI_pwmOutF;

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
      
      //if motor reached setpoint, within threshold,turn off
      if ((PI_pwmOutF < 5)&&(PI_pwmOutA < 5)) {
        analogWrite(M1PWM, 0);
        analogWrite(M2PWM, 0);
        integralErrorF = 0.0;           // reset integral error
        isRotationDone = true;
      }  

      Serial.print(errorF);
      Serial.print(",");
      Serial.println(integralErrorF);
    }  // end PI if statement
    
  } // end while loop
  return; // return to main loop

} // end function

void receiveData(int byteCount) {
  int i = 0; //counter of num bytes recieved
  while(Wire.available()) { //continuously grab data
    data[i] = Wire.read();
    i++; //i iterates up to num bytes recieved
  }
  if (i == 1) { //data was sent to read, 1 is read offset instruction
    //no need to do anything, just ignore
  } else {
    //place code to update aruco position here
    arucoPosition = data[1]; // since offset sent first, arucoPosition in data[1]
    //Serial.println(arucoPosition);
  }

}

void sendData() {
  I2C_Packet_t tempPacket;
  tempPacket.floatNum = motorPosition_degL;
  Wire.write(tempPacket.floatArrayNums, 4);

}