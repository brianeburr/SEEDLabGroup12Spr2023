/*
 * Mini Project 1
 * Filename: MiniProject1_Motor_Control_Final.ino
 *
 * Purpose: Move a DC motor with an encoder to a set position based on the location
            of an aruco marker that is located with a Raspberry Pi + camera.
            The variable that contain the position of the aruco will be updated via I2C.
            A PI controller was implemented to set a reasonable speed to move the wheel 
            to a desired location based on the aruco marker (quadrants 0-3).
            
            A shortest path algorithm was used to ensure that the wheel does not take
            longer than necessary to reach desired set point. The algorithm determines
            whether clockwise or counter-clockwise rotation provides the shortest path
            and sets the movement of the wheel in the appropriate direction.
 * Class: EENG350, SEED Lab
 * Creator: Sean West (PI motor control)
 * Co-Creator: Brian Burr (I2C)
 * Date: 02/26/2023
 *
 */

// include libraries
#include <Encoder.h>
#include <Wire.h>

 // pin connections
 const int encoderPinA = 2;   // CLK pin
 const int encoderPinB = 5;   // DAT pin

 const int encoderPinC = 3;  // CLK encoder 2
 const int encoderPinD = 6;  // DAT
 
 const int motorEnable = 4;   // Tri-state di sable both motor chanels when LOW (enable)
 const int M1DIR = 7;         // Motor polarity 1 (direction)
 const int M1PWM = 9;         // Motor volt 1 "speed"
 const int M2DIR = 8;
 const int M2PWM = 10;
 
 // setup encoder class to be used
 Encoder encoderL(encoderPinA, encoderPinB);
 Encoder encoderR(encoderPinC, encoderPinD);

 // PI controller variables

 const int Kp = 1.0;                       // proportional control
 const float Ki = 0;                 // integrator control
 int updateFrequency = 100;               // 0.1 seconds per update
 const int counts_per_rotation = 3200;    // the motor I took home was 3,200 but it should be 1,600 counts?
 unsigned long int previousMillis = 0;    // use to determine delta_t

 //PI Forward
 float errorF = 0.0;                       // in deg
 float integralErrorF = 0.0;               // in deg * seconds
 int PI_pwmOutF = 0;                       // PWM control 0-255

float motorPosition_degL = 0.0;           // current position of motor in degrees

 //Angle Correction
 float errorA = 0.0;                       // in deg
 float KpA = 0.5;
 int PI_pwmOutA = 0;                       // PWM control 0-255
 
 float motorPosition_degR = 0.0;           // current position of motor in deg
 
 // positional variables
 
 int arucoPosition = 2;                   // aruco position: 0,1,2, or 3
 float wheelCmm = (2*3.14159*7.75)/360;          //circumference of our wheels in cm/deg.
 float motorSetPosition_deg = 100 / wheelCmm;        // desired set position based on aruco position, in deg
 float previousSetPosition_deg = 0.0;     // account for last aruco set position
 const float motorSetPositionThreshold_deg = 5.0;  // allowable angular difference from set point allowed

const int maxSpeed = 128; //maximum motor speed 0-255

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

void loop() {

  int delta_t = (millis() - previousMillis);  // time since last PI controller execution
  // set previous millis to wait 0.1 seconds for next PI control loop
  

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

    /*
    //if motor reached setpoint, within threshold,turn off
    if ((PI_pwmOutF < 5)&&(PI_pwmOutA < 5)) {
      analogWrite(M1PWM, 0);
      analogWrite(M2PWM, 0);
      integralErrorF = 0.0;           // reset integral erro
    }  
    */
    
  }

  if (Serial.available()) {
    motorSetPosition_deg = Serial.parseFloat();
  }

  
  Serial.print(PI_pwmOutF);
  Serial.print(",");
  Serial.println(PI_pwmOutA);
}

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
