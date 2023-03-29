#include <Wire.h>
#include <Encoder.h>

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


float motorPosition_degL = 0.0;           // current position of motor in degrees
float motorPosition_degR = 0.0;           // current position of motor in degrees
const int maxSpeed = 70;

#define FOLLOWER_ADDRESS 0x04
int data[32]; // buffer to store i2c message into

typedef union I2C_Packet_t {
  byte floatArrayNums[4];
  float floatNum;
};

void setup() {

  currentState = IDLE;
  Wire.begin(FOLLOWER_ADDRESS);
  Wire.onReceive(receiveData);

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

  currentState = SCAN;
}

int loops = 0;

void loop() {
  // put your main code here, to run repeatedly:
    

  if (loops >= 20) {
      currentState = IDLE;
    }


  switch(currentState) {
    case IDLE:
    analogWrite(M2PWM, 0);
    analogWrite(M2PWM, 0);
    

    break;
    case SCAN:

    const int maxSpeed = 70;

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
    float angleSetPosition_deg = 360.0;
    
    motorPosition_degL = 0.0;           // current position of motor in degrees
    motorPosition_degR = 0.0;           // current position of motor in degrees

    int count = 0;

    // Typically would loop the following, but leaving it open to keep it dynamic
    // so that it doesnt get stuck doing something

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
      loops++;
    } // end if    

    encoderL.write(0);
    encoderR.write(0);

    

    


      break;
    case ADJUST:

      currentState = PAUSE;
    break;
    case MOVE:

      currentState = PAUSE;
    break;
    case STOP:

    break;
    case PAUSE:

    break;
  }
}

void receiveData(int byteCount) {
  int i = 0; //counter of num bytes recieved
  while(Wire.available()) { //continuously grab data
    data[i] = Wire.read();
    i++; //i iterates up to num bytes recieved
  }
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
    break;
    case 3:
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
}
