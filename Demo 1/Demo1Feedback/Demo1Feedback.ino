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
 const int motorEnable = 4;   // Tri-state di sable both motor chanels when LOW (enable)
 const int M1DIR = 7;         // Motor polarity 1 (direction)
 const int M1PWM = 9;         // Motor volt 1 "speed"
 
 // setup encoder class to be used
 Encoder encoder(encoderPinA, encoderPinB);

 // PI controller variables
 float error = 0.0;                       // in rad
 float integralError = 0.0;               // in rad * seconds
 int PI_pwmOut = 0;                       // PWM control 0-255
 const int Kp = 98;                       // proportional control
 const float Ki = 7.22;                   // integrator control
 int updateFrequency = 100;               // 0.1 seconds per update
 const int counts_per_rotation = 3200;    // the motor I took home was 3,200 but it should be 1,600 counts?
 unsigned long int previousMillis = 0;    // use to determine delta_t
 
 // positional variables
 int motorDirection = 0;                  // test variable to determine motor direction: 1 CCW, -1 CW, 0 not moving
 float motorPosition_rad = 0.0;           // current position of motor in radians
 int arucoPosition = 2;                   // aruco position: 0,1,2, or 3
 float motorSetPosition_rad = 0.0;        // desired set position based on aruco position, in rad
 float previousSetPosition_rad = 0.0;     // account for last aruco set position
 const float motorSetPositionThreshold_rad = 5.0 * (PI / 180.0);  // allowable angular difference from set point allowed

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

  Wire.begin(FOLLOWER_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  digitalWrite(motorEnable, 1);

  // Set baud rate
  Serial.begin(115200);
}

void loop() {
  motorSetPosition_rad = arucoPosition * (PI / 2.0);  // determine set position in radians

  // Determine if new aruco position
  // if set position changes, reset integral error
  if (motorSetPosition_rad != previousSetPosition_rad) {
    integralError = 0.0;
    previousSetPosition_rad = motorSetPosition_rad;
  }

  int delta_t = (millis() - previousMillis);  // time since last PI controller execution

  // Control motor every 0.1 seconds to move as necessary
  if (delta_t >= updateFrequency) {

    float fullRotation = 2.0 * PI;  // make calculations easier below

    // handle the case where encoder moves from pos to neg counts
    // normalizes encoder counts to 0 - counts_per_rotation
    // make sure motor position is within 0 - 359
    int encoderReading = encoder.read();
    while (encoderReading < 0) {
        encoderReading += counts_per_rotation;
    }
    while (encoderReading >= counts_per_rotation) {
        encoderReading -= counts_per_rotation;
    }    

    // read current encoder position and convert to rad
    motorPosition_rad = ((float) encoderReading * fullRotation) / (float) counts_per_rotation;
    
    // Calculate error based on shortest path to set position general calculations below
    // errorCCW = (setAngle - currentAngle + 180) % 360 - 180
    // errorCW = (currentAngle - setAngle + 180) % 360 - 180
    // !NOTE! can swap CW and CCW variable names to change sign associated with error (like when we add another wheel) 
    // currently set as desired with motor shaft pointed at you
    float errorCW = fmod(motorSetPosition_rad - motorPosition_rad + PI, fullRotation) - PI;
    float errorCCW = fmod(motorPosition_rad - motorSetPosition_rad + PI, fullRotation) - PI;
    float previousError = error;

    if (abs(errorCW) < abs(errorCCW)) {
      error = -1 * errorCW;
    } else {
      error = errorCCW;
    }

    // reset integralError if rotated past 180 degrees, changing the sign of the error term
    // otherwise will have to overcome integral error buildup
    if (error * previousError < 0) {
      integralError = 0;
    }

    // calculate integral error and determine PWM output to motor
    integralError += error * ((float) delta_t / 1000.0);        // assuming delta_t is calculated in seconds, gives rad * sec
    PI_pwmOut = (Kp * abs(error)) + (Ki * abs(integralError));  // PWM value to control motor speed

    // Make sure PWM value is within allowable range 0-255
    // calculated pwm values can be negative, use abs() to constrain to positive vals, direction already accounted for
    PI_pwmOut = constrain(abs(PI_pwmOut), 0, 255);   

    // write PWM signal to motor to change speed
    analogWrite(M1PWM, PI_pwmOut);

    // Determine if CCW or CW movement is shortest path to set point
    // *!* move inside where error is calculated? needs testing  
    if (error > 0) {
      digitalWrite(M1DIR, HIGH);     // rotate CCW
      motorDirection = 1;
    } else {
      digitalWrite(M1DIR, LOW);      // rotate CW
      motorDirection = -1;
    }

    // if motor reached setpoint, within threshold,turn off
    if (abs(error) <= motorSetPositionThreshold_rad) {
      digitalWrite(motorEnable, 0);  // turn motor OFF
      integralError = 0.0;           // reset integral error
      motorDirection = 0;            // motor not moving
    } else {
      digitalWrite(motorEnable, 1);  // keep/turn motor ON
    }

    // set previous millis to wait 0.1 seconds for next PI control loop
    previousMillis = millis();
  }
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
  tempPacket.floatNum = motorPosition_rad;
  Wire.write(tempPacket.floatArrayNums, 4);

}

