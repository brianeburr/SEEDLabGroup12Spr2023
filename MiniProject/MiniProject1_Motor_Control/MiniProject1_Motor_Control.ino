/*
 * Mini Project 1
 * Filename: MiniProject1_Motor_Control.ino
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
 * Creator: Sean West
 * Date: 02/21/2023
 *
*/

// this library tracks the encoder better than my code, so I am using it here
#include <Encoder.h>

 // pin connections
 const int encoderPinA = 2;   // CLK pin
 const int encoderPinB = 5;   // DAT pin
 const int motorEnable = 4;   // Tri-state di sable both motor chanels when LOW (enable)
 const int M1DIR = 7;         // Motor polarity 1 (direction)
 const int M1PWM = 9;         // Motor volt 1 "speed"
 //const int nSF = 12;          // Status flag indicator, dont think we need this?
 
 // setup encoder class to be used
 Encoder encoder(encoderPinA, encoderPinB);

 // PI controller variables
 float error = 0.0;                       // in rad
 float integralError = 0.0;               // in rad * seconds
 int PI_pwmOut = 0;                       // PWM controll 0-255
 const int Kp = 98;                       // proportional controll
 const float Ki = 7.22;                   // integrator controll
 int updateFrequency = 100;               // 0.1 seconds per update
 const int counts_per_rotation = 3200;    // the motor I took home was 3,200 but it should be 1,600 counts
 unsigned long int previousMillis = 0;    // use to determine delta_t
 
 // Positional variables
 int motorDirection = 0;                  // 1 is CCW, -1 is CW
 float motorPosition_rad = 0.0;           // current position of motor in radians
 int arucoPosition = 1;                   // aruco position: 0,1,2, or 3
 float motorSetPosition_rad = 0.0;        // desired set position based on aruco position
 float previousSetPosition_rad = 0.0;     // account for last aruco set position
 const float motorSetPositionThreshold_rad = 5.0 * (PI / 180.0);  // allowable angular difference from set point allowed

void setup() {

  // assign digital motor pins as outputs
  // note, encoder pins do not need to be set since the encoder.h library handles this
  pinMode(motorEnable, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);

  // Set baud rate
  Serial.begin(115200);
}

void loop() {
  motorSetPosition_rad = arucoPosition * (PI / 2.0);  // determine set position in radians

  // Determine if new aruco position
  if (motorSetPosition_rad != previousSetPosition_rad) {
    integralError = 0.0;  // if set position changes, reset integral error
    previousSetPosition_rad = motorSetPosition_rad;
  }

  int delta_t = (millis() - previousMillis);  // time since last PI controller execution

  // Controll motor every 0.1 seconds to move as necessary
  if (delta_t >= updateFrequency) {

    float fullRotation = 2.0 * PI;  // make calculations easier below

    // read current encoder position and convert to rad
    motorPosition_rad = ((float) encoder.read() * fullRotation) / (float) counts_per_rotation;
    
    // Calculate error and determine PWM output
    error = motorSetPosition_rad - fmod(motorPosition_rad, fullRotation);   // normalize position with fmod if >360 or <-360
    integralError += error * ((float) delta_t / 1000.0);                    // assuming delta_t is calculated in seconds, gives rad * sec
    PI_pwmOut = (Kp * error) + (Ki * integralError);                        // PWM value to control motor speed

    // Make sure PWM value is within allowable range 0-255
    // pwm values going negative, had to use abs() to constrain to positive vals
    PI_pwmOut = constrain(abs(PI_pwmOut), 0, 255);   

    // write PWM signal to motor to change speed
    analogWrite(M1PWM, PI_pwmOut);

    // Determine if CCW or CW movement is shortest path to set point
    // (destination - source + 360) % 360 > 180, move CCW
    if (fmod(motorSetPosition_rad - fmod(motorPosition_rad, fullRotation) + fullRotation, fullRotation) > PI) {
      digitalWrite(M1DIR, HIGH);    // rotate CCW
      motorDirection = 1;
    } else {
      digitalWrite(M1DIR, LOW);     // rotate CW
      motorDirection = -1;
    }

    // if motor reached setpoint, within threshold,turn off
    if (abs(error) <= motorSetPositionThreshold_rad) {
      analogWrite(motorEnable, 0);  // turn motor OFF
      integralError = 0.0;          // reset integral error
      motorDirection = 0;           // motor not moving
    } else {
      analogWrite(motorEnable, 1);  // keep/turn motor ON
    }

    // set previous millis
    // include here to ensure that time taken to execute code below is accounted for (might be longer than 100ms to print data)
    previousMillis = millis();

    // Test information print to serial port
    // !*! delete/comment before demo
    Serial.print(motorPosition_rad);
    Serial.print(", ");
    Serial.print(motorDirection);
    Serial.print(", ");
    Serial.print(PI_pwmOut);
    Serial.println();
    
  }
}