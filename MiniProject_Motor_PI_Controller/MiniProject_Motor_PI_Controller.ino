/*
 * File: MiniProject_Motor_PI_Controller.ino
 * Author: Sean West
 * Description: This program will read two encoders and will be able to update positional data (x,y,phi) and determine how fast each encoder is rotating
 * Date Created: February 5, 2023
 * Last Modified: February 6, 2023
 *
 * Note: I am using the notation where 1 = left and 2 = right. This was an easier naming convention than 'l' and 'r'
 */

// Constants
const int encoder1_clk_pin = 2;
const int encoder1_dt_pin = 4;
const int counts_per_rotation = 1600;
const double wheelRadius_m = 0.05;      // wheel radius in meters
const unsigned long interval = 5;       // interval used to wait before potentially printing zero velocity to serial port
const float pi = 3.1415926535897932384626433832795;

// Motor Pins
const int pinEnable = 4; // Tri-state di sable both motor chanels when LOW (enable)
const int M1DIR = 7;     // Motor polarity 1
const int M2DIR = 8;     // Motor polarity 2
const int M1PWM = 9;     // Motor volt 1 "speed"
const int M2PWM = 10;    // Motor Volt 2
const int nSF = 12;      // Status flag indicator

// Global variables
int encoder1_count = 0;
int encoder2_count = 0;

volatile double v1 = 0.0;                          // Velocity encoder 1
volatile double v2 = 0.0;                          // Velocity encoder 2
volatile bool encoder1_newData = false;   // Encoder 1 interrupt flag
volatile bool encoder2_newData = false;   // Encoder 2 interrupt flag
unsigned long previousMillis1 = 0;        // Keep track of time for encoder 1  
unsigned long previousMillis2 = 0;        // Keep track of time for encoder 2

// Struct to store encoder position and time
struct EncoderData {
  double position = 0.0;
  unsigned long time = 0;
  double phi_dot = 0.0;
};

// Create a structs to store current and previous values for encoders 1 and 2
EncoderData Encoder1_Previous;
EncoderData Encoder1_Current;
EncoderData Encoder2_Previous;
EncoderData Encoder2_Current;

// Struct to store positional data
struct Position {
  double x = 0.0;
  double y = 0.0;
  double phi = 0.0;
  unsigned long int time = 0;
};

// Create structs to store positional data
Position Position_Old;
Position Position_New;

void setup() {
  // Set the pin modes for the encoder CLK and DT pins
  pinMode(encoder1_clk_pin, INPUT_PULLUP);
  pinMode(encoder1_dt_pin, INPUT_PULLUP);
  pinMode(encoder2_clk_pin, INPUT_PULLUP);
  pinMode(encoder2_dt_pin, INPUT_PULLUP);

  // Attach the interrupts for the encoder CLK pins
  attachInterrupt(digitalPinToInterrupt(encoder1_clk_pin), encoder_1_update, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2_clk_pin), encoder_2_update, CHANGE);

  Serial.begin(115200);
}

void loop() {
  unsigned long currentMillis = millis();
  double time_seconds;

  // If no new updates from encoder 1 or 2 in a while, set velocity to zero and print updated values
  if ((currentMillis - Encoder1_Previous.time >= 500) || (currentMillis - Encoder2_Previous.time >= 500) || encoder1_newData || encoder2_newData) {

    // Set encoder 1 velocity to zero and reset its timer

    if (currentMillis - Encoder1_Previous.time >= 500) {
      v1 = 0.0;
      Encoder1_Previous.time = currentMillis;
    }
    
    if (currentMillis - Encoder2_Previous.time >= 500) {
      v2 = 0.0;
      Encoder2_Previous.time = currentMillis;
    }

    if (encoder1_newData) encoder1_newData = false;
    if (encoder2_newData) encoder2_newData = false;

    // Calculate x, y, and phi position and orientation
    // !*!
    // Move these calculations to ISR for more accurate positional data
    // !*!
    currentMillis = millis();
    double delta_seconds = (double) (currentMillis - Position_Old.time) / 1000.0;

    Position_New.x = Position_Old.x + (delta_seconds * cos(Position_Old.phi) * (v1 + v2) / 2.0);
    Position_New.y = Position_Old.y + (delta_seconds * sin(Position_Old.phi) * (v1 + v2) / 2.0);
    Position_New.phi = Position_Old.phi + (delta_seconds * (1 / b) * (v1 - v2));
    
    time_seconds = (double)millis() / 1000.0;
    Serial.print(time_seconds, 3);
    Serial.print(",");
    Serial.print(v1, 3);
    Serial.print(",");
    Serial.print(v2, 3);
    Serial.print(",");
    Serial.print(Position_New.x, 5);
    Serial.print(",");
    Serial.print(Position_New.y, 5);
    Serial.print(",");
    Serial.print(Position_New.phi, 5);
    Serial.println();

    // Update previous position data after printing current values
    // !*!
    // Move these calculations to ISR for more accurate positional data
    // !*!
    Position_Old.phi = Position_New.phi;
    Position_Old.x = Position_New.x;
    Position_Old.y = Position_New.y;
    Position_Old.time = currentMillis;    
  }
  
}

// Interrupt function for the first encoder
void encoder_1_update() {

  int A = digitalRead(encoder1_clk_pin);
  int B = digitalRead(encoder1_dt_pin);

  // Determine if CW or CCW rotation
  if (A == B) {
    encoder1_count++;
  }
  else {
    encoder1_count--;
  }

  // Update values for current and previous time and position for encoder 1
  Encoder1_Current.time = millis();
  Encoder1_Current.position = (double) encoder1_count * (2.0 * pi) / (double) counts_per_rotation;

  // Determine new position
  Position_New.phi = Position_Old.phi + (delta_seconds * (1 / b) * v1);

  // Calculate the velocity of the encoder
  float delta_time_s = (double)(Encoder1_Current.time - Encoder1_Previous.time) / 1000.0;
  Encoder1_Current.phi_dot = (double) (Encoder1_Current.position - Encoder1_Previous.position) / (double) (delta_time_s);
  v1 = wheelRadius_m * Encoder1_Current.phi_dot;


  // Shift current values into previous values
  Encoder1_Previous.time = Encoder1_Current.time;
  Encoder1_Previous.position = Encoder1_Current.position;

  Position_Old.phi = Position_New.phi;
  Position_Old.time = Encoder1_Current.time;    

  // set ISR flag
  encoder1_newData = true;
}