/*
 * Mini Project
 *
 * Purpose: STEP Response Experiment to make motor model 
 * Class: EENG350, SEED Lab
 * Creator: Sean West and Connor Denney
 * Date: 02/17/2023
 *
*/

// define pins used in program
#define CLK_1 2  //clock pin encoder 1
#define DT_1 5   //data pin encoder 1

#define Motor1 9
#define Motor1Dir 7
#define MotorEnable 4

#define SAMPLES 100

void updateData1();

void setup() {
  // assign pins
  pinMode(DT_1, INPUT_PULLUP);
  pinMode(CLK_1, INPUT_PULLUP);

  pinMode(Motor1, OUTPUT);
  pinMode(Motor1Dir, OUTPUT);

  pinMode(MotorEnable, OUTPUT);
  delay(5);
  digitalWrite(MotorEnable, HIGH);
  
  // interrupt on CLK_1 rising
  attachInterrupt(digitalPinToInterrupt(CLK_1), updateData1, CHANGE);

  Serial.begin(115200);

  digitalWrite(Motor1Dir, LOW);
  while(!Serial.available());
  Serial.read();
}

// Declare variables that will change in loop()
int angularPos1 = 0;
bool newData1 = false;
unsigned long previousTime1 = millis();
unsigned long interval = 10; //ms

int countsPerRevolution = 1600; //this is valid for a 50:1 gear ratio and 32 counts per encoder revolution.

int i = 0;



void loop() {

  if( (millis() - previousTime1) >= interval) {
    previousTime1 = millis();
    Serial.print(angularPos1);
    Serial.print(",");
    Serial.println(millis());
    i++;
  }

  if (i>5) {
    analogWrite(Motor1, 128);
  }
  
  if (i > SAMPLES) {
    i = 0;
    //angularPos1 = 0;
    analogWrite(Motor1, 0);
    while(!Serial.available());
    Serial.read();
  }

 
  
}

void updateData1() {
  
  // declare single use variable
  int A = digitalRead(CLK_1);
  int B = digitalRead(DT_1);


  // assuming CLK_1 is high due to rising edge, DT_1 determines if CW or CCW
  //if (millis() - previousTime1 < interval) return;
  //previousTime1 = millis();

  if (A == B) {
    angularPos1++;
  }
  else {
    angularPos1--;
  }


}
