#include <Wire.h>
#define FOLLOWER_ADDRESS 0x04
int data[32]; //array to store recieved data
byte floatNums[32];
float motorPosition_rad = 123.456;    // Current position of motor in radians
byte* motorRef;
int arucoPosition = 0;               // Aruco position: 0,1,2, or 3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Wire.begin(FOLLOWER_ADDRESS);
  Wire.onReceive(receiveData); // defines function to be called at recieve interrupt
  Wire.onRequest(sendData); // defines function when sending data back to pi
  Serial.println("Ready!");
}

void loop() {
  // put your main code here, to run repeatedly:
    delay(50);
}

void sendData() { //called when pi requests data
//place code to return aruco position here
  motorRef = (byte*) &motorPosition_rad;
  Wire.write(motorRef, 4);
  floatNums[0] = motorRef[0];
  floatNums[1] = motorRef[1];
  floatNums[2] = motorRef[2];
  floatNums[3] = motorRef[3];

  //Wire.write(floatNums, 4);

  /*for(int i = 0; i < 4; i++) {
    Serial.print(motorRef[i], HEX);
  }
  Serial.println();*/
  
  //Serial.println("" + (string) motorRef[0] + "|" + (string) motorRef[1] + "|" + (string) motorRef[2] + "|" +  (string) motorRef[3]);
  
}
void receiveData(int byteCount) {
//data to recieve will have offset first, then value to be sstored at address
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