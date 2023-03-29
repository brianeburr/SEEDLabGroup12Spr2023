#include <Wire.h>

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

//I2C stuff

#define FOLLOWER_ADDRESS 0x04
int data[32]; // buffer to store i2c message into

typedef union I2C_Packet_t {
  byte floatArrayNums[4];
  float floatNum;
};

void setup() {
  // put your setup code here, to run once:
  currentState = IDLE;
  Wire.begin(FOLLOWER_ADDRESS);
  Wire.onReceive(receiveData);
}

void loop() {
  // put your main code here, to run repeatedly:


  switch(currentState) {
    case IDLE:

    break;
    case SCAN:

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
