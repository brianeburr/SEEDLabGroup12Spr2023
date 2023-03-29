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

void setup() {
  // put your setup code here, to run once:
  currentState = IDLE;
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
