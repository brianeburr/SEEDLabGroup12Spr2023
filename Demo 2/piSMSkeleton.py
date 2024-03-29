## OpenCV Imports
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from enum import Enum
from time import sleep

## I2C Imports
import smbus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import struct
import RPi.GPIO as GPIO

bus = smbus.SMBus(1)
ardAddress = 0x04 # Arduino's initialized I2C address
GPIO.setmode(GPIO.BCM)
GPIO.setup(4,GPIO.IN)

################################################################################
## Pi State Functions

angle = 0
adjFail = False

def angleMeas(markID, corners, hfResX=320, hFov=29.26):
  global adjFail
  #angleThresh = 5 # For speed!!!
  angleThresh = 2 # For precision
  markerLabel = 'Marker {} x-angle is {}\n'
  xOffset = 0
  
  for ind in range(4):
    xOffset += hfResX - corners[ind][0]
  xDeg = (hFov * (xOffset / 4) / hfResX) - 0.5
  
  print(markerLabel.format(markID, xDeg))

  if(abs(angle - xDeg) < 2):
    adjFail = True

  if(abs(xDeg) < angleThresh):
    print('Moving to DISTANCE')
    return state.DISTANCE, xDeg
  else:
    print('Recurse to ADJUST')
    return state.ADJUST, xDeg


def distMeas(markID, corners, resY=480, fovY=48):
  markerLabel = 'Marker {} distance is {} ft\n'
  yOffset = 0
  
  for ind in range(4):
    yOffset += resY - corners[ind][1]
  yOffset = yOffset / 4
  dist = ((yOffset - 560)/(-636)) ** -1.35 # Connor's distance regression
  
  print(markerLabel.format(markID, dist))
  return dist

################################################################################
## I2C Comms Functions

def I2CMessage(offset, floatNum):
  # General I2C message to send
  #struct.unpack("f", struct.pack("f", 0.00582811585976)) might be useful if datatypes messed up
  #print(floatNum)
  #print(offset)
  
  ba = list(bytes(struct.pack('<f', floatNum)))
  #print(ba)
  bus.write_block_data(ardAddress, offset, ba)


def rotate(degrees):
  I2CMessage(2, degrees)
  #sleep(0.1) # Perhaps unnecessary
  I2CMessage(4,0)


def moveForward(cm):
  I2CMessage(3, cm)
  #sleep(0.1) # Perhaps unnecessary
  I2CMessage(5,0)

################################################################################
## OpenCV Camera Actions and State Machine

class state(Enum):
  IDLE = 0
  SCAN = 1
  ADJUST = 2
  DISTANCE = 3
  MOVE = 4

camState = state.IDLE
cap = cv.VideoCapture(-1)
if not cap.isOpened():
  print('Err: cannot open camera\n')
  exit()
print('Starting marker tracking...\n')

# Setup for ArUco parameters and marker dictionary
arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
params = aruco.DetectorParameters_create()

while True:
  ret, frame = cap.read()
  if not ret:
    print('Err: frame read error')
    break
    
  # Image transformations for the detectMarkers function
  grayed = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
  corners, markIDs, rejected = aruco.detectMarkers(grayed, arucoDict, parameters = params)
  
  if(GPIO.input(4) == 0): # If the Arduino is busy/moving, skip loop actions
    #print("busy")
    if(camState == state.MOVE):
      break
    continue

  
  ## Pi-Side State Machine
  if(camState == state.IDLE):
    print('Moving to SCAN\n')
    if(markIDs is not None): # If marker in frame on startup, skip SCAN state
      camState = state.ADJUST
      print('Moving to ADJUST')
      #cv.imshow('Initial State', aruco.drawDetectedMarkers(grayed, corners))
      continue
    # Send signal to Arduino to initiate SCAN state
    I2CMessage(0,0)
    I2CMessage(1,0)
    camState = state.SCAN
  
  elif(camState == state.SCAN):
    if(markIDs is not None): # If a marker is detected, move to the ADJUST state
      camState = state.ADJUST
      print('Moving to ADJUST')
      I2CMessage(0,0) # Send stop command to Arduino for ADJUST state
      
  elif(camState == state.ADJUST and markIDs is not None):
    camState, angle = angleMeas(markIDs[0], corners[0][0]) # Evaluate marker angle w/ camera
    if(camState == state.ADJUST): # If the angle is above the set threshold
      rotate(-1*angle) # Send angle to robot over I2C
    if(adjFail): # If the robot fails to turn the required angle, continue
      camState = state.DISTANCE
      print('Busted!')
      
  elif(camState == state.DISTANCE and markIDs is not None):
    dist = distMeas(markIDs[0], corners[0][0]) # Evaluate marker distance w/ camera
    moveForward(dist * 30.48) # Send distance to Arduino
    camState = state.MOVE
    print('Moving to MOVE\n')
    
  else: # MOVE state
    pass
  
print('Done.\n')
cap.release()
cv.destroyAllWindows()
