## OpenCV Imports
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from enum import Enum


## I2C Imports
import smbus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from struct import *

#####################################################################################
## Pi State Functions

def angleMeas(markID, corners, hfResX=320, hFov=29.26):
  markerLabel = 'Marker {} x-angle is {}\n'
  xOffset = 0
  
  for ind in range(4):
    xOffset += hfResX - corners[ind][0]
  xDeg = (hFov * (xOffset / 4) / hfResX) - 0.5
  
  print(markerLabel.format(markID, xDeg))
  #return xDeg
  
  if(-5 <= xDeg <= 5):
    return 2, xDeg
  else:
    return 3, xDeg


def distMeas(markID, corners, resY=480, fovY=48):
  markerLabel = 'Marker {} distance is {} ft\n'
  yOffset = 0
  
  for ind in range(4):
    yOffset += resY - corners[ind][1]
  yOffset = yOffset / 4
  dist = ((yOffset - 560)/(-636)) ** -1.234 # Connor's distance regression
  
  print(markerLabel.format(markID, dist))
  return dist
#####################################################################################

def I2CMessage(offset, float):
  #general I2C message to send
  pass


## OpenCV Camera Instantiation (Prelim)
# Needs tweaking for different possible camera setups
class state(Enum):
  IDLE = 0
  SCAN = 1
  ADJUST = 2
  DISTANCE = 3
  MOVE = 4

camState = IDLE
cap = cv.VideoCapture(-1)
if not cap.isOpened():
  print('Err: cannot open camera\n')
  exit()
print('Opening video feed...')

# Setup for ArUco parameters and marker dictionary
arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
params = aruco.DetectorParameters_create()

while True:
  ret, frame = cap.read()
  if not ret:
    break
    
  # Image transformations for the detectMarkers function
  grayed = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
  corners, markIDs, rejected = aruco.detectMarkers(grayed, arucoDict, parameters = params)
  
  
  if(camState == IDLE): # Idle
    print('Idle state, no action') # Needs a short bit of code for idle state
  elif(camState == SCAN): # Scan
    if(markIDs is not None):
      camState = ADJUST
      # Send stop command to robot
      # Small delay signal to account for robot stop delay
  elif(camState == ADJUST): # Adjust (Camera Angle)
    camState, angle = angleMeas(markIDs[0], corners[0][0])
    if(camState == ADJUST): # If the program sent back to adjust
      pass
      # Send angle to robot over I2C
    # If state is distMeas, no I2C should be needed
  elif(camState == DISTANCE): # Distance
    dist = distMeas(markID[0], corners[0][0])
    # Send distance to Arduino
    camState = MOVE
  else: # Move
    print('Move state, no action')
  
print('Done.\n')
cap.release()
cv.destroyAllWindows()
