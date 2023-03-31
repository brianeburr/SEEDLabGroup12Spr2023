## OpenCV Imports
import numpy as np
import cv2 as cv
import cv2.aruco as aruco


## I2C Imports
import smbus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from struct import *


## Pi State Functions
def idle():
  pass

def scan():
  #if(markerFound && (angle < arbitraryThresh)):
  #  return angleMeas
  pass

def angleMeas():
  #if():
  #  return adjust
  #else:
  #  return distMeas
  pass

def distMeas():
  #return move
  pass


def I2CMessage(offset, float):
  #general I2C message to send
  pass


## OpenCV Camera Instantiation (Prelim)
# Needs tweaking for different possible camera setups
camState = idle
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
  #grayed = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
  #grayed = cv.undistort(grayed, camMtx, dist, None, newCamMtx)
  #grayed = grayed[y:y+h, x:x+w]
  #corners, markIDs, rejected = aruco.detectMarkers(grayed, arucoDict, parameters = params)
  
  # State machine conditions will be above
  # I2C stuff can go here eventually
  
print('Done.\n')
cap.release()
cv.destroyAllWindows()
