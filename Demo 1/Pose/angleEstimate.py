## OpenCV Imports
import numpy as np
import cv2 as cv
import cv2.aruco as aruco

## I2C Imports
import smbus
import time, threading
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from struct import *

## I2C Stuff (Hi Brian)
angle = 0
detected = False
bus = smbus.SMBus(1)
# ardAddress = 0x04
lcd_cols = 16
lcd_rows = 2
i2c = board.I2C()

lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [0, 0, 100]
lcd.message = "System On"

"""
def sendSetpoint(setPoint):
  bus.write_byte_data(ardAddress, 0, setPoint) #ard will recieve 2 bytes on I2C, first 0 as offset for interp, then setpoint 
  
def recieveCurrentPosition():
  global currentPositionOfWheel
  currentPosFloatBytes = bus.read_i2c_block_data(ardAddress, 1, 4) #send offset of 1 to prep for request, then ard will sent 4 bytes corresponding to float of position
  currentPositionOfWheel = unpack('f', bytes(currentPosFloatBytes))[0]
  print(currentPositionOfWheel)
  
def writeLCD():
  global angle
  global lcd
  # I2C code from Mini Project
  print("X")
  lcd.clear()
  message = "Setpoint: " + str(mostRecentDetectedQuad) + "\nPosition: "+ str(round(currentPositionOfWheel,2))
  print(message)
  lcd.message = message
  print(mostRecentDetectedQuad)

def handleI2C():
  global angle
  # I2C code from Mini Project
  sendSetpoint(mostRecentDetectedQuad)
  print("setpoint sent")
  recieveCurrentPosition()
  print("Position calc'd")
  writeLCD()
  print("lcd written")
  threading.Timer(0.2, handleI2C).start()
"""

def handleI2C():
  global angle
  global detected
  if detected:
    lcd.clear()
    message = "Marker Detected :)\n" + "Angle: " + str(round(angle,2))
  else:
    lcd.clear()
    message = "Marker not detected\n:("
  threading.Timer(0.2, handleI2C).start()


## OpenCV Angle Calculation
def angleCalc(markID, corners, hfResX=320, hfResY=240, hFov=33):
  # markerLabel = 'Marker {} x-angle is {}\n'
  xOffset = 0
  
  for ind in range(4):
    xOffset += hfResX - corners[ind][0]
  xDeg = hFov * (xOffset / 4) / hfResX
  
  # print(markerLabel.format(markID, xDeg))
  return xDeg

## OpenCV Camera Setup
# Add in camera calibration
handleI2C()
cap = cv.VideoCapture(-1)
if not cap.isOpened():
  print('Err: cannot open camera\n')
  exit()
print('Opening video feed. Press \'q\' to exit')

arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
params = aruco.DetectorParameters_create()

while True:
  ret, frame = cap.read()
  if not ret:
    print('Err: cannot receive frame. Exiting...\n')
    break
  
  grayed = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
  # Introduce distortion protection
  
  corners, markIDs, rejected = aruco.detectMarkers(grayed, arucoDict, parameters = params)
  if markIDs is None:
    detected = False
    # print('No markers found.\n')
  else:
    for ind in range(markIDs.size):
      angle = angleCalc(markIDs[ind], corners[ind][0])
    detected = True
  
  # cv.imshow('Detected Markers', aruco.drawDetectedMarkers(frame, corners))
  if cv.waitKey(1) == ord('q'):
    break

print('Exiting program.\n')
cap.release()
cv.destroyAllWindows()
