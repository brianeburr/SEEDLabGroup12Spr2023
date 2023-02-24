#OpenCV setup
import numpy as np
import cv2 as cv
import cv2.aruco as aruco

#I2C setup
import smbus
import time, threading
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from struct import *

bus = smbus.SMBus(1)
ardAddress = 0x04 #arduinos initialized I2C address
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C() #initalize i2c object for comm

lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [0,0,100]
lcd.message = "hello"

mostRecentDetectedQuad = 0 #may need to update value to None
currentPositionOfWheel = 0


## Video Capture



## Quadrant Calculation

def findQuadrant(markID, corners, resX=640, resY=480):
    # Initial instantiations of variables and print labels to be used later
    markerLabel = 'Marker {} detected in Quadrant {}\n'
    sumOffX = sumOffY = 0
    resX /= 2
    resY /= 2
    quad = -1

    # Runs an average of the marker corners to find the horizontal center,
    # then finds the quadrant based off the sign of the distance from the
    # center of the image
    # Y values are unintuitive because +y is down in an image
    for ind in range(4):
        sumOffX += corners[ind][0] - resX
        sumOffY += corners[ind][1] - resY

    if sumOffX >= 0:
        if sumOffY >= 0: quad = 3
        else: quad = 0
    else:
        if sumOffY >= 0: quad = 2
        else: quad = 1
    
    print(markerLabel.format(markID, quad+1))
    return quad

def sendSetpoint(setPoint):
    bus.write_byte_data(ardAddress, 0, setPoint) #ard will recieve 2 bytes on I2C, first 0 as offset for interp, then setpoint 
    
def recieveCurrentPosition():
    global currentPositionOfWheel
    currentPosFloatBytes = bus.read_i2c_block_data(ardAddress, 1, 4) #send offset of 1 to prep for request, then ard will sent 4 bytes corresponding to float of position
    currentPositionOfWheel = unpack('f', bytes(currentPosFloatBytes))[0]
    print(currentPositionOfWheel)


    
def writeLCD():
    global mostRecentDetectedQuad
    global currentPositionOfWheel
    global lcd
    print("X")
    lcd.clear()
    message = "Setpoint: " + str(mostRecentDetectedQuad) + "\nPosition: "+ str(round(currentPositionOfWheel,2))
    print(message)
    lcd.message = message
    #print(mostRecentDetectedQuad)
    

def handleI2C():
    global mostRecentDetectedQuad
    sendSetpoint(mostRecentDetectedQuad)
    print("setpoint sent")
    recieveCurrentPosition()
    print("Position calc'd")
    writeLCD()
    print("lcd written")
    threading.Timer(0.2, handleI2C).start()

    
def vidCap():
    global mostRecentDetectedQuad
    # Starts a cv video capture for continuous object tracking
    cap = cv.VideoCapture(-1)
    if not cap.isOpened():
        print('Err: Cannot open camera\n')
        exit()
    print('Opening video feed. Press \'q\' to exit')

    # Fetches ArUco dictionary and creates default detection parameters
    arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    params = aruco.DetectorParameters_create()

    # Runs detection code until user input interrupt
    while True:
        idx = 0
        #time.sleep(1)
        # If there is an issue with frame reading, exits the program
        ret, frame = cap.read()
        if not ret:
            print('Err: Cannot receive frame. Exiting...\n')
            break

        # Runs marker detection on a grayscale version of the current frame,
        # then prints image quadrant of a detected marker
        grayed = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, markIDs, rejected = aruco.detectMarkers(grayed, arucoDict, parameters = params)
        if markIDs is None:
            print('No markers found.\n')
        else:
            for ind in range(markIDs.size):
                mostRecentDetectedQuad = findQuadrant(markIDs[ind], corners[ind][0])
                #print(mostRecentDetectedQuad)
                
        # Draws the detected markers on the frame for display
        # Mostly for testing, can be cut to reduce lag
        marked = aruco.drawDetectedMarkers(frame, corners)
        cv.namedWindow('Detected Markers', cv.WINDOW_NORMAL)
        cv.imshow('Detected Markers', marked)
        #if idx == 10:
         #   handleI2C()      ------re enable for i2c without threading
        #    idx = 0
        #idx+=1
       # try:
        #    handleI2C()
         #   pass
        #except:
         #   lcd.message = "IOError"
        #lcd.message = "hello"
        if cv.waitKey(1) == ord('q'):
            break

    # Destroy the capture and window processes once the program exits
    cap.release()
    cv.destroyAllWindows()



## Small User Interaction Code
loop = True
while(loop):
    i = 0
    print('Press <ENTER> to begin quadrant tracking.')
    print('Press <q> to stop the program if needed.\n')
    input('')
    handleI2C()
    vidCap()
    

