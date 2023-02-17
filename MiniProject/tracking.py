import numpy as np
import cv2 as cv
import cv2.aruco as aruco

## Video Capture
def vidCap():
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
                findQuadrant(markIDs[ind], corners[ind][0])

        # Draws the detected markers on the frame for display
        # Mostly for testing, can be cut to reduce lag
        marked = aruco.drawDetectedMarkers(frame, corners)
        cv.namedWindow('Detected Markers', cv.WINDOW_NORMAL)
        cv.imshow('Detected Markers', marked)

        if cv.waitKey(1) == ord('q'):
            break

    # Destroy the capture and window processes once the program exits
    cap.release()
    cv.destroyAllWindows()

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

## Small User Interaction Code
loop = True
while(loop):
    print('Press <ENTER> to begin quadrant tracking.')
    print('Press <q> to stop the program if needed.\n')
    input('')

    vidCap()
