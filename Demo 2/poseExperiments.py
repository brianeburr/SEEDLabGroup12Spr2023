## OpenCV Imports
import numpy as np
import cv2 as cv
import cv2.aruco as aruco
from time import sleep

# Some included camera matrices for possible calibration implementation
"""camMtx = np.array([[ 605.21881298, 0,            304.72314493],
                   [ 0,            602.70534241, 222.09439201],
                   [ 0,            0,            1           ]])"""

#camMtx = np.array([[ 605.21881298, 0,            304.72314493],
                   #[ 0,            602.70534241, 222.09439201],
                   #[ 0,            0,            1           ]])

#dist = np.array([[ 0, 0, 0, 0, 0 ]])

"""dist = np.array([[  5.71005717e-02,  -6.19737430e-02,  -7.16729572e-03,
  5.17895277e-04,  -7.38185549e-01]])"""

# radial^2, radial^4, tangentA, tangentB, radial^6

#h,w = (480,640)
#newCamMtx, roi = cv.getOptimalNewCameraMatrix(camMtx,dist,(w,h),1,(w,h))
#x,y,w,h = roi


## OpenCV Distance Calculation
def distCalc(markID, corners, hfResX=320, resY=480, fovY=48):
  #markerLabel = '{}, {}, {}, {}'
  markerLabel = 'Marker {} distance is {} ft\n'
  #markerLabel = 'Marker {} pixels from camera bottom: {}'
  yOffset = 0
  
  for ind in range(4):
    yOffset += resY - corners[ind][1]
  yOffset = yOffset/ 4
  dist = ((yOffset - 560)/(-636)) ** -1.234 # Some distance regression
  
  print(markerLabel.format(markID, dist))
  return dist

## OpenCV Print to Camera

## OpenCV Camera Setup
cap = cv.VideoCapture(-1)
cap.read()
cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv.CAP_PROP_EXPOSURE, 0.0075)

if not cap.isOpened():
  print('Err: cannot open camera\n')
  exit()
print('Opening video feed. Press \'q\' to exit')

# Sets up ArUco parameters for later
arucoDict = aruco.Dictionary_get(aruco.DICT_6X6_250)
params = aruco.DetectorParameters_create()

while True:
  ret, frame = cap.read()
  if not ret:
    print('Err: cannot receive frame. Exiting...\n')
    break
  
  # Some image transformations for feeding into the detectMarkers function
  grayed = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
  #grayed = cv.undistort(grayed, camMtx, dist, None, newCamMtx)
  #grayed = grayed[y:y+h, x:x+w]
  
  corners, markIDs, rejected = aruco.detectMarkers(frame, arucoDict, parameters = params)
  if markIDs is None:
    detected = False
    #print('No markers found.\n')
  else:
    for ind in range(markIDs.size):
      # Sends the detected markers to have their angles calculated and returned to a global variable
      dist = distCalc(markIDs[ind], corners[ind][0])
    detected = True
  
  cv.imshow('Detected Markers', aruco.drawDetectedMarkers(frame, corners))
  if cv.waitKey(1) == ord('q'):
    break

print('Exiting program.\n')
cap.release()
cv.destroyAllWindows()
