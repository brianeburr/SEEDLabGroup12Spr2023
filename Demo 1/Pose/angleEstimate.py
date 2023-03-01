import numpy as np
import cv2 as cv
import cv2.aruco as aruco

def angleCalc(markID, corners, hfResX=320, hfResY=240, hFov=33):
  # markerLabel = 'Marker {} x-angle is {}\n'
  xOffset = 0
  
  for ind in range(4):
    xOffset += hfResX - corners[ind][0]
  xDeg = hFov * (xOffset / 4) / hfResX
  
  # print(markerLabel.format(markID, xDeg))
  return xDeg


## Add in camera calibration
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
  ## Introduce distortion protection
  
  corners, markIDs, rejected = aruco.detectMarkers(grayed, arucoDict, parameters = params)
  if markIDs is None:
    # print('No markers found.\n')
  else:
    for ind in range(markIDs.size):
      angleCalc(markIDs[ind], corners[ind][0])
  
  ## Space for I2C calls?
  
  # cv.imshow('Detected Markers', aruco.drawDetectedMarkers(frame, corners))
  if cv.waitKey(1) == ord('q'):
    break

print('Exiting program.\n')
cap.release()
cv.destroyAllWindows()
