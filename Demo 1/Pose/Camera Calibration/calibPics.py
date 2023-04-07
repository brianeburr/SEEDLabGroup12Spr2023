import numpy as np
import cv2 as cv
import cv2.aruco as aruco

print("Calibration Camera Captures")
print("Press s to save, press q to quit")
count = 1
fGlob = "checker{}.png"

cap = cv.VideoCapture(-1)
if not cap.isOpened():
  print('Err: cannot open camera\n')
  exit()
print('Opening video feed. Press \'q\' to exit')

while True:
    ret, frame = cap.read()
    if not ret:
        print('Err: cannot receive frame. Exiting...\n')
        break

    cv.imshow('Video Feed', frame)
    k = cv.waitKey(1)
    if k == ord('s'):
        cv.imwrite(fGlob.format(count), frame)
        print("Captured {}".format(fGlob.format(count)))
        count = count + 1
    elif k == ord('q'):
        break

print('Exiting program.\n')
cap.release()
cv.destroyAllWindows()
