#!/usr/bin/env python3

import cv2
import imutils

# Open the device at the ID 0

cap = cv2.VideoCapture(6)

#Check whether user selected camera is opened successfully.

if not (cap.isOpened()):

    print("Could not open video device")


while(True):

# Capture frame-by-frame

    ret, frame = cap.read()
    frame = cv2.rotate(frame, cv2.ROTATE_180)

# Display the resulting frame
    frame = imutils.resize(frame, width=1500)

    cv2.imshow('preview',frame)

#Waits for a user input to quit the application

    if cv2.waitKey(1) & 0xFF == ord('q'):

        break

#When everything done, release the capture

cap.release()

cv2.destroyAllWindows()