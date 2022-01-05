import cv2
import numpy as np

# Playing video from file:
# cap = cv2.VideoCapture('vtest.avi')
# Capturing video from webcam:
clamp_cam = cv2.VideoCapture(0)
laptop_cam = cv2.VideoCapture(1)

# Capture frame-by-frame
ret1, frame1 = clamp_cam.read()
ret2, frame2 = laptop_cam.read()

# Handles the mirroring of the current frame
frame1 = cv2.flip(frame1,1)
frame2 = cv2.flip(frame2,1)

# Our operations on the frame come here
#gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Saves image of the current frame in jpg file
#name = 'frame' + str(currentFrame) + '.jpg'
# cv2.imwrite(name, frame)

# Display the resulting frame
cv2.imshow('frame1',frame1)
cv2.imshow('frame2', frame2)
if cv2.waitKey(0) & 0xFF == ord('q'):
    cv2.destroyAllWindows

# To stop duplicate image

# When everything done, release the capture
clamp_cam.release()
laptop_cam.release()
cv2.destroyAllWindows()