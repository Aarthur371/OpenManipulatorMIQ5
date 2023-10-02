"""from picamera import PiCamera
import cv2
import numpy as np
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
photo = np.empty((640 * 480 * 3), dtype=np.uint8)
camera.capture(photo, 'bgr')
cv2.imwrite("Test.png", photo)"""

import cv2

# open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

# take frame
ret, frame = cap.read()
# write frame to file
cv2.imwrite('image.jpg', frame)
# release camera
cap.release()