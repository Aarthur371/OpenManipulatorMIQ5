import cv2
import os
import numpy as np

horizontal_res = 640
vertical_res = 480

# open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_res)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_res)

# Prise de la photo
ret, frame = cap.read()

# write frame to file
cv2.imwrite('test_image.jpg', frame)
# release camera
cap.release()