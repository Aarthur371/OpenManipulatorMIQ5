from picamera import PiCamera
import cv2
import numpy as np
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
photo = np.empty((640 * 480 * 3), dtype=np.uint8)
camera.capture(photo, 'bgr')
cv2.imwrite("Test.png", photo)