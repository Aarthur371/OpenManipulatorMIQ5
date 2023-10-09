import cv2
import os
import numpy as np

horizontal_res = 640
vertical_res = 480

# open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_res)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_res)

# Camera calibration path
calib_camera_path = os.getcwd() + "/config_camera/" + str(horizontal_res) + "x" + str(vertical_res) + "/"
camera_matrix = np.loadtxt(calib_camera_path+'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_camera_path+'cameraDistortion.txt', delimiter=',')
matrice_camera_corrigee, ROI_camera_corrigee = cv2.getOptimalNewCameraMatrix(camera_matrix, camera_distortion, (horizontal_res, vertical_res), 1, (horizontal_res, vertical_res))

# Prise de la photo
ret, frame = cap.read()
# Correction de la photo avec les matrices de correction
photo_corrigee = cv2.undistort(frame, camera_matrix, camera_distortion, None, matrice_camera_corrigee)
# Rognage de la matrice pour ne garder que la partie corrigée
photo_corrigee = photo_corrigee[ROI_camera_corrigee[1]:ROI_camera_corrigee[1]+ROI_camera_corrigee[3],
                                ROI_camera_corrigee[0]:ROI_camera_corrigee[0]+ROI_camera_corrigee[2]]


# write frame to file
cv2.imwrite('image_corrigee.jpg', photo_corrigee)
# release camera
cap.release()