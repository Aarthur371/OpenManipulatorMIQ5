import cv2

# open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)

# set dimensions
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# take frame
ret, frame = cap.read()

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Conversion de l'image en couleur en image de nuances de gris
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
# Récupération des coins de l'échiquier
ret, corners = cv2.findChessboardCorners(gray, (7,7), None)      

# Si les coins ont été détectés on les trace et on les affiche pour l'utilisateur
corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
cv2.drawChessboardCorners(frame, (7,7), corners2, ret)


# write frame to file
cv2.imwrite('image.jpg', frame)
# release camera
cap.release()