"""
Code permettant de calculer les matrices de correction d'images de la caméra

Fonctionnement du code:
    On prend des photos en continu avec la picamera. Sur ces photos, on essaie de trouver un échiquier
    Si un échiqier est trouvé, on garde l'image. On continue ainsi jusqu'à ce que l'utilisateur appuie sur "q"
    On calcule les matrices de caméra et distortion avec les images d'échiqiers sauvergardées.
    On corrige une image de contrôle avec les matrices calculées et les matrices actuelles (si elles ont déjà été calculées)
    On affiche les deux images corrigées et l'utilisateur peut choisir de garder les nouvelles matrices ou de les défaussées
    
Spécificités du codes:
    Les paramètres du codes sont la résolution de la caméra et la longueur des arrêtes des cases de l'échiquier, les matrices changent si on modifie ces paramètres
    Les matrices de correction d'images et l'image de contrôle sont stockés dans un dossier "horizotal_resxvertical_res" -> exemple "640x480"
    Si ce dossier, l'une des matrices de correction ou l'image de contrôle n'existe pas, on le/la crée dans le code
    L'image de calibration peut être n'importe quelle image mais il est plus judicieux de prendre une photo de l'échiquier en gros plan pour observer les déformations
    Pour augmenter la précision de la correction, il faut que le plan de l'échiquier soit parallèle au plan de la lentille
    Il faut au moins une trentaine d'image pour avoir une correction efficace
"""

# Imports
import cv2
import numpy as np
import os.path
from time import sleep

# Variables de travail
horizotal_res = int(640*2) # en pixel
vertical_res = int(480*2) # en pixel
taille_case = 25 # en mm

# S'il n'existe pas déjà, on crée le dossier de paramètres de caméra pour la résolution choisie
try:
    os.mkdir(str(horizotal_res) + "x" + str(vertical_res))
except FileExistsError:
    pass

# Chemin vers les (futures) matrices de calibration 
path_camera_matrix = str(horizotal_res) + "x" + str(vertical_res) + "/" +"cameraMatrix.txt"
path_camera_distrotion = str(horizotal_res) + "x" + str(vertical_res) + "/" + "cameraDistortion.txt"
path_image_controle = str(horizotal_res) + "x" + str(vertical_res) + "/" + "imageControle.jpg"

# Configuration de la picamera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizotal_res)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_res)
photo = np.empty((vertical_res * horizotal_res * 3), dtype=np.uint8)

# Variable pour stocker les images 
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, taille_case, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((7*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Initialisation du compteur d'images prise en compte pour la calibration
images_valides=0
threshold = int(input("Combien d'imagess voulez vous prendre ?"))
_, frame = cap.read()

# On capture en permancence les photos de la picamera 
while(images_valides <= threshold):
#for frame in picamera.capture_continuous(photo, format="bgr", use_video_port=True):
    
    # Prise de la photo
    ret, frame = cap.read()
    # Conversion de l'image en couleur en image de nuances de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Récupération des coins de l'échiquier
    ret, corners = cv2.findChessboardCorners(gray, (7,7), None)      
    # Si les coins n'ont pas été détectés, on indique que l'image n'est pas valide et on interrompt l'itération en cours 
    if ret == False:
        print("Echiquier non détecté, image non prise en compte")
        continue
    
    #Si l'image a été validée on ajoute l'image aux tableaux de données et on incrémente le compteur d'image
    corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
    print("Photo valide prise. Nombre total de photos prises : " + str(images_valides))
    objpoints.append(objp)
    imgpoints.append(corners2)
    images_valides+=1
    
    # Temporisation pour ne pas avoir trop d'image et donc un temps de calcul trop long 
    sleep(0.2)


# Calcul et affichage des matrices de caméra et de distortion
print("Calcul des matrices en cours")
_, new_mtx, new_dist, _, _ = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("Matrice de la caméra obtenue :")
print(new_mtx)
print("Matrice de distortion obtenue :")
print(new_dist)

#Enregitrement de la dernière photo prise
cv2.imwrite('echequier_non_corige.jpg', frame)

# Correction d'image avec les nouvelles matrices
# Calcul de la nouvelle matrice de correction et la zone d'intérêt liée à cette matrice
new_cameramtx, new_zoi = cv2.getOptimalNewCameraMatrix(new_mtx, new_dist, (horizotal_res, vertical_res), 1, (horizotal_res, vertical_res))
# Correction de l'image
nouvelle_correction = cv2.undistort(frame, new_mtx, new_dist, None, new_cameramtx)
# Rognage de l'image pour faire correspondre l'image à la correction
nouvelle_correction = nouvelle_correction[new_zoi[1]:new_zoi[1]+new_zoi[3], new_zoi[0]:new_zoi[0]+new_zoi[2]]
# Affichage de l'image corrigée
cv2.imwrite('echequier_non_corige.jpg', nouvelle_correction)