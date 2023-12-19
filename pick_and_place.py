from commande_robot import *
from traitement_image import *
from time import sleep
from datetime import datetime
import cv2

#Constantes
positionDepotVert = [25, 175]
positionDepotAutre = [25, -175]
offsetSaisie = 30
hauteurSaisie = 5

# Définition de la mire
CHECKERBOARD = (6,8)
squareDim = (25, 25)



manipulator = OpenManipulator()
manipulator.goto_position_photo()
manipulator.ouverture_pince()
input("Retirez les kaplas et appuyez sur entrée")
imgCalib = manipulator.prise_photo()
cv2.imwrite("Calib.jpg", imgCalib)
[Lcam, origin] = CalibrationPosition(imgCalib, CHECKERBOARD, squareDim)
input("Positionnez les kaplas et appuyez sur entrée")
img = manipulator.prise_photo()
cv2.imwrite("Photo.jpg", img)
objectsPositions = GetObjectsPositions(img, 4, 1, 3, 70, 3000, Lcam , squareDim)
print(objectsPositions)
M = MatriceChangementRepere(np.array([219, -95, 0]), np.array([96, -98, 0]), np.array([218, 78, 0]))


for position in objectsPositions:
    
    X_Rcam = np.array([position[0], position[1], 0])
    X_Rrobot = np.matmul(M, np.concatenate((X_Rcam,1), axis=None))
    manipulator.MGI_DH(X_Rrobot[0], X_Rrobot[1], offsetSaisie, 3)
    manipulator.MGI_DH(X_Rrobot[0], X_Rrobot[1], hauteurSaisie, 3)
    manipulator.fermeture_pince()
    manipulator.MGI_DH(X_Rrobot[0], X_Rrobot[1], offsetSaisie, 3)
    if position[2] == 1:
        manipulator.MGI_DH(positionDepotVert[0], positionDepotVert[1], offsetSaisie, 3)
    else:
        manipulator.MGI_DH(positionDepotAutre[0], positionDepotAutre[1], offsetSaisie, 3)
    manipulator.ouverture_pince()
