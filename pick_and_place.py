from commande_robot import *
from traitement_image import *
from time import sleep

#Constantes
positionDepot = [100, 100] # a definir
offsetSaisie = 50

# ----- Définir constantes -----

# Définition de la mire
CHECKERBOARD = (6,8)
squareDim = (25, 25)



manipulator = OpenManipulator()
manipulator.goto_position_photo()
manipulator.ouverture_pince()
sleep(2)
#imgCalib = manipulator.prise_photo()
#[Lcam, origin] = CalibrationPosition(imgCalib, CHECKERBOARD, squareDim)
input("Positionnez les kaplas et appuyez sur entrée")
img = manipulator.prise_photo()
#objectsPositions = GetObjectsPositions(img, 5, 15, 2, 147, 3000, Lcam, origin, CHECKERBOARD, squareDim)

#objectsPositions = np.array([[96, -98,0],[96, 98,0]])
objectsPositions = np.array([[107, 48,0], [113, 170, 0]])
M = MatriceChangementRepere(np.array([219, -95, 0]), np.array([96, -98, 0]), np.array([218, 78, 0]))


for position in objectsPositions:
    
    X_Rcam = np.array([position[0], position[1], 0])
    X_Rrobot = np.matmul(M, np.concatenate((X_Rcam,1), axis=None))
    manipulator.MGI_DH(X_Rrobot[0], X_Rrobot[1], offsetSaisie, 5)
    manipulator.MGI_DH(X_Rrobot[0], X_Rrobot[1], 0, 5)
    manipulator.fermeture_pince()
    manipulator.MGI_DH(X_Rrobot[0], X_Rrobot[1], offsetSaisie, 5)
    manipulator.MGI_DH(positionDepot[0], positionDepot[1], offsetSaisie, 5)
    manipulator.MGI_DH(positionDepot[0], positionDepot[1], 0, 5)
    manipulator.ouverture_pince()