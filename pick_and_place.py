from commande_robot import *
from traitement_image import *
from time import sleep
from datetime import datetime
import cv2

# Constantes de déplacement du robot
positionDepotBleu = [0, 120]
positionDepotAutre = [0, -120]
offsetSaisie = 30
hauteurSaisie = 2
taille_kapla = 14
nb_vert = 0
nb_non_vert = 0

# Matrice de changement du repère caméra vers le repère de l'OpenManipulator
M = MatriceChangementRepere(np.array([219, -95, 0]), np.array([96, -98, 0]), np.array([218, 78, 0]))

# Définition de la mire
CHECKERBOARD = (6,8)
squareDim = (25, 25)

# Instanciation de l'objet OpenManipulator
manipulator = OpenManipulator()

# Déplacement du robot jusqu'à la position de prise de photo
manipulator.goto_position_photo(2)
# Attente d'une réponse de l'utilisateur pour prendre la photo de calibration
input("Retirez les kaplas et appuyez sur entrée")
# Prise de la photo de calibration
imgCalib = manipulator.prise_photo()
# Récupération des paramètres extrinsèques
[Lcam, origin] = CalibrationPosition(imgCalib, CHECKERBOARD, squareDim)


try:

    while(True):
        
        # Attente d'une réponse de l'utilisateur pour prendre la photo avec les kaplas
        input("Positionnez les kaplas et appuyez sur entrée")
        # Prise de la photo avec kaplas
        img = manipulator.prise_photo()
        #cv2.imwrite("Photo.jpg", img)
        # Récupération de la position et des couleurs des kaplas détectés
        objectsPositions = GetObjectsPositions(img, 4, 1, 3, 70, 3000, Lcam , squareDim)
        #print(objectsPositions)
        # Ouverture de la pince
        manipulator.ouverture_pince(0.5)

        for position in objectsPositions:
            # Récupération des coordonnées de l'objet dans le repère caméra
            X_Rcam = np.array([position[0], position[1], 0])
            # Conversion des coordonnées dans le repère du robot
            X_Rrobot = np.matmul(M, np.concatenate((X_Rcam,1), axis=None))
            # Déplacement au dessus de l'objet 
            manipulator.MGI_DH(X_Rrobot[0], X_Rrobot[1], offsetSaisie, 1)
            # Descente jusqu'à la position de saisie
            manipulator.MGI_DH(X_Rrobot[0], X_Rrobot[1], hauteurSaisie, 0.5)
            # Prise de l'objet
            manipulator.fermeture_pince(1)
            # Remontée de l'objet
            manipulator.MGI_DH(X_Rrobot[0], X_Rrobot[1], offsetSaisie, 0.5)

            # Si l'objet est vert
            if position[2] == 0:
                # Déplacement du robot au dessus la zone de dépôt pour kapla vert
                manipulator.MGI_DH(positionDepotBleu[0], positionDepotBleu[1], hauteurSaisie+taille_kapla*nb_vert+offsetSaisie, 1)
                # Descente du kapla
                manipulator.MGI_DH(positionDepotBleu[0], positionDepotBleu[1], hauteurSaisie+taille_kapla*nb_vert, 0.5)
                # Laché du kaple
                manipulator.ouverture_pince(1)
                # Remontée du robot
                manipulator.MGI_DH(positionDepotBleu[0], positionDepotBleu[1], hauteurSaisie+taille_kapla*nb_vert+offsetSaisie, 0.5)
                # Ajout du compteur du nombre de kaplas verts
                nb_vert += 1
            # Si l'objet n'est pas vert
            else:
                # Déplacement du robot au dessus la zone de dépôt pour kapla non vert
                manipulator.MGI_DH(positionDepotAutre[0], positionDepotAutre[1], hauteurSaisie+taille_kapla*nb_non_vert+offsetSaisie, 1)
                # Descente du kapla
                manipulator.MGI_DH(positionDepotAutre[0], positionDepotAutre[1], hauteurSaisie+taille_kapla*nb_non_vert, 0.5)
                # Lachée du kaple
                manipulator.ouverture_pince(1)
                # Remontée du robot
                manipulator.MGI_DH(positionDepotAutre[0], positionDepotAutre[1], hauteurSaisie+taille_kapla*nb_non_vert+offsetSaisie, 0.5)
                # Ajout du compteur du nombre de kaplas non verts
                nb_non_vert += 1

            # Déplacement du robot jusqu'à la position de prise de photo
            manipulator.goto_position_photo(2)

except KeyboardInterrupt:
    print("Fin de programme")
