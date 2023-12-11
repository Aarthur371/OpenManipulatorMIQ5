import cv2
from open_manipulator_msgs.srv import *
from commande_robot import OpenManipulator

manipulator = OpenManipulator()


horizontal_res = 1920
vertical_res = 1440
# open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_res)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_res)

try:
    while(True):
        reponse = input("Prise de photo -> p nom_photo\n" +
                        "MGI -> i x y z\n" +
                        "MDG -> d q1 q2 q3 q4\n"+
                        "déplacement relatif -> r qi theta\n" + 
                        "Ouvertur de la pince -> o\n"+
                        "Fermeture de la pince -> f")
        if reponse != "":
            tab_reponse = reponse.split(' ')
            if tab_reponse[0] == "p":
                ret, frame = cap.read()
                cv2.imwrite(tab_reponse[1] + '.jpg', frame)
                print("Photo prise")
            elif tab_reponse[0] == "i":
                manipulator.MGI(float(tab_reponse[1])/100, float(tab_reponse[2])/100, float(tab_reponse[3])/100, 1)
            elif tab_reponse[0] == "d":
                manipulator.MGD(float(tab_reponse[1]), float(tab_reponse[2]), float(tab_reponse[3]), float(tab_reponse[4]), 1)
            elif tab_reponse[0] == "r":
                manipulator.deplacement_relatif_moteur(float(tab_reponse[2]), tab_reponse[1], 1)
            elif tab_reponse[0] == "o":
                manipulator.ouvrir_pince(0.01, 1)
            elif tab_reponse[0] == "f":
                manipulator.ouvrir_pince(-0.01, 1)
            else:
                print("Commande non reconnue")
        else:
            print("Commande vide")
except KeyboardInterrupt:
    print("Fin du programme")
    cap.release()