import rospy
import cv2
import math
from open_manipulator_msgs.srv import *
from open_manipulator_msgs.msg import *

class OpenManipulator :

    d1 = 77
    a2 = 130
    a3 = 124
    a4 = 126+35
    phi = math.atan2(128, 24)

    def __init__(self):

        # Liste des noms des services pour utiliser les fonctions de cette classe
        self.service_name_MGI = "/goal_task_space_path"#_position_only"i
        self.service_name_MGD = "/goal_joint_space_path"
        self.service_name_MGD_relatif = "/goal_joint_space_path_from_present"
        self.service_name_deplacement_effecteur = "/goal_tool_control"

        # open camera
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)


    def prise_photo(self):
        """
        Fonction permettant de prendre une photo avec la pi caméra

        Sorties:
            frame : Photo prise par la caméra. Renvoie None si la prise de photo a échouté
        """
        # On appelle 5 fois la fonction "read" parce que sinon la photo ne s'actualise pas :)
        self.cap.read()
        self.cap.read()
        self.cap.read()
        self.cap.read()
        ret, frame = self.cap.read()
        return frame if ret else None
    


    def MGI(self, x, y, z, time):

        """
        Fonction utilisant le service "/goal_task_space_path_position_only" permettant d'utiliser le MGI du robot
        Entrées :
            x,y,z :
                coordoonées, en mètres, à atteindre par l'effecteur dans le repère du robot
            t :
                temps, en secondes, pour atteindre la position 
        Sorties :
            is_planned = booleén indiquant si le robot peut se rendre à cette position 
        """

        # Attente d'une réponse du servide 
        rospy.wait_for_service(self.service_name_MGI)
        try:
            # Création de la fonction pour appeler le service
            position_service = rospy.ServiceProxy(self.service_name_MGI, SetKinematicsPose)
            # Création du message à envoyer au service 
            arg = SetKinematicsPoseRequest()
            arg.end_effector_name = 'gripper'
            # Renseignement de la position à atteindre
            arg.kinematics_pose.pose.position.x = x
            arg.kinematics_pose.pose.position.y = y
            arg.kinematics_pose.pose.position.z = z
            # Renseignement du temps pour atteindre la précédente position
            arg.path_time = time
            # Appel du service 
            resp1 = position_service(arg)
            return resp1
        
        # En cas d'erreur, on renvoie "False" pour indiquer que la génération de la consigne a échoué
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
    
    def MGI_DH(self, x, y, z, t):

        """
        Fonction utilisant le service "/goal_task_space_path_position_only" permettant d'utiliser le MGI du robot
        Entrées :
            x,y,z :
                coordoonées, en mètres, à atteindre par l'effecteur dans le repère du robot
            t :
                temps, en secondes, pour atteindre la position 
        Sorties :
            is_planned = booleén indiquant si le robot peut se rendre à cette position 
        """

        q1 = math.atan2(y, x)
        try:
            X1 = math.sqrt(x**2 + y**2)
            Y1 = -(z + self.a4 - self.d1)
            q3 = math.acos((Y1**2 + X1**2 - self.a2**2 - self.a3**2) / (2*self.a2*self.a3)) - self.phi
            k1 = self.a2*math.cos(self.phi) + self.a3*math.cos(q3)
            k2 = self.a2*math.sin(self.phi) - self.a3*math.sin(q3)
            q2 = math.atan2(k2*X1 + Y1*k1, k1*X1 - k2*Y1)
            q4 = math.pi/2 - q2 - q3
            print("{0:.3f} {1:.3f} {2:.3f} {3:.3f}".format(q1*180/math.pi, q2*180/math.pi, q3*180/math.pi, q4*180/math.pi))
            self.MGD(q1, q2, q3, q4, t)
        except ValueError:
            print("Position inatteignable")



    def MGD(self, q1, q2, q3, q4, t):

        """
        Fonction utilisant le service "/goal_joint_space_path" permettant d'utiliser le clMGD du robot
        Entrées :
            q1, q2, q3, q4 :
                le tableau des coordonnées angulaires, en radians, à atteindre les coordonnées angulaires sont numérotés par ordre croissant à partir de la base du robot
                exemple : la coordonnées angulaire 1 est celle du moteur fixé au bâti
            t :
                le temps, en secondes, pour atteindre les coordonnées angulaires demandées
        Sorties :
            is_planned :
                booleén indiquant si les coordonnées angulaires peuvent être atteintes par les moteurs 
        """

        # Attente d'une réponse du servide 
        rospy.wait_for_service(self.service_name_MGD)
        try:
            # Création de la fonction pour appeler le service
            joints_service = rospy.ServiceProxy(self.service_name_MGD, SetJointPosition)
            # Création du message à envoyer au service
            arg = SetJointPositionRequest()
            # Renseignement des coordonnées à atteindre 
            arg.joint_position.position = [q1, q2, q3, q4]
            # Renseignement du temps pour atteindre les coordonnées 
            arg.path_time = t
            # Tableau permettant de spécifier à quel moteur s'affecte quelle coordonnée angulaire
            arg.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
            # Appel du service 
            retour = joints_service(arg)
            return retour
        
        # En cas d'erreur, on renvoie "False" pour indiquer que la génération de la consigne a échoué
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
        


    def MGD_relatif(self, q1, q2, q3, q4, t):

        """
        Fonction utilisant le service "/goal_joint_space_path_from_present" permettant de déplacer les moteur relativement à la
        coordonnée angulaire actuelle des moteurs 
        Entrées :
            q1, q2, q3, q4 :
                le tableau des coordonnées angulaires relatives, en radians, à atteindre. Les coordonnées angulaires relatives sont numérotés par ordre croissant à partir de la base du robot
                exemple 1 : la coordonnées angulaire 1 est celle du moteur fixé au bâti
                exemple 2 : si q1 est à pi/2 avant l'appel de la fonction, une consigne de pi/2 produira une position angulaire de q1 de pi/2 + pi/2 = pi radians
            t :
                le temps, en secondes, pour atteindre les coordonnées angulaires demandées
        Sorties :
            is_planned :
                booleén indiquant si les coordonnées angulaires peuvent être atteintes par les moteurs 
        """

        # Attente d'une réponse du servide 
        rospy.wait_for_service(self.service_name_MGD_relatif)
        try:
            # Création de la fonction pour appeler le service
            joints_service = rospy.ServiceProxy(self.service_name_MGD_relatif, SetJointPosition)
            # Création du message à envoyer au service
            arg = SetJointPositionRequest()
            # Renseignement des coordonnées à atteindre 
            arg.joint_position.position = [q1, q2, q3, q4]
            # Renseignement du temps pour atteindre les coordonnées 
            arg.path_time = t
            # Tableau permettant de spécifier à quel moteur s'affecte quelle coordonnée angulaire
            arg.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
            # Appel du service 
            retour = joints_service(arg)
            return retour
        
        # En cas d'erreur, on renvoie "False" pour indiquer que la génération de la consigne a échoué
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False
        



    def deplacement_relatif_moteur(self, angle, moteur, t):

        """
        Fonction utilisant la fonction "MGD_relatif" pour contrôler les moteurs un à un avec des déplacments relatifs
        Entrées :
            angle :
                la coordonnée angulaire, en radians, à atteindre relativement à l'angle actuel du robot 
                exemple : si l'angle du moteur choisi est à pi/2, deplacement_relatif_moteur(pi/2, ..., ...) déplacera le moteur
                à l'angle pi/2 + pi/2 = pi
            moteur :
                chaîne de caractère désignant le moteur à déplacer. On fournit la chaîne "qi" avec i, dans [1;4], l'indice associé au moteur à déplacer, sachant qu'on numérote les moteurs par ordre croissant à partir de la base du robot
                exemple : pour contrôler le moteur 1, le moteur fixé au bâti, on envoie la chaîne "q1"
            t :
                le temps, en secondes, pour atteindre la coordonnée angulaire demandée
        Sorties :
            is_planned :
                booleén indiquant si les coordonnées articulaire peuvent être atteintes par les moteurs 
        """

        if moteur =="q1":
            self.MGD_relatif(angle, 0, 0, 0, t)
        elif moteur == "q2":
            self.MGD_relatif(0, angle, 0, 0, t)
        elif moteur =="q3":
            self.MGD_relatif(0, 0, angle, 0, t)
        elif moteur == "q4":
            self.MGD_relatif(0, 0, 0, angle, t)




    def controle_effecteur(self, angle, t):

        """
        Fonction utilisant le service "/goal_tool_control" pour controler la pince
        Entrées
            angle :
                la coordonnée angulaire, en radians, à atteindre par le moteur contrôlant la pince.
            t :
                le temps, en secondes, pour atteindre la coordonnée angulaire demandée
        Sorties :
            is_planned :
                booleén indiquant si les coordonnées articulaire peuvent être atteintes par les moteurs 
        """

        # Attente d'une réponse du servide 
        rospy.wait_for_service(self.service_name_deplacement_effecteur)
        try:
            # Création de la fonction pour appeler le service
            effector_service = rospy.ServiceProxy(self.service_name_deplacement_effecteur, SetJointPosition)
            # Création du message à envoyer au service
            arg = SetJointPositionRequest()
            # Renseignement du temps pour atteindre les coordonnées, ici imposé à une 1 seconde
            arg.path_time = t
            # Tableau permettant de spécifier à quel moteur s'affecte quelle coordonnée angulaire
            arg.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
            # Ecriture de la coordonnée angulaire pour pouvoir ouvrir la pince
            arg.joint_position.position = [0, 0, 0, 0, angle]
            resp1 = effector_service(arg)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False  
        
    def goto_position_photo(self):
        "Fonction allant à la position permettant de prendre une photo"
        self.MGD(0, -0.8, 0.13, 1.95, 2)
        
    def ouverture_pince(self):
        "Fonction permettant d'ouvrir la pince"
        self.controle_effecteur(0.01, 1)

    def fermeture_pince(self):
        "Fonction permettant de fermer la pince"
        self.controle_effecteur(-0.01, 1)
