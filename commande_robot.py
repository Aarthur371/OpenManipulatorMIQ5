import rospy
from open_manipulator_msgs.srv import *
from open_manipulator_msgs.msg import *

class OpenManipulator :

    def __init__(self):

        # Liste des noms des services pour utiliser les fonctions de cette classe
        self.service_name_MGI = "/goal_task_space_path"#_position_only"
        self.service_name_MGD = "/goal_joint_space_path"
        self.service_name_deplacement_relatif = "/goal_joint_space_path_from_present"
        self.service_name_deplacement_effecteur = "/goal_tool_control"

        # Création d'un souscripteur permettant de récupérer les informations à propos des moteurs
        #rospy.init_node("souscripteurOpenManipulateur")
        #rospy.Subscriber("/joint_states", JointState, self.get_OpenManipulator_data)

        # Variables contenant la force et la position de l'effecteur 
        self.force_effecteur = 0
        self.position_effecteur = 0
        #rospy.spin()

    
    def get_OpenManipulator_data(self, data):
        """
        Fonction de callback appelée lorsque des informations sont publiées sur le topic "/joint_states" le permettant de récupérer la force et la position de l'effecteur 
        """
        self.force_effecteur = data.effort[4]
        self.position_effecteur = data.position[4]
        
    

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
        


    def MGD(self, q1, q2, q3, q4, t):

        """
        Fonction utilisant le service "/goal_joint_space_path" permettant d'utiliser le MGD du robot
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
        


    def deplacement_relatif_moteur(self, angle, moteur, t):

        """
        Fonction utilisant le service "/goal_joint_space_path_from_present" permettant de déplacer les moteur relativement à la
        coordonnée angulaire actuelle des moteurs 
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

        # Attente d'une réponse du servide 
        rospy.wait_for_service(self.service_name_deplacement_relatif)

        try:
            # Création de la fonction pour appeler le service
            relative_joint_service = rospy.ServiceProxy(self.service_name_deplacement_relatif, SetJointPosition)
            # Création du message à envoyer au service
            arg = SetJointPositionRequest()
            # Tableau permettant de spécifier à quel moteur s'affecte quelle coordonnée angulaire
            arg.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
            # Renseignement du temps pour atteindre les coordonnées 
            arg.path_time = t

            # Renseignement du moteur à déplacer en fonction du moteur sélection
            if moteur =="q1":
                arg.joint_position.position = [angle, 0, 0, 0]
            elif moteur == "q2":
                arg.joint_position.position = [0, angle, 0, 0]
            elif moteur =="q3":
                arg.joint_position.position = [0, 0, angle, 0]
            elif moteur == "q4":
                arg.joint_position.position = [0, 0, 0, angle]

            # Appel du service 
            retour = relative_joint_service(arg)
            return retour
        
        # En cas d'erreur, on renvoie "False" pour indiquer que la génération de la consigne a échoué
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False 


    def ouvrir_pince(self, angle, t):

        """
        Fonction utilisant le service "/goal_tool_control" pour ouvrir la pince
        Entrées :
            t :
                le temps, en secondes, pour ouvrir la pince
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
            # Renseignement du temps pour atteindre les coordonnées 
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
        

    """  
    def fermer_pince(self, t):
        rospy.wait_for_service(self.service_name_deplacement_effecteur)
        try:
            effector_service = rospy.ServiceProxy(self.service_name_deplacement_effecteur, SetJointPosition)
            arg = SetJointPositionRequest()
            arg.path_time = t
            arg.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
            arg.joint_position.position = [0, 0, 0, 0, 1]
            resp1 = effector_service(arg)
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return False 
    """