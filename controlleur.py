
import sys
import rospy
from open_manipulator_msgs.srv import *
import cv2
import os
import numpy as np

def set_position_client(x, y, z, time):
    service_name = '/goal_task_space_path_position_only'

    rospy.wait_for_service(service_name)

    try:
        set_position = rospy.ServiceProxy(service_name, SetKinematicsPose)

        arg = SetKinematicsPoseRequest()
        arg.end_effector_name = 'gripper'
        arg.kinematics_pose.pose.position.x = x
        arg.kinematics_pose.pose.position.y = y
        arg.kinematics_pose.pose.position.z = z
        arg.path_time = time
        resp1 = set_position(arg)
        print('Service done!')
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False

horizontal_res = 640
vertical_res = 480

# open camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_res)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_res)

try:
    while(True):
        reponse = input("Prise de photo -> p nom_photo\nDéplacemet du robot (en cm) -> d x y z\n")
        if reponse != "":
            tab_reponse = reponse.split(' ')
            if tab_reponse[0] == "p":
                ret, frame = cap.read()
                cv2.imwrite(tab_reponse[1] + '.jpg', frame)
                print("Photo prise")
            elif tab_reponse[0] == "d":
                response = set_position_client(tab_reponse[1]/100, tab_reponse[2]/100, tab_reponse[3]/100, 1)
            else:
                print("Commande non reconnue")
        else:
            print("Commande vide")
except KeyboardInterrupt:
    print("Fin du programme")
    cap.release()