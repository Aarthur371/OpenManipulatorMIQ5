import rospy
from open_manipulator_msgs.srv import *

def set_angle_client(angle):
    service_name = '/goal_joint_space_path_from_present'
    rospy.wait_for_service(service_name)
    try:
        set_joint = rospy.ServiceProxy(service_name, SetJointPosition)
        arg = SetJointPositionRequest()
        arg.path_time = 1
        arg.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        arg.joint_position.position = angle
        arg.joint_position.max_accelerations_scaling_factor = 1
        arg.joint_position.max_velocity_scaling_factor = 1
        resp1 = set_joint(arg)
        print('Service done!')
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False
    

def set_relative_angle_client(angle):
    service_name = '/goal_joint_space_path_from_present'
    rospy.wait_for_service(service_name)
    try:
        set_joint = rospy.ServiceProxy(service_name, SetJointPosition)
        arg = SetJointPositionRequest()
        arg.path_time = 1
        arg.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        arg.joint_position.position = angle
        arg.joint_position.max_accelerations_scaling_factor = 1
        arg.joint_position.max_velocity_scaling_factor = 1
        resp1 = set_joint(arg)
        print('Service done!')
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False
        
        
def set_tool_client(angle):
    service_name = '/goal_tool_control'
    rospy.wait_for_service(service_name)
    try:
        set_joint = rospy.ServiceProxy(service_name, SetJointPosition)
        arg = SetJointPositionRequest()
        arg.path_time = 1
        arg.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        arg.joint_position.position = angle
        arg.joint_position.max_accelerations_scaling_factor = 1
        arg.joint_position.max_velocity_scaling_factor = 1
        resp1 = set_joint(arg)
        print('Service done!')
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False     

response = set_tool_client([0.0, 0.0, 0.0, 0.0, -0.05])