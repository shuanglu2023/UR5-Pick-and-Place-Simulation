#!/home/ziyu/shuang/hand_object_interaction/.venv/bin/python3

import os
import math
import copy
import json
import actionlib
import control_msgs.msg
from controller import ArmController
from gazebo_msgs.msg import ModelStates, LinkStates
import rospy
from pyquaternion import Quaternion as PyQuaternion
import numpy as np
from gazebo_ros_link_attacher.srv import SetStatic, SetStaticRequest, SetStaticResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import pandas as pd
import tf
from tf.transformations import quaternion_matrix, quaternion_from_matrix

PKG_PATH = os.path.dirname(os.path.abspath(__file__))

MODELS_INFO = {
    "cuboid_1": {
        "home": [-0.07979586779896053, -0.6490006390379754, 0.74993510842652]
    },
    "octagon_1": {
        "home": [-0.059488238802733504, -0.5609799975331256, 0.7499096900791358]
    },
    "parallelogram_1": {
        "home": [-0.011351652074722551, -0.5820653011775399, 0.7498989574661152]
    },
    "star_1": {
        "home": [0.009039494191138434, -0.6292100695901965, 0.7499096795147189]
    }
}


# Compensate for the interlocking height
INTERLOCKING_OFFSET = 0.019

SAFE_X = -0.40
SAFE_Y = -0.13
SURFACE_Z = 0.774

# Resting orientation of the end effector
DEFAULT_QUAT = PyQuaternion(axis=(0, 1, 0), angle=math.pi)

# Resting position of the end effector
DEFAULT_POS = (-0.1, -0.5, 1.2)

DEFAULT_PATH_TOLERANCE = control_msgs.msg.JointTolerance()
DEFAULT_PATH_TOLERANCE.name = "path_tolerance"
DEFAULT_PATH_TOLERANCE.velocity = 10

def get_gazebo_model_name(model_name, vision_model_pose):
    """
        Get the name of the model inside gazebo. It is needed for link attacher plugin.
    """
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    epsilon = 0.05
    for gazebo_model_name, model_pose in zip(models.name, models.pose):
        if model_name not in gazebo_model_name:
            continue
        # Get everything inside a square of side epsilon centered in vision_model_pose
        ds = abs(model_pose.position.x - vision_model_pose.position.x) + abs(model_pose.position.y - vision_model_pose.position.y)
        if ds <= epsilon:
            return gazebo_model_name
    raise ValueError(f"Model {model_name} at position {vision_model_pose.position.x} {vision_model_pose.position.y} was not found!")


def get_model_name(gazebo_model_name):
    return gazebo_model_name.replace("lego_", "").split("_", maxsplit=1)[0]


def get_legos_pos(vision=False):
    #get legos position reading vision topic
    if vision:
        legos = rospy.wait_for_message("/lego_detections", ModelStates, timeout=None)
    else:
        models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
        legos = ModelStates()
        for name, pose in zip(models.name, models.pose):
            if "_1" not in name or "Box" in name:
                continue
            name = get_model_name(name)
            legos.name.append(name)
            legos.pose.append(pose)
    return [(lego_name, lego_pose) for lego_name, lego_pose in zip(legos.name, legos.pose)]


def run_task(model_pose, gazebo_model_name):
    x = model_pose.position.x
    y = model_pose.position.y
    z = model_pose.position.z
    model_quat = PyQuaternion(
        x=model_pose.orientation.x,
        y=model_pose.orientation.y,
        z=model_pose.orientation.z,
        w=model_pose.orientation.w)

    print(f'controller gripper pose {controller.gripper_pose}')
    # (x, y, z), quat = controller.gripper_pose
    # Get above the object
    print(f'move to x {x}, y {y} z {z+0.1}')
    controller.move_to(x, y, z+0.1, target_quat=DEFAULT_QUAT)



def close_gripper(gazebo_model_name, closure=0):
    set_gripper(0.81-closure*10)
    rospy.sleep(0.5)
    # Create dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        attach_srv.call(req)


def open_gripper(gazebo_model_name=None):
    # set_gripper(0.0)
    set_gripper(0.2)
    # Destroy dynamic joint
    if gazebo_model_name is not None:
        req = AttachRequest()
        req.model_name_1 = gazebo_model_name
        req.link_name_1 = "link"
        req.model_name_2 = "robot"
        req.link_name_2 = "wrist_3_link"
        detach_srv.call(req)


def set_model_fixed(model_name):
    req = AttachRequest()
    req.model_name_1 = model_name
    req.link_name_1 = "link"
    req.model_name_2 = "ground_plane"
    req.link_name_2 = "link"
    attach_srv.call(req)

    req = SetStaticRequest()
    print("{} TO HOME".format(model_name))
    req.model_name = model_name
    req.link_name = "link"
    req.set_static = True

    setstatic_srv.call(req)


def get_approach_quat(facing_direction, approach_angle):
    quat = DEFAULT_QUAT
    if facing_direction == (0, 0, 1):
        pitch_angle = 0
        yaw_angle = 0
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        pitch_angle = + 0.2
        if abs(approach_angle) < math.pi/2:
            yaw_angle = math.pi/2
        else:
            yaw_angle = -math.pi/2
    elif facing_direction == (0, 0, -1):
        pitch_angle = 0
        yaw_angle = 0
    else:
        raise ValueError(f"Invalid model state {facing_direction}")

    quat = quat * PyQuaternion(axis=(0, 1, 0), angle=pitch_angle)
    quat = quat * PyQuaternion(axis=(0, 0, 1), angle=yaw_angle)
    quat = PyQuaternion(axis=(0, 0, 1), angle=approach_angle+math.pi/2) * quat

    return quat


def get_axis_facing_camera(quat):
    axis_x = np.array([1, 0, 0])
    axis_y = np.array([0, 1, 0])
    axis_z = np.array([0, 0, 1])
    new_axis_x = quat.rotate(axis_x)
    new_axis_y = quat.rotate(axis_y)
    new_axis_z = quat.rotate(axis_z)
    # get angle between new_axis and axis_z
    angle = np.arccos(np.clip(np.dot(new_axis_z, axis_z), -1.0, 1.0))
    # get if model is facing up, down or sideways
    if angle < np.pi / 3:
        return 0, 0, 1
    elif angle < np.pi / 3 * 2 * 1.2:
        if abs(new_axis_x[2]) > abs(new_axis_y[2]):
            return 1, 0, 0
        else:
            return 0, 1, 0
        #else:
        #    raise Exception(f"Invalid axis {new_axis_x}")
    else:
        return 0, 0, -1


def get_approach_angle(model_quat, facing_direction):#get gripper approach angle
    if facing_direction == (0, 0, 1):
        return model_quat.yaw_pitch_roll[0] - math.pi/2 #rotate gripper
    elif facing_direction == (1, 0, 0) or facing_direction == (0, 1, 0):
        axis_x = np.array([0, 1, 0])
        axis_y = np.array([-1, 0, 0])
        new_axis_z = model_quat.rotate(np.array([0, 0, 1])) #get z axis of lego
        # get angle between new_axis and axis_x
        dot = np.clip(np.dot(new_axis_z, axis_x), -1.0, 1.0) #sin angle between lego z axis and x axis in fixed frame
        det = np.clip(np.dot(new_axis_z, axis_y), -1.0, 1.0) #cos angle between lego z axis and x axis in fixed frame
        return math.atan2(det, dot) #get angle between lego z axis and x axis in fixed frame
    elif facing_direction == (0, 0, -1):
        return -(model_quat.yaw_pitch_roll[0] - math.pi/2) % math.pi - math.pi
    else:
        raise ValueError(f"Invalid model state {facing_direction}")


def set_gripper(value):
    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value  # From 0.0 to 0.8
    goal.command.max_effort = 100  # # Do not limit the effort
    action_gripper.send_goal_and_wait(goal, rospy.Duration(10))

    return action_gripper.get_result()

def T_inv(T_in):
    R_in = T_in[:3,:3]
    t_in = T_in[:3,[-1]]
    R_out = R_in.T
    t_out = -np.matmul(R_out,t_in)
    return np.vstack((np.hstack((R_out,t_out)),np.array([0, 0, 0, 1])))

def get_matrix(trans):
    R = quaternion_matrix([0,0,0,1])
    R[:3,3] = np.array([trans[0],trans[1],trans[2]])
    return R


if __name__ == "__main__":
    print("Initializing node of kinematics")
    rospy.init_node("send_joints")

    controller = ArmController()

    # Create an action client for the gripper
    action_gripper = actionlib.SimpleActionClient(
        "/gripper_controller/gripper_cmd",
        control_msgs.msg.GripperCommandAction
    )
    print("Waiting for action of gripper controller")
    action_gripper.wait_for_server()

    setstatic_srv = rospy.ServiceProxy("/link_attacher_node/setstatic", SetStatic)
    attach_srv = rospy.ServiceProxy("/link_attacher_node/attach", Attach)
    detach_srv = rospy.ServiceProxy("/link_attacher_node/detach", Attach)
    setstatic_srv.wait_for_service()
    attach_srv.wait_for_service()
    detach_srv.wait_for_service()

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)
    open_gripper()
    print("Waiting for detection of the models")
    rospy.sleep(0.5)
    legos = get_legos_pos(vision=False)
    legos.sort(reverse=True, key=lambda a: (a[1].position.x, a[1].position.y))

    T_base_cam = [[0, -9.99999999e-01, -4.09517666e-05, -1.00002652e-01],
                [-9.99998712e-01,  1.27581534e-06,  1.60473041e-03, -4.99926988e-01],
                [-1.60473036e-03,  4.09538666e-05, -9.99998712e-01,  5.95224385e-01],
                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]

    listener = tf.TransformListener()

    models = ['cuboid','star','parallelogram','octagon']
    # models = ['parallelogram']
    for model_name in models:
        # model_name = 'cuboid'
        # reach & grasp
        # try position generated in dmp in camera coordinates
        file_path = f"/home/ziyu/semantic_programming/{model_name}_reach.csv"
        df=pd.read_csv(file_path,header=None)
        for index, row in df.iterrows():
            # print(f'dmp points {row.values[0]} {row.values[1]} {row.values[2]}')
            T_cam_obj = get_matrix(trans=[row.values[0],row.values[1],row.values[2]])
            T_base_obj = np.matmul(T_base_cam,T_cam_obj)
            # T_world_obj = np.matmul(T_cam_obj,T_inv(np.array(T_cam_world)))
            x = T_base_obj[0,3]
            y = T_base_obj[1,3]
            z = T_base_obj[2,3] + 0.75
            # print(f'move to {x} {y} {z}')
            if model_name == 'parallelogram':
                quat = PyQuaternion(axis=(0, 0, 1), angle=math.radians(15))
                quat = DEFAULT_QUAT * quat
                # print(f'quat {quat}')
                y = y + 0.01
                controller.move_to(x,y,z,target_quat=quat)
            else:
                controller.move_to(x,y,z,target_quat=DEFAULT_QUAT)
        
        # move down to grasp
        z = z - 0.15
        controller.move_to(x,y,z)
        gazebo_model_name = model_name + "_1"

        close_gripper(gazebo_model_name,0.04)
        z = z + 0.2
        controller.move_to(x,y,z,target_quat=DEFAULT_QUAT)

        # move & release
        file_path = f"/home/ziyu/semantic_programming/{model_name}_move.csv"
        df=pd.read_csv(file_path,header=None)
        for index, row in df.iterrows():
            T_cam_obj = get_matrix(trans=[row.values[0],row.values[1],row.values[2]])
            T_base_obj = np.matmul(T_base_cam,T_cam_obj)
            x = T_base_obj[0,3]
            y = T_base_obj[1,3]
            z = T_base_obj[2,3] + 0.75
            # print(f'move to {x} {y} {z}')
            controller.move_to(x,y,z)
        
        # move down to release
        z = z - 0.1
        controller.move_to(x,y,z,target_quat=DEFAULT_QUAT)
        print(f'relase position arrived')
        open_gripper(gazebo_model_name)
        rospy.sleep(1)
        z = z + 0.2
        controller.move_to(x,y,z)

        # rospy.sleep(0.4)

    controller.move_to(*DEFAULT_POS, DEFAULT_QUAT)