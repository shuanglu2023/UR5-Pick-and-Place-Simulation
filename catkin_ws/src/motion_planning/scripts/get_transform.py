#!/home/ziyu/shuang/hand_object_interaction/.venv/bin/python3
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates, LinkStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
import numpy as np

def get_matrix(trans,rot):
    T = quaternion_matrix([rot[0],rot[1],rot[2],rot[3]])
    T[:3,3] = np.array([trans[0],trans[1],trans[2]])
    return T

# Resting orientation of the end effector
DEFAULT_QUAT = (0,0,0,1)
# Resting position of the end effector
DEFAULT_POS = (-0.1, -0.5, 1.2)

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    legos = ModelStates()

    for name, pose in zip(models.name, models.pose):
        # if "_1" not in name or "Box" in name:
        #     continue
        if "_1" not in name:
            continue
        T_base_obj = get_matrix([pose.position.x,pose.position.y,pose.position.z-0.743],[pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
        print(f'{name} position in base frame {T_base_obj[0,3],T_base_obj[1,3],T_base_obj[2,3]}')
        print(f'{name} orientation {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w}')
        try:
            # base in camera link
            (trans,rot) = listener.lookupTransform('/cam_link', '/base', rospy.Time(0))
            T_cam_base = get_matrix(trans,rot)
            (trans,rot) = listener.lookupTransform('/base', '/cam_link', rospy.Time(0))
            T_base_cam = get_matrix(trans,rot)
            print(f'T_base_cam')
            print(f'{T_base_cam}')

            result = np.matmul(T_cam_base, T_base_obj)
            print(f'{name} T_cam_obj')
            print(f'{result}')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    rospy.sleep(0.4)
