#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import numpy as np
import pytransform3d.rotations as pr
import random

# poses from world file
table_pose_world            = np.array([2,-3,0, 0,0,0])
aruco_cube_pose_world       = np.array([2,-2.7,0.5, 0,0,0])
tiago_dual_pose_world       = np.array([1.95,-2.3,0.05, 0,0,-1.57])

# generate can grid
first_can_position = np.array([1.9, -3, 0.55, 0, 0, 0])
last_can_position = np.array([1.6, -2.7, 0.55, 0, 0, 0])
n = 9
num_points_per_dim = int(n ** 0.5)
if num_points_per_dim * num_points_per_dim != n:
    raise ValueError("n must be a perfect square")
steps = (last_can_position - first_can_position) / (num_points_per_dim - 1)
can_positions = np.array([first_can_position + np.multiply(steps, [i, j, 0, 0, 0, 0]) 
                    for i in range(num_points_per_dim) for j in range(num_points_per_dim)])

class Gripper:
    def __init__(self,
                 arm,
                 service_grasp='/parallel_gripper_{}_controller/grasp',
                 service_release ='/parallel_gripper_{}_controller/release'):
        
        self._service_grasp = service_grasp.format(arm)
        self._service_release = service_release.format(arm)

        # Get gripper control services
        rospy.wait_for_service(self._service_grasp, timeout=1)
        rospy.wait_for_service(self._service_release, timeout=1)

        self._release_service = rospy.ServiceProxy(self._service_release, Empty)
        self._grasp_service = rospy.ServiceProxy(self._service_grasp, Empty)


    def open(self):
        self._release_service()

    def close(self):
        self._grasp_service()

def reset_world():
    rospy.wait_for_service('/gazebo/reset_world')
    try:
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world()
    except rospy.ServiceException as e:
        rospy.logerr("Reset world service call failed: %s" % e)

def set_model_state(model_name, new_pose):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = new_pose
        resp = set_state(model_state)
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

if __name__ == '__main__':
    rospy.init_node('set_model_state')

    can_pose = rospy.get_param('~can_pose', 1)

    # Detach can from gripper
    g_right = Gripper("right")
    g_right.open()
    g_left = Gripper("left")
    g_left.open()
    rospy.sleep(0.5)

    # Reset the world
    reset_world()

    # Set Aruco cube pose
    aruco_cube_pose_world_quat = pr.quaternion_from_euler(aruco_cube_pose_world[3:], i=0,j=1,k=2, extrinsic=False)
    aruco_cube_pose = Pose()
    aruco_cube_pose.position.x = aruco_cube_pose_world[0]
    aruco_cube_pose.position.y = aruco_cube_pose_world[1]
    aruco_cube_pose.position.z = aruco_cube_pose_world[2]
    aruco_cube_pose.orientation.w = aruco_cube_pose_world_quat[0]
    aruco_cube_pose.orientation.x = aruco_cube_pose_world_quat[1]
    aruco_cube_pose.orientation.y = aruco_cube_pose_world_quat[2]
    aruco_cube_pose.orientation.z = aruco_cube_pose_world_quat[3]
    success = set_model_state('aruco_cube', aruco_cube_pose)

    # Set tiago pose
    tiago_dual_pose_world_quat = pr.quaternion_from_euler(tiago_dual_pose_world[3:], i=0,j=1,k=2, extrinsic=False)
    tiago_dual_pose = Pose()
    tiago_dual_pose.position.x = tiago_dual_pose_world[0]
    tiago_dual_pose.position.y = tiago_dual_pose_world[1]
    tiago_dual_pose.position.z = tiago_dual_pose_world[2]
    tiago_dual_pose.orientation.w = tiago_dual_pose_world_quat[0]
    tiago_dual_pose.orientation.x = tiago_dual_pose_world_quat[1]
    tiago_dual_pose.orientation.y = tiago_dual_pose_world_quat[2]
    tiago_dual_pose.orientation.z = tiago_dual_pose_world_quat[3]
    success = set_model_state('tiago_dual', tiago_dual_pose)

    #  Set can pose
    if can_pose == 0:
        x_min, x_max = can_positions[0,0], can_positions[-1,0]
        y_min, y_max = can_positions[0,1], can_positions[-1,1]
        x_random = random.uniform(x_min, x_max)
        y_random = random.uniform(y_min, y_max)
        coke_can_slim_pose_world = np.array([x_random, y_random, can_positions[0,2], 0, 0, 0])
    elif can_pose <= can_positions.shape[0] and can_pose > 0:
        coke_can_slim_pose_world = can_positions[can_pose-1,:]
    else:
        raise Exception("Unspecified can position.") 

    coke_can_slim_pose_world_quat = pr.quaternion_from_euler(coke_can_slim_pose_world[3:], i=0,j=1,k=2, extrinsic=False)
    coke_can_slim_pose = Pose()
    coke_can_slim_pose.position.x = coke_can_slim_pose_world[0]
    coke_can_slim_pose.position.y = coke_can_slim_pose_world[1]
    coke_can_slim_pose.position.z = coke_can_slim_pose_world[2]
    coke_can_slim_pose.orientation.w = coke_can_slim_pose_world_quat[0]
    coke_can_slim_pose.orientation.x = coke_can_slim_pose_world_quat[1]
    coke_can_slim_pose.orientation.z = coke_can_slim_pose_world_quat[3]
    success = set_model_state('coke_can_slim', coke_can_slim_pose)

    print(f"can pose: {coke_can_slim_pose}")


    print("\nTo get the robot in its home configuration restart the tiago wbc.\n")