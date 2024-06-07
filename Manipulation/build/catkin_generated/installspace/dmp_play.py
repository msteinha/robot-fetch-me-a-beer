#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from movement_primitives.dmp import CartesianDMP
import pytransform3d.transformations as pt
import tf
from std_srvs.srv import Empty
import rospkg

class Gripper:
    def __init__(self,
                 arm,
                 service_grasp='/parallel_gripper_{}_controller/grasp',
                 service_release ='/parallel_gripper_{}_controller/release'):
        
        # Initialize the gripper control services
        self._service_grasp = service_grasp.format(arm)
        self._service_release = service_release.format(arm)

        # Get gripper control services
        rospy.wait_for_service(self._service_grasp, timeout=1)
        rospy.wait_for_service(self._service_release, timeout=1)

        # Create service proxies
        self._release_service = rospy.ServiceProxy(self._service_release, Empty)
        self._grasp_service = rospy.ServiceProxy(self._service_grasp, Empty)


    def open(self):
        self._release_service()

    def close(self):
        self._grasp_service()

class DMP_ROS:
    def __init__(self):
        # ROS node initialization
        rospy.init_node("dmp_play")
        
        # ROS parameters
        self.motion = rospy.get_param('~motion', 'grab')

        print(f"Playing motion '{self.motion}'.\n")

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('arl_manipulation_baseline')
        
        # TF frames
        self._base_frame = "base_footprint"
        self._pose_EE_frame = "arm_right_tool_link"
        self.listener = tf.TransformListener()
        
        # ROS subscribers and publishers
        self.sub_sim_state = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_cb, queue_size=10)
        self.pub_des_pose = rospy.Publisher('/whole_body_kinematic_controller/arm_right_tool_link_goal', PoseStamped, queue_size=10)

        # ROS rate setup
        self.ros_rate = 500
        self.rate = rospy.Rate(self.ros_rate)

    def publish_pose(self, pub, base_frame, position, orientation):
        # Publish a stamped pose message to agiven topic

        # Create and populate a PoseStamped message
        msg = PoseStamped()
        msg.header.frame_id = base_frame
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]

        msg.pose.orientation.w = orientation[0]
        msg.pose.orientation.x = orientation[1]
        msg.pose.orientation.y = orientation[2]
        msg.pose.orientation.z = orientation[3]

        # Publish the pose message
        pub.publish(msg)

    def get_tiago_state(self):
        # Update the TIAGo's end-effector and grasping pose
        trans, rot = self.listener.lookupTransform(self._base_frame, self._pose_EE_frame, rospy.Time(0))
        self.pose_EE = np.array([trans[0], trans[1], trans[2], rot[3], rot[0], rot[1], rot[2]]) # pose of right 'arm_right_tool_link' frame in robot base frame 'base_footprint'
        robot_T_EE = pt.transform_from_pq(self.pose_EE)
        EE_T_grasp = pt.transform_from(np.array([[1, 0, 0],[0, 0, -1],[0, 1, 0]]), np.array([0.151, 0, 0]))
        robot_T_grasp = robot_T_EE @ EE_T_grasp
        self.pose_G = pt.pq_from_transform(robot_T_grasp)   # pose of right grasping frame in robot base frame 'base_footprint'
    
    def gazebo_cb(self, data):
        # Callback function for Gazebo model states
        index_can = data.name.index("coke_can_slim")
        can_pose = np.array([data.pose[index_can].position.x, data.pose[index_can].position.y, data.pose[index_can].position.z,
                                      data.pose[index_can].orientation.w, data.pose[index_can].orientation.x, data.pose[index_can].orientation.y, data.pose[index_can].orientation.z])
        index_robot = data.name.index("tiago_dual")
        robot_pose = np.array([data.pose[index_robot].position.x, data.pose[index_robot].position.y, data.pose[index_robot].position.z,
                                      data.pose[index_robot].orientation.w, data.pose[index_robot].orientation.x, data.pose[index_robot].orientation.y, data.pose[index_robot].orientation.z])
        
        w_T_robot = pt.transform_from_pq(robot_pose)
        w_T_can = pt.transform_from_pq(can_pose)
        
        robot_T_can = np.linalg.inv(w_T_robot) @ w_T_can
            
        self.sim_can_pose = pt.pq_from_transform(robot_T_can)  # pose of can in robot base frame 'base_footprint'

    def load_dmp_parameters(self, dmp_parameter_path):
       # Load DMP parameters
        pass

    def run_dmp(self):
        # Execute the DMP trajectory
        pass            

if __name__ == '__main__':
    # Initialize the gripper for the right arm.
    g = Gripper("right")

    # Initialize the DMP (Dynamic Motion Primitive) handling in ROS.
    dmp_ros = DMP_ROS()


    # Load the DMP parameters from the specified file path
    dmp_ros.load_dmp_parameters(dmp_parameter_path)

    # Example stucture for executing dmp motions:
    # Gripper actions to grasp/release at start of motion
    if dmp_ros.motion == "grasp" or dmp_ros.motion == "grab":
        print("Opening gripper")
        g.open()

    # Execute the DMP to perform the desired motion
    dmp_ros.run_dmp()

    # Gripper actions to grasp/release at end of motion
    if dmp_ros.motion == "grasp" or dmp_ros.motion == "grab":
        print("Closing gripper")
        g.close()
    elif dmp_ros.motion == "place":
        print("Opening gripper")
        g.open()

    