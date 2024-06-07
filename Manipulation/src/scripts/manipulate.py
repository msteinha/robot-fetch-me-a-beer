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
from bagpy import bagreader
import matplotlib.pyplot as plt
import sys

TOOL_LINK_POSE = '/arm_right_tool_link_pose'
MODEL_STATES = '/gazebo/model_states'
GRIPPER_POSE = '/gripper_right_grasping_frame_pose'
GRIPPER_STATUS = '/parallel_gripper_right/is_grasped'
TABLE_CAMERA = '/table_camera/image_raw/compressed'

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

class DmpRos:
    def __init__(self):
        # ROS node initialization
        rospy.init_node("dmp_play")
        
        # ROS parameters
        self.motion = rospy.get_param('/motion', 'grasp')

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

        # DMP parameters
        self.dmp_weights = None
        self.dmp_execution_time = None
        self.dmp_dt = None
        # self.sim_can_pose = None

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
        print("get tiago state...")
        self.listener.waitForTransform(self._base_frame, self._pose_EE_frame, rospy.Time(0), rospy.Duration(5))
        # Update the TIAGo's end-effector and grasping pose
        trans, rot = self.listener.lookupTransform(self._base_frame, self._pose_EE_frame, rospy.Time(0))
        self.pose_EE = np.array([trans[0], trans[1], trans[2], rot[3], rot[0], rot[1], rot[2]]) # pose of right 'arm_right_tool_link' frame in robot base frame 'base_footprint'
        robot_T_EE = pt.transform_from_pq(self.pose_EE)
        EE_T_grasp = pt.transform_from(np.array([[1, 0, 0],[0, 0, -1],[0, 1, 0]]), np.array([0.151, 0, 0]))
        robot_T_grasp = robot_T_EE @ EE_T_grasp
        self.pose_G = pt.pq_from_transform(robot_T_grasp)   # pose of right grasping frame in robot base frame 'base_footprint'
    
    def R_T_EE(self, position, orientation):
        # transformation from grasping frame to end-effector (arm right tool link)
        grasp_T_EE = pt.transform_from(np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]), np.array([-0.151, 0, 0]))

        # transformation from robot base frame to grasping frame
        robot_T_grasp = pt.transform_from_pq(np.array([position[0], position[1], position[2], 
                                                       orientation[0], orientation[1], orientation[2], orientation[3]]))

        # transformation from robot base frame to end-effector (arm right tool link)
        robot_T_EE = robot_T_grasp @ grasp_T_EE

        # pose of end-effector in robot base frame 
        pose_EE = pt.pq_from_transform(robot_T_EE)
        return np.array([pose_EE[0], pose_EE[1], pose_EE[2], pose_EE[3], pose_EE[4], pose_EE[5], pose_EE[6]])

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
        dmp_parameters = np.load(dmp_parameter_path)
        self.dmp_weights = dmp_parameters['weights']
        self.dmp_execution_time = dmp_parameters['duration']
        self.dmp_dt = dmp_parameters['dt']

        self.cartesian_dmp = CartesianDMP(execution_time=self.dmp_execution_time, dt=self.dmp_dt, n_weights_per_dim=10)
        self.cartesian_dmp.set_weights(self.dmp_weights)

    def run_dmp(self, target_position=None):
        self.get_tiago_state()

        n_steps = int(self.dmp_execution_time / self.dmp_dt) + 1
        T = np.linspace(0, self.dmp_execution_time, n_steps)

        if self.motion == 'grasp':
            target_position = self.sim_can_pose #TODO: target_position parameter should be used here (needed for pipeline)
            target_orientation = np.array([0.97967917, 0.00438199, 0.16039803, 0.12034117]) #15-00-43
        
            goal_pose = np.array([target_position[0],
                                target_position[1],
                                target_position[2] + 0.1,
                                target_orientation[0],
                                target_orientation[1],
                                target_orientation[2],
                                target_orientation[3]])
        elif self.motion == 'retrieve':
            # target_position = np.array([0.27094205, -0.4120216, 1.05597785]) # from bag file
            target_orientation = np.array([0.97967917, 0.00438199, 0.16039803, 0.12034117]) #15-00-43

            goal_pose = np.array([target_position[0],
                                  target_position[1],
                                  target_position[2],
                                  target_orientation[0],
                                  target_orientation[1],
                                  target_orientation[2],
                                  target_orientation[3]])
        elif self.motion == 'place':
            # target_position = np.array([0.55, -0.2, 0.63]) # 0.42 -0.32
            target_orientation = np.array([0.97967917, 0.00438199, 0.16039803, 0.12034117])

            goal_pose = np.array([target_position[0],
                                  target_position[1],
                                  target_position[2],
                                  target_orientation[0],
                                  target_orientation[1],
                                  target_orientation[2],
                                  target_orientation[3]])
        else:
            print("wrong action")
            exit(-1)

        # configure dmp and run open loop to get trajectory 
        self.cartesian_dmp.configure(t=T, start_y=self.pose_G, goal_y=goal_pose)
        t_dmp, y_dmp = self.cartesian_dmp.open_loop()

        # Publish the generated trajectory
        for pose in y_dmp:

            # get pose of end-effector
            pose_G = pose
            pose_EE = self.R_T_EE(pose_G[:3], pose_G[3:])

            generated_position = pose_EE[:3]
            generated_orientation = pose_EE[3:]

            # publish pose
            self.publish_pose(self.pub_des_pose, self._base_frame, generated_position, generated_orientation)
            self.rate.sleep()

        # plot_trajectory(y_dmp)

        # self.get_tiago_state()
        # print("initial can pose: " + str(initial_can_pose))
        # print("current can pose: " + str(self.sim_can_pose)) 
        # print("needed EE pose: " + str(self.R_T_EE(goal_pose[:3], goal_pose[3:])))
        # print("estimated EE pose: " + str(self.R_T_EE(y_dmp[-1][:3], goal_pose[3:])))
        # print("actual EE pose: " + str(self.pose_EE))

def load_data(file_path):
    reader = bagreader(file_path + 'grasp.bag')

    poses = []
    timestamps = []

    for topic, msg, ts in reader.reader.read_messages(topics=TOOL_LINK_POSE):
        poses.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        timestamps.append(ts.to_sec())

    poses = np.array(poses)
    timestamps = np.array(timestamps)

    return (poses, timestamps)

def plot_trajectory(poses):
    # Create a figure and 3D axes
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot positions
    ax.plot(poses[:, 0], poses[:, 1], poses[:, 2], label='Position', color='blue')

    # Plot orientation arrows
    for i in range(len(poses)):
        x, y, z = poses[i][:3]
        q_w, q_x, q_y, q_z = poses[i][3:]  # Assuming orientations are quaternion representations
        arrow_length = 0.2  # Length of the arrow representing orientation
        # ax.quiver(x, y, z, quat[1], quat[2], quat[3], length=arrow_length, color='red')
        ax.quiver(x, y, z, q_x, q_y, q_z, length=arrow_length)

    # Set labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Show plot
    plt.show()

if __name__ == '__main__':
    action = sys.argv[1]
    dmp_parameter_path = '/home/user/exchange/Manipulation/2024_ARL_demos_all/_2024-03-14-15-00-43/' + str(action) + '_dmp.npz'
    rospy.set_param('motion', action)

    # Initialize the gripper for the right arm.
    g = Gripper("right")

    # Initialize the DMP (Dynamic Motion Primitive) handling in ROS.
    dmp_ros = DmpRos()

    # Load the DMP parameters from the specified file path
    dmp_ros.load_dmp_parameters(dmp_parameter_path)

    # Set target position
    if dmp_ros.motion == "grasp":
        target_position = None # TODO: sim_can_pose should be used here 
    elif dmp_ros.motion == "retrieve":
        target_position = np.array([0.27094205, -0.4120216, 1.05597785])
    elif dmp_ros.motion == "place":
        target_position = np.array([0.55, -0.2, 0.63])

    # Example stucture for executing dmp motions:
    # Gripper actions to grasp/release at start of motion
    if dmp_ros.motion == "grasp" or dmp_ros.motion == "grab":
        print("Opening gripper")
        g.open()

    # Execute the DMP to perform the desired motion
    dmp_ros.run_dmp(target_position)

    # Gripper actions to grasp/release at end of motion
    if dmp_ros.motion == "grasp" or dmp_ros.motion == "grab":
        print("Closing gripper")
        g.close()
    elif dmp_ros.motion == "place":
        print("Opening gripper")
        g.open()