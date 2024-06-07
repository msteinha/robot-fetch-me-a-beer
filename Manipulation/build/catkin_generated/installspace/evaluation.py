#!/usr/bin/env python3
import numpy as np
import os
import rospkg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
from bagpy import bagreader
import rosbag
import pandas as pd
import pytransform3d.rotations as pr
import pytransform3d.transformations as pt
from arl_manipulation_baseline.smoothness import sparc


def flip_quaternions(quat, ref_quat=[1, 0, 0, 0]):
    return -quat if np.dot(ref_quat, quat) < 0 else quat

def flip_quaternion_trajectory(quat_traj):
    transposed = quat_traj.shape[0] != 4
    if transposed:
        quat_traj = quat_traj.T  # Transpose to (4, n) for consistency
    
    flipped_quat_traj = np.empty_like(quat_traj)
    flipped_quat_traj[:, 0] = quat_traj[:, 0]
    for i in range(1, quat_traj.shape[1]):
        flipped_quat_traj[:, i] = flip_quaternions(quat_traj[:, i], flipped_quat_traj[:, i - 1])

    return flipped_quat_traj.T if transposed else flipped_quat_traj

def load_bag(file_path):
    print("loading bag file...")
    bag = bagreader(file_path)
    tiago_gripper_pose_df = pd.read_csv(bag.message_by_topic('/gripper_right_grasping_frame_pose'))
    time = np.array(tiago_gripper_pose_df['Time']) - tiago_gripper_pose_df['Time'][0]
    pose = tiago_gripper_pose_df[['pose.position.x', 'pose.position.y', 'pose.position.z', 
                                'pose.orientation.w', 'pose.orientation.x', 'pose.orientation.y', 'pose.orientation.z']].values
    
    # Remove duplicate measurements
    initial_length = len(pose)
    diffs = np.linalg.norm(np.diff(pose, axis=0), axis=1)
    changed_indices = np.insert(np.where(diffs > 0)[0] + 1, 0, 0)
    print(f"Number of duplicate measurements: {initial_length - len(changed_indices)}")
    duplicate_indices = [i for i in range(initial_length) if i not in changed_indices]
    print(f"Original length: {initial_length}")
    print(f"Indices of duplicates: {duplicate_indices}")

    # Correct quaternion flipping
    pose[:,3:] = flip_quaternion_trajectory(pose[:,3:])

    data = {'pose': pose, 'time': time}
    
    # gazebo          
    poses_can = []
    poses_table = []
    print("iterating over all messages")
    with rosbag.Bag(file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states']):
            # Assuming 'your_topic_name' is the topic where model states are published
            if 'coke_can_slim' in msg.name:
                index_obj_c = msg.name.index('coke_can_slim')
                # Extract the pose and append it to the list
                pose_c = np.array([msg.pose[index_obj_c].position.x, 
                                msg.pose[index_obj_c].position.y, 
                                msg.pose[index_obj_c].position.z,
                                msg.pose[index_obj_c].orientation.w, 
                                msg.pose[index_obj_c].orientation.x, 
                                msg.pose[index_obj_c].orientation.y, 
                                msg.pose[index_obj_c].orientation.z])
            if 'tiago_dual' in msg.name:
                index_obj_r = msg.name.index('tiago_dual')
                # Extract the pose and append it to the list
                pose_r = np.array([msg.pose[index_obj_r].position.x, 
                                msg.pose[index_obj_r].position.y, 
                                msg.pose[index_obj_r].position.z,
                                msg.pose[index_obj_r].orientation.w, 
                                msg.pose[index_obj_r].orientation.x, 
                                msg.pose[index_obj_r].orientation.y, 
                                msg.pose[index_obj_r].orientation.z])
            if 'table' in msg.name:
                index_obj_t = msg.name.index('table')
                # Extract the pose and append it to the list
                pose_t = np.array([msg.pose[index_obj_t].position.x, 
                                msg.pose[index_obj_t].position.y, 
                                msg.pose[index_obj_t].position.z,
                                msg.pose[index_obj_t].orientation.w, 
                                msg.pose[index_obj_t].orientation.x, 
                                msg.pose[index_obj_t].orientation.y, 
                                msg.pose[index_obj_t].orientation.z])

            # processing data
            robot_pose_can = gazebo_data_processing(pose_r, pose_c) 
            poses_can.append(robot_pose_can)
            robot_pose_table = gazebo_data_processing(pose_r, pose_t) 
            poses_table.append(robot_pose_table)

    return data, np.array(poses_can), np.array(poses_table)

def gazebo_data_processing(robot_pose, obj_pose):
    
    w_T_robot = pt.transform_from_pq(robot_pose)
    w_T_obj = pt.transform_from_pq(obj_pose)
    robot_T_obj = np.linalg.inv(w_T_robot) @ w_T_obj

    return pt.pq_from_transform(robot_T_obj)

def evaluate(data, can_data, table_data, motion):
    traj = data["pose"]
    time = data["time"]
    dt = np.mean(np.diff(time))
    duration = time[-1]
    speed = np.diff(traj[:,:3], axis=0)/dt
    absSpeed = np.linalg.norm(speed, axis=1)
    sparc_val, f, f_sel = sparc(movement=absSpeed, fs=1/dt, padlevel=4, fc=10.0, amp_th=0.05)

    success = check_success(traj, can_data, table_data, motion)

    return success, duration, sparc_val

def check_success(data, can_data, table_data, motion="grab"):
    success = 0
    # for grab check: distance gripper to can
    if motion == "grab":
        print("Checking success of grabbing motion")
        if np.linalg.norm(data[-1,:3] - can_data[-1,:3]) < 0.05:
            print(np.linalg.norm(data[-1,:3] - can_data[-1,:3]))
            success = 1
    # for place check: check if bottle upright (direction can z-axis up with less than 20 degree tilt) and bottle on table (bottle z pos > table) 
    elif motion == "place":
        print("Checking success of placing motion")
        R_can = pr.matrix_from_quaternion(can_data[-1,3:])
        can_up = R_can[:, 2]
        can_upright = np.dot(can_up, [0, 0, 1]) > np.cos(np.radians(20))
        diff_can_table_top = can_data[-1,2] - (table_data[-1,2]+0.4)
        can_on_table = diff_can_table_top > 0 and diff_can_table_top < 0.15
        print(np.degrees(np.arccos(np.dot(can_up, [0, 0, 1]))))
        print(can_data[-1,2] - (table_data[-1,2]+0.4))
        if can_upright and can_on_table and diff_can_table_top:
            success = 1
    elif motion == "retrieve":
        print("Checking success of retrieving motion")
        start_grabbed = np.linalg.norm(data[0,:3] - can_data[0,:3]) < 0.05
        end_grabbed = np.linalg.norm(data[-1,:3] - can_data[-1,:3]) < 0.05
        if start_grabbed and end_grabbed:
            print(np.linalg.norm(data[0,:3] - can_data[0,:3]))
            print(np.linalg.norm(data[-1,:3] - can_data[-1,:3]))
            success = 1

    return success

def visualize_gripper_poses(pose_data):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(pose_data[:,0], pose_data[:,1], pose_data[:,2], marker='o')
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    ax.set_zlabel('Z position')
    plt.show()

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('arl_manipulation_baseline')
    
    eval_data_path = os.path.join(package_path, "data", "eval_data")

    results = [] 

    # Loop through all files in the eval_data_path
    for file in sorted(os.listdir(eval_data_path)):
        file_path = os.path.join(eval_data_path, file)
        if file_path.endswith('.bag'):
            if "grab" in file:
                motion_type = "grab"
            elif "retrieve" in file:
                motion_type = "retrieve"
            elif "place" in file:
                motion_type = "place"
            else:
                raise ValueError(f"Unknown motion type for file: {file}")
            print(f"motion type: {motion_type}")
            data, can_data, table_data = load_bag(file_path)
            # visualize_gripper_poses(data['pose'])
            success, duration, sparc_val = evaluate(data, can_data, table_data, motion=motion_type)
            print(f"{file}: success={success}, duration={duration:.3f}, smoothness={sparc_val:.3f}")
            results.append((motion_type, success, duration, sparc_val))
    
    eval_result_path = os.path.join(package_path, "data", "eval_result")
    if not os.path.exists(eval_result_path):
        os.makedirs(eval_result_path)
    np_results = np.array(results)
    np.save(os.path.join(eval_result_path, 'evaluation_results.npy'), np_results)