#!/usr/bin/env python3
import numpy as np
from bagpy import bagreader
import pandas as pd
import rosbag
import os
import matplotlib.pyplot as plt

TOOL_LINK_POSE = '/arm_right_tool_link_pose'
MODEL_STATES = '/gazebo/model_states'
GRIPPER_POSE = '/gripper_right_grasping_frame_pose'
GRIPPER_STATUS = '/parallel_gripper_right/is_grasped'
TABLE_CAMERA = '/table_camera/image_raw/compressed'

# model_states indices
GROUND_PLANE_IDX = 0
TABLE_IDX = 2
CAN_POSE_IDX = 4
TIAGO_DUAL_IDX = 6

DISTANCE_THRESHOLD = 1e-2

def main():
    bag_file = 'C:/Users/morit/Documents/Advanced_Robot_Learning/Manipulation/2024_ARL_demos_all/_2024-03-14-15-00-43.bag'

    save_path = bag_file.split('.')[0] + '/'

    # Load data
    data = load_data(bag_file)

    # Process and segment the data
    (ts_grasping_done, ts_retrieving_done) = process_data(data)
    
    # Save the segmented data
    save_data(data, ts_grasping_done, ts_retrieving_done, save_path)

def load_data(file):    
    return bagreader(file)

def process_data(data):
    initial_can_pos = None
    ts_grasping_done = None
    ts_retrieving_done = None
    ts_max_can_distance = None
    max_can_distance = None

    # get initial can pose
    model_states = data.reader.read_messages(topics=MODEL_STATES)
    initial_model_states = list(model_states)[0]
    initial_can_pos = initial_model_states.message.pose[CAN_POSE_IDX].position
    initial_can_pos = np.array([initial_can_pos.x, initial_can_pos.y, initial_can_pos.z])

    # get timestamp when gripper is closed (grasping)
    for topic, msg, ts in data.reader.read_messages(topics=GRIPPER_STATUS):
        if msg.data == True:
            ts_grasping_done = ts
            break

    for topic, msg, ts in data.reader.read_messages(topics=MODEL_STATES):
        if ts < ts_grasping_done:
            continue

        # calculate current distance of can from initial can position
        current_can_pos = np.array([msg.pose[CAN_POSE_IDX].position.x, msg.pose[CAN_POSE_IDX].position.y, msg.pose[CAN_POSE_IDX].position.z])
        current_can_distance = np.linalg.norm(current_can_pos - initial_can_pos)

        # special handling for first loop
        if max_can_distance is None:
            max_can_distance = current_can_distance
            continue

        # set new max distance
        if current_can_distance >= max_can_distance:
            max_can_distance = current_can_distance
            ts_max_can_distance = ts

    ts_retrieving_done = ts_max_can_distance

    # check if timestamps found
    if ts_grasping_done is None or ts_retrieving_done is None:
        print("Error: Timestamp(s) not found! Check bag file!")
        
    # print("timestamp grasp end: " + str(ts_grasping_done))
    # print("timestamp retrieve end: " + str(ts_retrieving_done))

    return (ts_grasping_done, ts_retrieving_done)

def save_data(data, ts_grasping_done, ts_retrieving_done, save_path):
    grasp_poses = np.empty((0, 7))
    grasp_timestamps = np.empty((0,))
    retrieve_poses = np.empty((0, 7))
    retrieve_timestamps = np.empty((0,))
    place_poses = np.empty((0, 7))
    place_timestamps = np.empty((0,))
    last_pose = None

    # write messages to correct npz files
    for topic, msg, ts in data.reader.read_messages(topics=GRIPPER_POSE):

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        w = msg.pose.orientation.w
        o_x = msg.pose.orientation.x
        o_y = msg.pose.orientation.y
        o_z = msg.pose.orientation.z

        # calculate dot product to last pose
        if last_pose is not None:
            dot_product = np.dot(np.array([o_x, o_y, o_z]), last_pose[4:])

            # if orientations not align -> flip orientation
            if dot_product < 0:
                o_x = -o_x
                o_y = -o_y
                o_z = -o_z

        if ts <= ts_grasping_done:
            grasp_poses = np.vstack((grasp_poses, [x, y, z, w, o_x, o_y, o_z]))
            grasp_timestamps = np.append(grasp_timestamps, ts.to_sec())
        elif ts <= ts_retrieving_done:
            retrieve_poses = np.vstack((retrieve_poses, [x, y, z, w, o_x, o_y, o_z]))
            retrieve_timestamps = np.append(retrieve_timestamps, ts.to_sec())
        else:
            place_poses = np.vstack((place_poses, [x, y, z, w, o_x, o_y, o_z]))
            place_timestamps = np.append(place_timestamps, ts.to_sec())

        last_pose = np.array([x, y, z, w, o_x, o_y, o_z])

    # print(grasp_poses[-1])
    # print(retrieve_poses[-1])
    print(place_poses[-1])

    # plot_trajectory(grasp_poses)    
    # plot_trajectory(retrieve_poses)    
    plot_trajectory(place_poses)    
    
    if 0:
        grasp_file = rosbag.Bag('grasp.bag', 'w')
        retrieve_file = rosbag.Bag('retrieve.bag', 'w')
        place_file = rosbag.Bag('place.bag', 'w')
        for topic, msg, ts in data.reader.read_messages(topics=TABLE_CAMERA):
            if ts <= ts_grasping_done:
                grasp_file.write('/table_camera/image_raw/compressed', msg)
            elif ts <= ts_retrieving_done:
                retrieve_file.write('/table_camera/image_raw/compressed', msg)
            else:
                place_file.write('/table_camera/image_raw/compressed', msg)

        grasp_file.close()
        retrieve_file.close()
        place_file.close()

    np.savez(save_path + 'grasp.npz', pose=grasp_poses, time=grasp_timestamps)
    np.savez(save_path + 'retrieve.npz', pose=retrieve_poses, time=retrieve_timestamps)
    np.savez(save_path + 'place.npz', pose=place_poses, time=place_timestamps)

def plot_trajectory(poses):
    # Create a figure and 3D axes
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot positions
    ax.plot(poses[:, 0], poses[:, 1], poses[:, 2], label='Position', color='blue')

    last_pose = None

    # Plot orientation arrows
    for i in range(len(poses)):
        x, y, z = poses[i][:3]
        q_w, q_x, q_y, q_z= poses[i][3:]
        arrow_length = 0.2
        dot_product = 0

        # calculate dot product to last pose
        if last_pose is not None:
            dot_product = np.dot(np.array([q_x, q_y, q_z]), last_pose[4:])

            # # if orientations not align -> flip orientation
            # if dot_product < 0:
            #     o_x = -o_x
            #     o_y = -o_y
            #     o_z = -o_z

        if dot_product < 0:
             ax.quiver(x, y, z, q_x, q_y, q_z, length=arrow_length, color='red')
        else:
            ax.quiver(x, y, z, q_x, q_y, q_z, length=arrow_length)

        last_pose = poses[i]

    # Set labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Show plot
    plt.show()

if __name__ == '__main__':
    main()