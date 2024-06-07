#!/usr/bin/env python3
import numpy as np
from movement_primitives.dmp import CartesianDMP
from bagpy import bagreader
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob
import os
import pytransform3d.transformations as pt
import sys

TOOL_LINK_POSE = '/arm_right_tool_link_pose'
MODEL_STATES = '/gazebo/model_states'
GRIPPER_POSE = '/gripper_right_grasping_frame_pose'
GRIPPER_STATUS = '/parallel_gripper_right/is_grasped'
TABLE_CAMERA = '/table_camera/image_raw/compressed'

def main():
    file_path = 'C:/Users/morit/Documents/Advanced_Robot_Learning/Manipulation/2024_ARL_demos_all/_2024-03-14-15-00-43/'
    save_path = file_path
    action = sys.argv[1]
    
    # Load data
    poses, timestamps = load_data(file_path, action=action)

    # Train DMP
    dmp_weights, execution_time, dt = train_dmp(poses, timestamps)

    # Save results
    save_dmp_parameters(dmp_weights, execution_time, dt, save_path, action)

def load_data(file_path, action='grasp'):
    motion_data = np.load(file_path + '/' + action + '.npz')
    return motion_data['pose'], motion_data['time']

def train_dmp(poses, timestamps, action='grasp'):
    n_steps = poses.shape[0]
    dt = 1/500
    execution_time = timestamps[-1] - timestamps[0]

    execution_time = 10
    # if action == 'grasp' or action == 'place':
    #     execution_time = 15
    # elif action == 'retrieve':
    #     execution_time = 7
    # else:
    #     print('wrong action')
    #     exit(-1)
    n_weights = 10
    smooth_scaling = False

    # if action == 'place':
    #     n_weights = 3
    #     smooth_scaling = True
        

    dmp = CartesianDMP(execution_time=execution_time, dt=dt, n_weights_per_dim=n_weights, smooth_scaling=smooth_scaling)

    T = np.linspace(0, execution_time, n_steps)
    Y = poses
        
    dmp.imitate(T, Y)
    weights = dmp.get_weights()

    # print(weights)
    # evaluate_dmp(dmp, poses, T)

    return weights, execution_time, dt

def evaluate_dmp(dmp : CartesianDMP, learned_poses, times):
    # generate can grid
    first_can_position = np.array([1.9, -3, 0.55, 1, 0, 0, 0])
    last_can_position = np.array([1.6, -2.7, 0.55, 1, 0, 0, 0])
    n = 9
    num_points_per_dim = int(n ** 0.5)
    if num_points_per_dim * num_points_per_dim != n:
        raise ValueError("n must be a perfect square")
    steps = (last_can_position - first_can_position) / (num_points_per_dim - 1)
    can_positions = np.array([first_can_position + np.multiply(steps, [i, j, 0, 1, 0, 0, 0]) 
                        for i in range(num_points_per_dim) for j in range(num_points_per_dim)])

    # can_pos = np.array([1.75, -2.85, 0.49, 0.0, 0.0, 0.0, 1.0])
    # tool_link_pos = learned_poses[-1]

    # print(can_pos)
    # print(tool_link_pos)
    # print(can_pos[:3]-tool_link_pos[:3])
    # exit(-1)

    can_position = can_positions[4]
    # can_position = np.array([0.55502906, -0.24390224, 0.48721702, -0.91393093, -0.05658766, -0.05658766, -0.05658766])
    # can_position = np.array([5.50018287e-01, -2.00276326e-01, 4.90109031e-01, 7.06860885e-01, 7.07352585e-01, 6.82685950e-05, -7.16491331e-05])
    goal_position = np.array([0.42686114, -0.31272466, 0.66129273, 0.94653658, -0.2183759, 0.09138971, 0.21915382])

    dmp.configure(t=times, start_y=learned_poses[0], goal_y=goal_position)

    t_dmp, y_dmp = dmp.open_loop()
    # for index in range(y_dmp.shape[0]):
    #     y_dmp[index][3:] = -y_dmp[index][3:]

    # plot_trajectory(learned_poses)
    print(y_dmp[-1])
    plot_trajectory(y_dmp)
    plt.show()

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
    # plt.show()

def save_dmp_parameters(dmp_weights, execution_time, dt, save_path, action='grasp'):
    np.savez(save_path + action + '_dmp.npz', 
             weights=dmp_weights, 
             duration=execution_time, 
             dt=dt)

if __name__ == '__main__':
    main()