# Manipulation using DMPs
This package contains the skeleton code for the manipulation part in the "Advanced Methods of Robot Learning" course.

# Student Assignments
In this part of the course, students are expected to:
1. Process and label demonstration data: correct quaternion orientation, perform data segmentation, and add annotations.
2. Use Cartesian DMPs to encode the demonstrated motions.
3. Implement the reproduction of the learned motions on the robot to execute the pick and place task.

# Submission
For successful completion of the course, each student must submit the following components via the TUWEL course together with the report (June 20):
1) Segmented and annotated demonstrations
    - 9 demonstrations of the pick and place task, one for each of the predefined grid positions of the can
    - Must include the trajectory of the grasping frame
    - Segment according to the motion types “grab”, “retrieve”, and “place”
    - Save segmented motions as separate .npz files where the filename contains the motion type (e.g. "grab_1.npz"). This should include:
        - Array "pose" containing the pose (x,y,z,qw,qx,qy,qz) trajectory of the grasping frame
        - Array "time" containing the time array corresponding to the grasping frame's pose trajectory
        - Any additional arrays that are necessary for your implementation
2) Trained DMP parameters
    - Submit one DMP parameter set for each motion type: “grab”, “retrieve”, and “place”
    - Include weights, durations, and other relevant parameters used for the DMP encoding
    - Save DMP parameters in .npz files where the filename contains the motion type (e.g. "grab_dmp.npz"). This should include:
        - Array "weights" containing the learned weights of the forcing term
        - Array "duration" containing the motion duration
        - Any additional arrays that are necessary for your implementation
3) Motion reproductions
    - Provide your code for executing learned motions on the robot
    - Recordings of 5 reproductions (.bag files) with random can starting positions. For each reproduction the different motion types should be recorded separatly, e.g. "reproduction_1_grab.bag". This must include:
        - Can position in robot base frame
        - Gripper frame in robot base frame
        - Video of the reproduction


# Information on the provided code

## Data recording 
The provided `record.launch` allows to record the state of the objects in simulation, the pose of the right arm tool link, the pose of the right gripper, the grasping state, and an rgb video of the table and the robot.
The script `publish_pose_from_tf.py` is used to publish the right arm tool link pose and right gripper pose.

## Data processing (1. Task for students)
In the `process_data.py` you will have to implement your code to process the data, segment the motion, and annotate the motion segements. An example structure is given here. You can use the python module bagpy to read in data from a recorded bag file.

## DMP encoding (2. Task for students)
The implementation of the motion encoding should be done in the `dmp_learn.py` script. There the processed data should be load, a Cartesian DMP learned, and the DMP parameters saved to a file. An example structure is given in the provided file.

## DMP motion reproduction (3. Task for students)
The `dmp_play.py` script provides the functionality to open/close the gripper, to get the poses of the arm tool link and gripper from the TIAGo++ robot, and to get the pose of the can with respect to the robot's base footprint frame. You will have to implement your approach to load the DMP parameters, generate the DMP motion, and command this motion to the robot. The motion of the right arm can be commanded by using the provided publisher for the robot's whole body controller. For recoding the reproduced motion use `record.launch` as mentioned in the "Data recording" section.

## Resetting the gazebo world
You can use `roslaunch arl_manipulation_baseline set_world.launch can_pose:=0` to reset the model poses. With the argument `can_pose` you can specify the desired can pose to be in a 3x3 grid (`can_pose` 1-9) or at a random position within this area (`can_pose` 0). To reset the robot's joint configuration restart the whole-body controller.

## Evaluation
By running the evaluation script all bag files placed in the subdirectory `/data/eval_data/` will be evaluate and the results saved in subdirectory `/data/eval_results`.
For the evaluation to work properly the file name must containe "grab", "retrieve", or "place" to identify the appropriate motion type.
The evaluation metrics are:
1) success rate: 1=success, 0=failed
2) execution time: the lower the better
3) motion smoothness: the higher the better

Your submitted code will be evaluated using these metrics for 10 random can starting positions. If you think a finer segmentation than "grab", "retrieve", and "place" is advantageous, feel free to implement your approach accordingly. However, make sure that in the end these motions can be executed as one of the motion types "grab", "retrieve", and "place".

# Important commands
## Docker
- Start docker using the provided shell script: 
    ```
    ./docker_run.sh
    ```

- List all active Docker containers by running the command: 
    ```
    docker ps
    ```

- Open a new terminal window in the existing Docker container by running the command: 
    ```
    docker exec -it [docker id] bash
    ```

## Tiago
- Start the TIAGo++ gazebo simulation: `roslaunch tiago_dual_gazebo tiago_dual_gazebo.launch public_sim:=true end_effector_left:=pal-gripper end_effector_right:=pal-gripper world:=pick_coca gzpose:="-x 1.95 -y -2.3 -z 0.0 -R 0.0 -P 0.0 -Y -1.57" base_type:=omni_base`

- Stop all controllers by executing the ROS service call:
  ```bash
  rosservice call /controller_manager/switch_controller "start_controllers: 
  - ''
  stop_controllers:
  - 'head_controller'
  - 'arm_left_controller'
  - 'arm_right_controller'
  - 'torso_controller'
  - 'whole_body_kinematic_controller'
  strictness: 0"tness: 0"

- Start whole bod controller:
    ```bash
    roslaunch tiago_dual_wbc tiago_dual_wbc.launch

- Set control parameters:
    ```bash
    rosrun pal_wbc_utils push_interactive_marker_pose_task _source_data:=topic_reflexx_typeII _tip_name:=arm_right_tool_link _position_gain:=3 _orientation_gain:=3 _reference_frame:=/base_footprint

- Set torso reference height:
    ```bash
    rosrun pal_wbc_utils push_torso_height_ref _before_task_id:=position_arm_right_tool_link _link_name:=torso_lift_link  _reference_height:=1.1


If you have any further questions, please use the TUWEL course forum or contact the teaching assistants.