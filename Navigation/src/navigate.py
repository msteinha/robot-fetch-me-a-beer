import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Header
from gazebo_msgs.msg import ModelStates
import actionlib
import yaml
from dataclasses import dataclass


def _generate_goal(pose: dict) -> MoveBaseGoal:
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = pose['frame_id']
	goal.target_pose.header.stamp = rospy.Time.now()

	goal.target_pose.pose.position = Point(
        x=pose['position']['x'],
        y=pose['position']['y'],
        z=pose['position']['z'])
    
	goal.target_pose.pose.orientation = Quaternion(
        x=pose['orientation']['x'],
        y=pose['orientation']['y'],
        z=pose['orientation']['z'],
        w=pose['orientation']['w'])
	return goal

def _generate_goals(poses_dict: dict) -> list:
	poses = [item['pose'] for item in poses_dict['poses']]
	return poses

"""
	Class for saving the results
"""
@dataclass
class ExecutedGoal:
    result: int
    initial_pose: Pose
    goal_sent: rospy.rostime.Time
    final_pose: Pose
    result_received: rospy.rostime.Time


class GoalPublisher:
	def __init__(self, poses_file_path=None):
		# rospy.init_node('goal_publisher') 

		# Load and parse pose file
		if poses_file_path != None:
			with open(poses_file_path, 'r') as file:
				self.poses = yaml.safe_load(file)
		
		# Create substriber to get gt robot state
		self.state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_state_cb, queue_size=1)

		# Initialize action client
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.client.wait_for_server()

		rospy.loginfo("action server available")

	def model_state_cb(self, msg):
		# Callback for the state subscriber
		pass
		
	def publish_goal(self, goal_pose : MoveBaseGoal):
		self.client.send_goal(goal_pose)
		self.client.wait_for_result(timeout=rospy.Duration(0))
		return self.client.get_result()
		

	def compute_metrics_rate(self):
		# Compute the metrics
		raise NotImplementedError()


if __name__ == "__main__":
	# Initialize node, execute path, compute metrics
	print('Initializing GoalPublisher')
	publisher = GoalPublisher('/home/user/exchange/Navigation/pose_pairs.yaml')

	print('Generating Goals')
	poses = _generate_goals(publisher.poses)

	print('Publishing Goals')
	for pose in poses:
		goal_pose = _generate_goal(pose)
		result = publisher.publish_goal(goal_pose)
		# TODO: evaluate action
		print(result)
