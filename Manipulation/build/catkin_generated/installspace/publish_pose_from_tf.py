#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
import rospy
import tf

class PublishPoseFromTF:
    def __init__(self):
        self.listener = tf.TransformListener()

        # This frame is used as output pose's reference coordinate
        self._base_frame = rospy.get_param('~base_frame')
        # This frame's pose relative to base_frame will be published as geometry_msgs/PoseStamped
        self._pose_frame = rospy.get_param('~pose_frame')
        self._timeout = 1.0

        self._pose_pub_topic = self._pose_frame + "_pose"

        self._pose_pub = rospy.Publisher(self._pose_pub_topic, PoseStamped, queue_size = 1)

        self.main_loop()
    
    def main_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self.listener.waitForTransform(self._base_frame, self._pose_frame, now, rospy.Duration(self._timeout))
                trans, rot = self.listener.lookupTransform(self._base_frame, self._pose_frame, now)
            except tf.Exception as e:  # Catch more specific exceptions
                rospy.logwarn("Failed to get transform: " + str(e))  # Log the actual exception
                continue

            pose = PoseStamped()
            pose.header.frame_id = self._base_frame
            pose.header.stamp = rospy.Time.now()

            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]

            self._pose_pub.publish(pose)

            rate.sleep()
            
 
if __name__ == '__main__':
    rospy.init_node('publish_pose_from_tf')
    publish_pose_from_tf = PublishPoseFromTF()