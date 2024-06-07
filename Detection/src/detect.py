#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Image, CameraInfo
from movement_primitives.dmp import CartesianDMP
import pytransform3d.transformations as pt
import rospkg
from bagpy import bagreader
import matplotlib.pyplot as plt
import sys
from bagpy import bagreader
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
from visualization_msgs.msg import Marker
import tf
    
class ObjectDetector:
    def __init__(self):
        rospy.init_node("detect")

        # depth image, will be updated by subscriber
        self.depth_img = np.zeros((480, 640))

        # get intrinsic matrix 
        print("Getting camera info ...")
        camera_info = rospy.wait_for_message('/xtion/depth_registered/camera_info', CameraInfo, timeout=5)
        self.K = np.array(camera_info.K).reshape((3, 3))
        print("Successfully retrieved intrinsic matrix:")
        print(self.K)

        # load YOLO model
        # ID = 6 # here add your trainID directory
        self.model = YOLO("/home/user/exchange/Detection/runs/detect/train_best/best.pt")
        # self.model = YOLO('yolov8n.pt')
        
        self.listener = tf.TransformListener()

        # TODO: synchronize depth and rgb image subscribers

        # initialize ROS publishers
        self.image_pub = rospy.Publisher("/xtion/rgb/image_raw2", Image, queue_size=10)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)
        # self.image_pub = rospy.Publisher("/kinect2/qhd/image_color_rect2", Image, queue_size=10)
        self.bridge = CvBridge()

        # initialize ROS subscribers
        # self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, self.callback)
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.callback_image)
        self.depth_sub = rospy.Subscriber("/xtion/depth_registered/image_raw", Image, self.callback_depth)

        self._xtion_frame = "xtion_rgb_optical_frame" # frame of depth camera
        self._map_frame = "map" # world frame
        self._base_frame = "base_footprint" # robot base frame

        self.can_position = None

        print("Successfully initialized Publisher/Subscriber and YOLO model!")

    def callback_image(self, data):
        time = data.header.stamp
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        results = self.detect_objects(image)
        annotated_image = self.annotate_image(results[0], time)
    
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def callback_depth(self, data):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)

        self.depth_img = depth_img

    def convert_2D_to_3D_point(self, x, y):
        K_inv = np.linalg.inv(self.K)
        depth = self.depth_img[y, x]

        point_2d = np.array([x, y , 1])
        point_3d = (K_inv @ point_2d) * depth
        
        point = np.array([point_3d[0], point_3d[1], point_3d[2], 1])
        print('3D point: ' + str(point))
        return point
    
    def base_T_xtion(self, point, time):
        self.listener.waitForTransform(self._xtion_frame, self._base_frame, time, rospy.Duration(5))
        trans, rot = self.listener.lookupTransform(self._xtion_frame, self._base_frame, time)

        pose_xtion = np.array([trans[0], trans[1], trans[2], rot[3], rot[0], rot[1], rot[2]]) 
        base_T_xtion = pt.transform_from_pq(pose_xtion)

        point_map = np.linalg.inv(base_T_xtion) @ point

        return point_map

    def detect_objects(self, image):
        return self.model(image)

    def publish_marker(self, point):
        marker = Marker()

        marker.header.frame_id = self._base_frame
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 2
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        self.marker_pub.publish(marker)
        self.can_position = np.array([point[0], point[1], point[2]]) # should be removed

    def annotate_image(self, result, time):
        for idx in range(len(result.boxes)):
            boxes = result.boxes[idx]
            cls = result.names[int(boxes.cls.item())]

            if cls != 'can':
                print("Detected something else than can")
                continue

            x1, y1, x2, y2 = map(int, result.boxes[idx].xyxy[0])

            offset_x = int(np.abs(x1 - x2) / 2)
            offset_y = int(np.abs(y1 - y2) / 2)

            point = self.convert_2D_to_3D_point(x1 + offset_x, y1 + offset_y)
            point = self.base_T_xtion(point, time)
            self.publish_marker(point)

            conf = boxes.conf.item()

            cv2.rectangle(result.orig_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.putText(result.orig_img, cls, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 0 ,255), thickness=2)
            cv2.putText(result.orig_img, str(round(conf, 2)), (x1, y2), cv2.FONT_HERSHEY_SIMPLEX, 1, color=(0, 0 ,255), thickness=2)

        return result.orig_img

if __name__ == '__main__':
    image_converter = ObjectDetector()

    try: 
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")
