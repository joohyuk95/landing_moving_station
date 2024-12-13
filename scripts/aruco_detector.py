#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2.aruco as aruco
from tf.transformations import quaternion_matrix

class ArucoDetector:
    def __init__(self):
        # ROS Node initialization
        rospy.init_node('aruco_marker_detection')
        self.state = Odometry()
        self.pose = PoseStamped()
        self.Rt_cam_to_aruco = np.eye(4)
        self.Rt_body_to_cam = np.eye(4)
        self.Rt_world_to_body = np.eye(4)
        
        x, y, z = 0.108, 0, -0.01 # SITL
        # x, y, z = 0.09, 0, -0.95 # OFFBOARD

        r_x = np.array([[1.0, 0.0, 0.0],
                        [0.0, -1.0, 0.0],
                        [0.0, 0.0, -1.0]])
        r_z = np.array([[0.0, -1.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0]])
        R = r_x @ r_z
        self.Rt_body_to_cam[:3, :3] = R[:3, :3]        
        self.Rt_body_to_cam[0, 3] = x
        self.Rt_body_to_cam[1, 3] = y
        self.Rt_body_to_cam[2, 3] = z

        # Camera matrix and distortion coefficients
        self.camera_matrix = None
        self.dist_coeffs = None

        # Initialize CvBridge for converting ROS images to OpenCV images
        self.bridge = CvBridge()

        # Define ArUco dictionary and detector parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters_create()

        # Initialize publishers
        self.pose_pub = rospy.Publisher("/aruco_marker/pose", PoseStamped, queue_size=10)
        self.image_pub = rospy.Publisher("/aruco_marker/result_image", Image, queue_size=10)

        # Subscribers
        # SITL
        self.image_sub = rospy.Subscriber("/iris/camera/rgb/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/iris/camera/rgb/camera_info", CameraInfo, self.camera_info_callback)
        self.state_sub = rospy.Subscriber("mavros/local_position/odom", Odometry, self.state_callback)

        # OFFBOARD
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # self.camera_info_sub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.camera_info_callback)
        # self.state_sub = rospy.Subscriber("/drone/mavros/local_position/odom", Odometry, self.state_callback)

    def state_callback(self, msg):
        # Extract position
        position = msg.pose.pose.position
        x, y, z = position.x, position.y, position.z

        # Extract orientation (quaternion)
        orientation = msg.pose.pose.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w

        # Convert quaternion to rotation matrix
        rotation_matrix = quaternion_matrix([qx, qy, qz, qw])

        # Construct the transformation matrix        
        self.Rt_world_to_body[:3, :3] = rotation_matrix[:3, :3]  # Rotation
        self.Rt_world_to_body[:3, 3] = [x, y, z]                 # Translation

    def camera_info_callback(self, msg):
        """Callback function to store camera intrinsic parameters."""
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D)

    def image_callback(self, msg):
        """Callback function to process images from the camera topic."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera matrix or distortion coefficients not yet received.")
            return

        # Convert ROS Image to OpenCV image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Could not convert image: %s" % e)
            return

        # Detect ArUco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)

        if ids is not None and 7 in ids:
            # Find the index of marker ID 7
            index = np.where(ids == 7)[0][0]

            # Estimate pose for marker ID 7
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index], 0.4, self.camera_matrix, self.dist_coeffs)

            # Draw the detected marker and axis on the image
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec[0], tvec[0], 0.1)          
                   

            # Convert rotation vector to Euler angles
            rot_mat, _ = cv2.Rodrigues(rvec[0])            
            self.Rt_cam_to_aruco[:3, :3] = rot_mat
            self.Rt_cam_to_aruco[:3, 3] = tvec[0].reshape(-1)
            
            Rt = self.Rt_world_to_body @ self.Rt_body_to_cam @ self.Rt_cam_to_aruco
            yaw_rad = np.arctan2(Rt[1, 0], Rt[0, 0])  # atan2(sin(yaw), cos(yaw))
    
            # Convert from radians to degrees
            yaw_deg = np.degrees(yaw_rad)
            if yaw_deg <= 0:
                yaw_deg = -yaw_deg
            else:
                yaw_deg = 360 - yaw_deg
            
            
            self.pose.header.stamp = rospy.Time.now()
            self.pose.header.frame_id = "world"     
            self.pose.pose.position.x = Rt[0, 3]
            self.pose.pose.position.y = Rt[1, 3]
            self.pose.pose.position.z = Rt[2, 3]
            self.pose.pose.orientation.z = yaw_deg

            # print("global_yaw: {}".format(yaw_deg))
            # print("trans_mat Rt: {}".format(Rt[:3, 3]))
            # print("---")
            self.pose_pub.publish(self.pose)
        else:
            self.pose = PoseStamped()
            self.pose.pose.orientation.z = -1
            self.pose_pub.publish(self.pose)
        
        try:
            result_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(result_image)
        except Exception as e:
            rospy.logerr("Failed to publish image: %s" % e)

    def run(self):
        """Start the ROS loop and spin the node."""
        rospy.spin()


if __name__ == '__main__':
    # Instantiate and run the ArUco detector
    detector = ArucoDetector()
    detector.run()
