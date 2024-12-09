#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import time
import tf
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

class ARTagDetector:
    def __init__(self):
        rospy.init_node('ar_tag_detector', anonymous=True)

        self.bridge = CvBridge()

        self.cv_color_image = None
        self.cv_depth_image = None

        #for logitech camera
        self.color_image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.color_image_callback)
        #self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)
        self.camera_info_sub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.camera_info_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Load the ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.marker_length = 0.1   # 100mm for the printed out one, the one on the turtlebot is different what, PRINT out tags same size as the one on turtlebot

        self.tf_listener = tf.TransformListener()  

        self.point_pub = rospy.Publisher("goal_point", PointStamped, queue_size=10)
        self.camera_point_pub = rospy.Publisher("camera_pixel_ar_tags", PointStamped, queue_size=10)
        #include AR tag value 

        rospy.spin()

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth):
        print(self.camera_info_sub)
        # TODO: Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = ((u - self.cx) * depth) / self.fx
        Y = ((v - self.cy) * depth) / self.fy
        Z = depth
        return X, Y, Z

    def color_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (BGR8 format)
            #self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # If we have both color and depth images, process them
            #if self.cv_depth_image is not None:
            self.detect_process_ar_tags()

        except Exception as e:
            print("Error:", e)

    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("Error:", e)

    def detect_process_ar_tags(self):
        gray = cv2.cvtColor(self.cv_color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            rospy.loginfo("No AR tags detected")
            return
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, np.array([[self.fx, 0, self.cx],
                                                                                                      [0, self.fy, self.cy],
                                                                                                      [0, 0, 1]]), None)

        #find center for each of the tags
        for i, id_ in enumerate(ids.flatten()):
            #center_x = int(np.mean(corners[i][0][:, 0]))
            #center_y = int(np.mean(corners[i][0][:, 1]))

            tvec = tvecs[i][0]
            rvec = rvecs[i][0]

            #for camera pixel points
            marker_corners = corners[i][0]

            #centers
            u_pixel = np.mean(marker_corners[:, 0])
            v_pixel = np.mean(marker_corners[:, 1])

            pixel_point = PointStamped()
            pixel_point.header.stamp = rospy.Time.now()
            pixel_point.header.frame_id = str(id_)

            pixel_point.point.x = u_pixel
            pixel_point.point.y = v_pixel
            pixel_point.point.z = 0
            
            pixel_point_x, pixel_point_y, pixel_point_z = pixel_point.point.x, pixel_point.point.y, pixel_point.point.z

            self.camera_point_pub.publish(pixel_point)
            # Fetch the depth value at the center
            #depth = self.cv_depth_image[center_y, center_x]

            #if depth == 0:
            #    rospy.loginfo(f"Incorrect tag {id_}: invalid depth at center ({center_x}, {center_y}).")
            #   continue

            #camera_x, camera_y, camera_z = self.pixel_to_point(center_x, center_y, depth)
            #camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
            # Convert from mm to m
            #camera_link_x /= 1000
            #camera_link_y /= 1000
            #camera_link_z /= 1000

            camera_x, camera_y, camera_z = tvec

            # Convert the (X, Y, Z) coordinates from camera frame to odom frame
            try:
                self.tf_listener.waitForTransform("/odom", "/camera_link", rospy.Time(), rospy.Duration(10.0))
                #point_odom = self.tf_listener.transformPoint("/odom", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_link"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
                point_odom = self.tf_listener.transformPoint("/odom", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_link"), point=Point(camera_x, camera_y, camera_z)))
                X_odom, Y_odom, Z_odom = point_odom.point.x, point_odom.point.y, point_odom.point.z
                print("Real-world coordinates in odom frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(X_odom, Y_odom, Z_odom))

                if X_odom < 0.001 and X_odom > -0.001:
                    print("Erroneous goal point, not publishing - Is the AR Tag in frame?")
                else:
                    print("Publishing goal point: ", X_odom, Y_odom, Z_odom)
                    # Publish the transformed point
                    self.point_pub.publish(PointStamped(header = Header(stamp=rospy.Time(), frame_id=str(id_)), point = Point(X_odom, Y_odom, Z_odom)))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                #print("TF Error: " + str(e))
                print("TF Error: " + e)
                return

if __name__ == '__main__':
    ARTagDetector()
