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
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from std_msgs.msg import Header

import tf.transformations as tf_trans

#detected_tag_0 = False
#detected_tag_1 = False

PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

class ARTagDetector:
    def __init__(self):
        rospy.init_node('ar_tag_detector', anonymous=True)

        self.bridge = CvBridge()

        self.cv_color_image = None
        self.cv_depth_image = None

        #for logitech camera
        #self.color_image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.color_image_callback)
        self.color_image_sub = rospy.Subscriber("raw_birds_eye_image", Image, self.color_image_callback)
        #self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)
        self.camera_info_sub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.camera_info_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.detected_tag_0 = False
        self.detected_tag_1 = False
        # Load the ArUco dictionary
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.marker_length = 0.1   # 100mm for the printed out one, the one on the turtlebot is different what, PRINT out tags same size as the one on turtlebot

        self.tf_listener = tf.TransformListener()  

        self.point_pub = rospy.Publisher("goal_point", PointStamped, queue_size=10)
        self.camera_point_pub = rospy.Publisher("camera_pixel_ar_tags", PointStamped, queue_size=1)
        self.camera_pose_pub = rospy.Publisher("camera_pose_ar_tags", PoseStamped, queue_size=10)
        #include AR tag value 
        #rate = rospy.Rate(1)
        #rospy.sleep(1)
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
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo("Recieved image data")

            cv2.imshow("Camera Feed", self.cv_color_image)
            self.detect_process_ar_tags()
        #except Exception as e:
            # If we have both color and depth images, process them
            #if self.cv_depth_image is not None:
         #   rospy.loginfo(f"Error processing image")

        #except Exception as e:
         #   print("Error:", e)

    #def depth_image_callback(self, msg):
    #    try:
            # Convert the ROS Image message to an OpenCV image (16UC1 format)
    #        self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("Error:", e)

    #detected_tag_0 = False
    #detected_tag_1 = False

    def detect_process_ar_tags(self):
        gray = cv2.cvtColor(self.cv_color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            rospy.loginfo("No AR tags detected")
            return
        
        #show tags
        frame = cv2.aruco.drawDetectedMarkers(self.cv_color_image, corners, ids)
        cv2.imshow('Detected AR Tags', frame)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, np.array([[self.fx, 0, self.cx],
                                                                                                      [0, self.fy, self.cy],
                                                                                                      [0, 0, 1]]), None)
        #print('IDs:', ids)
        #find center for each of the tags
        for i, id_ in enumerate(ids.flatten()):
            #print('i', i)
            #center_x = int(np.mean(corners[i][0][:, 0]))
            #center_y = int(np.mean(corners[i][0][:, 1]))

            #if (id_ == [0]) and self.detected_tag_0:
            #    print('0 already detected')
            #    continue
            #if (id_ == [1]) and self.detected_tag_1:
            #    print('1 already detected')
            #    continue

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

            pixel_point.point.x = int(u_pixel/(680/350))
            pixel_point.point.y = int(v_pixel/(680/350))
            pixel_point.point.z = 0
            
            if id_ == [0]:
                #print('0 detected')
                self.detected_tag_0 = True
                x0 = pixel_point.point.x
                y0 = pixel_point.point.y
            if id_ == [1]:
                #print('1 detected')
                self.detected_tag_1 = True
                x1 = pixel_point.point.x
                y1 = pixel_point.point.y

            rotation_matrix, _ = cv2.Rodrigues(rvec)  
            quaternion = tf_trans.quaternion_from_matrix(
                [[rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], 0],
                [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], 0],
                [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], 0],
                [0, 0, 0, 1]]
            )

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = str(id_)

            pose.pose.position.x = tvec[0]
            pose.pose.position.y = tvec[1]
            pose.pose.position.z = tvec[2]

            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            self.camera_pose_pub.publish(pose)
            
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
            '''
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
            '''

if __name__ == '__main__':
    ARTagDetector()
