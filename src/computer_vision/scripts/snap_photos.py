#!/usr/bin/env python3
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import numpy as np

class PhotoSnapper:
    def __init__(self):
        # load parameters from the parameter server
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.save_subdir = rospy.get_param('~save_subdir', 'camera_snapshots')
        self.aruco_dict_name = rospy.get_param('~aruco_dictionary', 'DICT_6X6_250')
        self.squares_horizontally = rospy.get_param('~squares_horizontally', 5)
        self.squares_vertically = rospy.get_param('~squares_vertically', 5)
        self.square_length = rospy.get_param('~square_length', 0.3048)
        self.marker_length = rospy.get_param('~marker_length', 0.15)
        self.total_page_length_px = rospy.get_param('~total_page_length_px', 2000)
        self.margin_px = rospy.get_param('~margin_px', 20)

        # set up save directory inside imgs/
        self.aruco_dict = getattr(cv.aruco, self.aruco_dict_name)
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.save_dir = os.path.join(script_dir, '..', '..', '..', 'camera_calibration', self.save_subdir)
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # try to load camera calibration parameters (if they exist)
        cam_matrix_path = os.path.join(script_dir, '..', '..', '..', 'camera_calibration', 'cameraMatrix.npy')
        dist_coeffs_path = os.path.join(script_dir, '..', '..', '..', 'camera_calibration', 'distCoeffs.npy')
        self.camera_matrix = np.load(cam_matrix_path) if os.path.exists(cam_matrix_path) else None
        self.dist_coeffs = np.load(dist_coeffs_path) if os.path.exists(dist_coeffs_path) else None
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            rospy.loginfo("Loaded camera calibration parameters. Undistorting in real-time.")

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.latest_image = None
        self.image_count = 0

        rospy.loginfo(f"PhotoSnapper initialized and subscribed to: {self.image_topic}")
        rospy.loginfo(f"Images will be saved in: {self.save_dir}")
        rospy.loginfo("Press 'S' to save a snapshot, 'Q' to quit.")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # undistort image and save a copy before beginning to draw on the board
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            cv_image = cv.undistort(cv_image, self.camera_matrix, self.dist_coeffs)
        raw_image = cv_image.copy()

        dictionary = cv.aruco.getPredefinedDictionary(self.aruco_dict)
        board = cv.aruco.CharucoBoard_create(
            self.squares_horizontally,
            self.squares_vertically,
            self.square_length,
            self.marker_length,
            dictionary
        )
        detector_params = cv.aruco.DetectorParameters_create()

        marker_corners, marker_ids, _ = cv.aruco.detectMarkers(cv_image, dictionary, parameters=detector_params)
        if marker_ids is not None and len(marker_ids) > 0:
            cv.aruco.drawDetectedMarkers(cv_image, marker_corners, marker_ids)
            retval, charuco_corners, charuco_ids = cv.aruco.interpolateCornersCharuco(marker_corners, marker_ids,
                                                                                      cv_image, board)
            if retval > 0:
                cv.aruco.drawDetectedCornersCharuco(cv_image, charuco_corners, charuco_ids)
                rospy.loginfo("ChArUco board detected!")

        #  display the annotated image
        self.latest_image = raw_image
        cv.imshow("Camera Feed", cv_image)

        # enter 'S' to take a snapshot, enter 'Q' to quit and camera parameters
        key = cv.waitKey(1) & 0xFF
        if key == ord('S'):
            self.save_image()
        elif key == ord('Q'):
            rospy.signal_shutdown("User requested shutdown.")

    def save_image(self):
        if self.latest_image is not None:
            filename = f"snapshot_{self.image_count:03d}.png"
            filepath = os.path.join(self.save_dir, filename)
            cv.imwrite(filepath, self.latest_image)
            rospy.loginfo(f"Saved image: {filepath}")
            self.image_count += 1
        else:
            rospy.logwarn("No image to save.")


if __name__ == '__main__':
    rospy.init_node('photo_snapper', anonymous=True)
    snapper = PhotoSnapper()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv.destroyAllWindows()

