#!/usr/bin/env python3
import rospy
import cv2 as cv
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class BirdsEyeViewNode:
    def __init__(self):

        # initialize paths
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        script_dir = os.path.dirname(os.path.realpath(__file__))
        cam_matrix_path = os.path.join(script_dir, '..', '..', '..', 'imgs', 'cameraMatrix.npy')
        dist_coeffs_path = os.path.join(script_dir, '..', '..', '..', 'imgs', 'distCoeffs.npy')

        if not os.path.exists(cam_matrix_path) or not os.path.exists(dist_coeffs_path):
            rospy.logerr("Camera calibration files not found. Run calibration first.")
            raise FileNotFoundError("Missing cameraMatrix.npy or distCoeffs.npy")

        self.camera_matrix = np.load(cam_matrix_path)
        self.dist_coeffs = np.load(dist_coeffs_path)

        rospy.loginfo("Loaded camera calibration parameters.")


        # image_points should be the corners of the maze after undistortion.
        self.image_points = np.array([[100, 200],
                                      [500, 200],
                                      [500, 400],
                                      [100, 400]], dtype=np.float32)

        # world coorindates output view size and coordinates
        self.world_points = np.array([[0,   0],
                                      [500, 0],
                                      [500, 400],
                                      [0,   400]], dtype=np.float32)

        # compute the homography
        self.H, _ = cv.findHomography(self.image_points, self.world_points)
        if self.H is None:
            rospy.logerr("Could not compute homography. Check your correspondences.")
            raise ValueError("Invalid image_points or world_points for homography.")

        rospy.loginfo("Homography matrix computed.")

        # the size of the bird’s-eye view output
        self.output_size = (500, 400)  # (width, height)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

        cv.namedWindow("Bird's-Eye View", cv.WINDOW_NORMAL)
        rospy.loginfo(f"BirdsEyeViewNode subscribed to: {self.image_topic}")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # undistort the image
        undistorted = cv.undistort(cv_image, self.camera_matrix, self.dist_coeffs)

        # warp to bird’s-eye view
        birds_eye = cv.warpPerspective(undistorted, self.H, self.output_size)

        # show the result
        cv.imshow("Bird's-Eye View", birds_eye)
        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown("User requested shutdown.")

if __name__ == "__main__":
    rospy.init_node('birds_eye_view_node', anonymous=True)
    try:
        node = BirdsEyeViewNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv.destroyAllWindows()
