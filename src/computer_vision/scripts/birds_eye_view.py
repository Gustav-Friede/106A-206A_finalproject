#!/usr/bin/env python3
import rospy
import cv2 as cv
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class BirdsEyeViewNode:
    def __init__(self):
        # Subscribe to rectified image
        self.image_topic = rospy.get_param("~image_topic", "/head_camera/image_rect_color")

        # Example image_points and world_points
        # Adjust these based on known correspondences in your scene
        self.image_points = np.array([[150, 35],
                                      [524, 32],
                                      [635, 344],
                                      [0, 331]], dtype=np.float32)

        self.world_points = np.array([[0.0,   0.0],
                                      [1.524, 0.0],
                                      [1.524, 1.524],
                                      [0.0,   1.524]], dtype=np.float32)

        # Compute homography
        self.H, _ = cv.findHomography(self.image_points, self.world_points)
        if self.H is None:
            rospy.logerr("Could not compute homography. Check your correspondences.")
            raise ValueError("Invalid image_points or world_points for homography.")
        rospy.loginfo("Homography matrix computed.")

        # Output size for the bird’s-eye view
        self.output_size = (680, 480)  # width, height

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

        cv.namedWindow("Bird's-Eye View", cv.WINDOW_NORMAL)
        rospy.loginfo(f"BirdsEyeViewNode subscribed to: {self.image_topic}")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Since image is already rectified by image_proc, no need for undistort
        birds_eye = cv.warpPerspective(cv_image, self.H, self.output_size)

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
