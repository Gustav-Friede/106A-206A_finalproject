#!/usr/bin/env python3
import cv2 as cv
import rospy
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class HSVBirdsEyePublisher:

    def __init__(self):

        MORPH_KERNEL_SIZE = (4, 4)

        # initialize directory paths
        script_dir = os.path.dirname(os.path.abspath(__file__))
        img_path = os.path.join(script_dir,
                                '..',
                                'data',
                                'camera_snapshots',
                                'snapshot_000.png')
        self.save_dir = os.path.join(script_dir,
                                     '..',
                                     'data',
                                     'camera_calibration')

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        img = cv.imread(img_path)
        if img is None:
            raise FileNotFoundError("Image not found at the specified path.")

        # threshold the image to create a binary mask
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        clahe = cv.createCLAHE(clipLimit=1.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        _, thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

        # fill small holes and consolidate the maze shape using morphological closing
        kernel = cv.getStructuringElement(cv.MORPH_RECT, MORPH_KERNEL_SIZE)
        closed = cv.morphologyEx(thresh, cv.MORPH_CLOSE, kernel)

        # find contours and pick the largest one
        contours, hierarchy = cv.findContours(closed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            rospy.logerr("No contours found. Adjust THRESH_VALUE or MORPH_KERNEL_SIZE.")
            rospy.signal_shutdown("No contours found.")
            return

        contours = sorted(contours, key=cv.contourArea, reverse=True)
        maze_contour = None

        # approximate the contour to a polygon
        for cnt in contours:
            perimeter = cv.arcLength(cnt, True)
            epsilon = 0.03 * perimeter
            approx = cv.approxPolyDP(cnt, epsilon, True)

            # ensure approximated polygon has 4 vertices
            if len(approx) == 4:
                maze_contour = approx
                break

        if maze_contour is None:
            rospy.logerr("No quadrilateral found. Try adjusting preprocessing steps.")
            rospy.signal_shutdown("No quadrilateral found.")
            return

        # sort corners
        corners = maze_contour.reshape(4, 2)
        maze_corners = np.zeros((4, 2), dtype=np.int32)
        s = corners.sum(axis=1)

        # top-left, bottom-right corners
        maze_corners[0] = corners[np.argmin(s)]
        maze_corners[2] = corners[np.argmax(s)]

        diff = np.diff(corners, axis=1)
        # top-right, bottom-left corners
        maze_corners[1] = corners[np.argmin(diff)]
        maze_corners[3] = corners[np.argmax(diff)]

        self.image_points = maze_corners

        self.world_points = np.array([
            [0.0, 0.0],
            [680.0, 0.0],
            [680.0, 680.0],
            [0.0, 680.0]
        ], dtype=np.float32)

        # find homography
        self.H, _ = cv.findHomography(self.image_points, self.world_points)
        if self.H is None:
            rospy.logerr("Could not compute homography. Check correspondences.")
            rospy.signal_shutdown("Homography computation failed.")
            return
        rospy.loginfo("Homography matrix computed.")

        # output window screen size for the bird’s-eye view
        self.output_size = (700, 700)  # width, height

        self.bridge = CvBridge()

        # Subscribe to the raw image
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

        # Publishers for the HSV channels
        self.hue_pub = rospy.Publisher("/hue_image", Image, queue_size=1)
        self.sat_pub = rospy.Publisher("/saturation_image", Image, queue_size=1)
        self.val_pub = rospy.Publisher("/value_image", Image, queue_size=1)

        rospy.loginfo(f"Subscribing to: {self.image_topic}")
        rospy.loginfo("Publishing hue image on: /hue_image")
        rospy.loginfo("Publishing saturation image on: /saturation_image")
        rospy.loginfo("Publishing value image on: /value_image")


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # apply homography
        birds_eye = cv.warpPerspective(cv_image, self.H, self.output_size)

        # convert to HSV and extract hue, saturation, value
        hsv = cv.cvtColor(birds_eye, cv.COLOR_BGR2HSV)
        hue, saturation, value = cv.split(hsv)

        # publish images
        hue_msg = self.bridge.cv2_to_imgmsg(hue, encoding="mono8")
        sat_msg = self.bridge.cv2_to_imgmsg(saturation, encoding="mono8")
        val_msg = self.bridge.cv2_to_imgmsg(value, encoding="mono8")

        self.hue_pub.publish(hue_msg)
        self.sat_pub.publish(sat_msg)
        self.val_pub.publish(val_msg)


if __name__ == "__main__":
    rospy.init_node('hsv_camera_feed', anonymous=True)
    node = HSVBirdsEyePublisher()
    rospy.spin()
