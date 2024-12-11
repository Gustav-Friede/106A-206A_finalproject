#!/usr/bin/env python3
import cv2
import rospy
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from threading import Thread
from queue import Queue

class BirdsEyeViewNode:
    def __init__(self):
        MORPH_KERNEL_SIZE = (4, 4)

        # Initialize paths and directories
        script_dir = os.path.dirname(os.path.abspath(__file__))
        img_path = os.path.join(script_dir, '..', 'data', 'camera_snapshots', 'snapshot_000.png')
        self.save_dir = os.path.join(script_dir, '..', 'data', 'camera_calibration')

        os.makedirs(self.save_dir, exist_ok=True)

        img = cv2.imread(img_path)
        if img is None:
            raise FileNotFoundError("Image not found at the specified path.")

        # Threshold the image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Morphological closing to consolidate maze shape
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, MORPH_KERNEL_SIZE)
        closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            rospy.logerr("No contours found. Adjust threshold or kernel size.")
            exit(1)

        # Find largest quadrilateral
        maze_contour = max(contours, key=cv2.contourArea)
        perimeter = cv2.arcLength(maze_contour, True)
        approx = cv2.approxPolyDP(maze_contour, 0.03 * perimeter, True)

        if len(approx) != 4:
            rospy.logerr("No quadrilateral found. Adjust preprocessing steps.")
            exit(1)

        corners = approx.reshape(4, 2)
        maze_corners = np.zeros((4, 2), dtype=np.int32)
        s = corners.sum(axis=1)
        diff = np.diff(corners, axis=1)

        # Order corners
        maze_corners[0] = corners[np.argmin(s)]
        maze_corners[2] = corners[np.argmax(s)]
        maze_corners[1] = corners[np.argmin(diff)]
        maze_corners[3] = corners[np.argmax(diff)]

        # Subscribe to rectified image
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.image_points = maze_corners
        self.world_points = np.array([[0.0, 0.0], [680.0, 0.0], [680.0, 680.0], [0.0, 680.0]], dtype=np.float32)
        self.H, _ = cv2.findHomography(self.image_points, self.world_points)

        if self.H is None:
            raise ValueError("Invalid points for homography.")

        self.output_size = (700, 700)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        self.image_queue = Queue()
        Thread(target=self.process_images).start()

        rospy.loginfo("BirdsEyeViewNode initialized.")

    def image_callback(self, msg):
        self.image_queue.put(msg)

    def process_images(self):
        while not rospy.is_shutdown():
            if not self.image_queue.empty():
                msg = self.image_queue.get()
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                birds_eye = cv2.warpPerspective(cv_image, self.H, self.output_size)
                hue, saturation, value = cv2.split(cv2.cvtColor(birds_eye, cv2.COLOR_BGR2HSV))

                cv2.imshow("Bird's-Eye View", value)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.signal_shutdown("User requested shutdown.")

if __name__ == "__main__":
    rospy.init_node('birds_eye_view_node', anonymous=True)
    try:
        node = BirdsEyeViewNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
