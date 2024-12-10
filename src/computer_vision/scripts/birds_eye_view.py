#!/usr/bin/env python3
import cv2
import rospy
import cv2 as cv
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class BirdsEyeViewNode:


    def __init__(self):

        MORPH_KERNEL_SIZE = (4, 4)

        # initialize directory paths
        script_dir = os.path.dirname(os.path.abspath(__file__))
        img_path = os.path.join(script_dir,
                                '..',
                                'data',
                                'camera_snapshots',
                                'snapshot_000.png')
        self.save_subdir = rospy.get_param('~save_subdir', '')
        self.save_dir = os.path.join(script_dir, '..', 'data', 'camera_calibration')

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

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

        # find contours in the processed image and pick the largest contour with hope it's the maze outline
        contours, hierarchy = cv.findContours(closed, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            print("No contours found. Consider adjusting THRESH_VALUE or MORPH_KERNEL_SIZE.")
            exit(1)
        contours = sorted(contours, key=cv.contourArea, reverse=True)
        maze_contour = None

        # approximate the contour to a polygon
        for cnt in contours:
            perimeter = cv.arcLength(cnt, True)
            epsilon = 0.03 * perimeter  # Adjust epsilon if needed
            approx = cv.approxPolyDP(cnt, epsilon, True)

            # ensure approximated polygon has 4 vertices
            if len(approx) == 4:
                maze_contour = approx
                break

        if maze_contour is None:
            print("No quadrilateral found. Try adjusting preprocessing steps.")
            exit(1)

        # sort corners by y-coordinate and allow x-coordinate be the tie-breaker
        corners = maze_contour.reshape(4, 2)

        maze_corners = np.zeros((4, 2), dtype=np.int32)
        s = corners.sum(axis=1)

        # top-left point has the smallest difference (x-y)
        # bottom-right point will have the largest sum (x+y)
        maze_corners[0] = corners[np.argmin(s)]
        maze_corners[2] = corners[np.argmax(s)]

        diff = np.diff(corners, axis=1)
        maze_corners[1] = corners[np.argmin(diff)]
        maze_corners[3] = corners[np.argmax(diff)]


        # subscribe to rectified image
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
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
            rospy.logerr("Could not compute homography. Check your correspondences.")
            raise ValueError("Invalid image_points or world_points for homography.")
        rospy.loginfo("Homography matrix computed.")

        # output window screen size for the bird’s-eye view
        self.output_size = (700, 700)  # width, height

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.latest_image = None
        self.image_count = 0

        cv.namedWindow("Bird's-Eye View", cv.WINDOW_NORMAL)
        rospy.loginfo(f"BirdsEyeViewNode subscribed to: {self.image_topic}")

    def image_callback(self, msg):
       # print('image_callback triggered')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        birds_eye = cv.warpPerspective(cv_image, self.H, self.output_size)
        latest_image = birds_eye.copy()

        cv.imshow("Bird's-Eye View", birds_eye)
        key = cv.waitKey(1) & 0xFF
        if key == ord('S'):
            if latest_image is not None:
                filename = f"snapshot_{self.image_count:02d}.png"
                filepath = os.path.join(self.save_dir, filename)

                hue, saturation, value = cv2.split(cv2.cvtColor(latest_image, cv.COLOR_BGR2HSV))
                cv.imwrite(filepath, latest_image)
                cv.imwrite(os.path.join(self.save_dir, f"hue_{self.image_count:01d}.png"), hue)
                cv.imwrite(os.path.join(self.save_dir, f"sat_{self.image_count:01d}.png"), saturation)
                cv.imwrite(os.path.join(self.save_dir, f"value_{self.image_count:01d}.png"), value)
                rospy.loginfo(f"Saved image: {filepath}")
                self.image_count += 1
            else:
                rospy.logwarn("No image to save.")
        if key == ord('Q'):
            rospy.signal_shutdown("User requested shutdown.")


if __name__ == "__main__":
    rospy.init_node('birds_eye_view_node', anonymous=True)
    try:
        node = BirdsEyeViewNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv.destroyAllWindows()
