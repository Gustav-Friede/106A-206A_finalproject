#!/usr/bin/env python3
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os

script_dir = os.path.dirname(os.path.realpath(__file__))
img_directory = os.path.join(script_dir, '..', '..', '..', 'imgs/', 'camera_snapshots/')


class PhotoSnapper:
    def __init__(self):
        # bridge to convert ROS images to OpenCV
        self.bridge = CvBridge()

        # subscribe to the camera image topic
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.latest_image = None
        self.image_count = 0

        # directory to save images
        self.save_dir = img_directory
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        rospy.loginfo("PhotoSnapper initialized, waiting for images...")

    def image_callback(self, msg):
        # convert ROS Image message to OpenCV image
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # press 's' to save the current frame, or 'q' to exit
        cv.imshow("Camera Feed", self.latest_image)
        key = cv.waitKey(1) & 0xFF

        if key == ord('s'):
            self.save_image()
        elif key == ord('q'):
            rospy.signal_shutdown("User requested shutdown.")

    def save_image(self):
        if self.latest_image is not None:
            filename = f"snapshot_{self.image_count:04d}.png"
            filepath = os.path.join(self.save_dir, filename)
            cv.imwrite(filepath, self.latest_image)
            rospy.loginfo(f"Saved image: {filepath}")
            self.image_count += 1
        else:
            rospy.logwarn("No image available to save.")


if __name__ == '__main__':
    rospy.init_node('photo_snapper', anonymous=True)
    snapper = PhotoSnapper()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv.destroyAllWindows()
