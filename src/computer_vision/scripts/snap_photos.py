#!/usr/bin/env python3
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os

class PhotoSnapper:
    def __init__(self):

        # initialize topics and filepaths
        image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        save_subdir = rospy.get_param('~save_subdir', 'camera_snapshots')
        script_dir = os.path.dirname(os.path.realpath(__file__))
        img_directory = os.path.join(script_dir, '..', '..', '..', 'imgs', save_subdir)

        # subscribe to camera image topic
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.latest_image = None
        self.image_count = 0

        # save image and report to terminal
        if not os.path.exists(img_directory):
            os.makedirs(img_directory)
        self.save_dir = img_directory

        rospy.loginfo(f"PhotoSnapper initialized and subscribed to: {image_topic}")
        rospy.loginfo(f"Images will be saved in: {self.save_dir}")
        rospy.loginfo("Press 'S' to save a snapshot, 'Q' to quit in the display window.")

    def image_callback(self, msg):
        # convert ROS Image message to OpenCV image
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # show camera feed ('S' to snap, 'Q' to quit)
        cv.imshow("Camera Feed", self.latest_image)
        key = cv.waitKey(1) & 0xFF

        if key == ord('S'):
            self.save_image()
        elif key == ord('Q'):
            rospy.signal_shutdown("User requested shutdown.")

    def save_image(self):
        if self.latest_image is not None:
            # noinspection PyTypeChecker
            filepath = os.path.join(self.save_dir, f"snapshot_{self.image_count:03d}.png")
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
