#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2  # OpenCV

def publish_static_image():
    # 1. Initialisiere den ROS-Knoten
    rospy.init_node('static_image_publisher', anonymous=True)

    # 2. Erstelle einen Publisher für das Topic
    image_pub = rospy.Publisher('/hue_image', Image, queue_size=10)

    # 3. Erstelle ein CvBridge-Objekt
    bridge = CvBridge()

    # 4. Lade das Bild (Pfad anpassen)
    image_path = "/home/cc/ee106a/fa24/class/ee106a-acj/final_project/imgs/reduced_maze.png"  # Ersetze dies durch den Pfad zu deinem Bild
    img = cv2.imread(image_path)

    if img is None:
        rospy.logerr(f"Bild konnte nicht geladen werden: {image_path}")
        return

    # 5. Definiere die Veröffentlichungsrate
    rate = rospy.Rate(1)  # 1 Hz (Einmal pro Sekunde)

    while not rospy.is_shutdown():
        try:
            # 6. Konvertiere das OpenCV-Bild in ein ROS-Image
            ros_image = bridge.cv2_to_imgmsg(img, encoding="bgr8")

            # 7. Veröffentliche das Bild
            image_pub.publish(ros_image)
            rospy.loginfo("Statisches Bild veröffentlicht.")

            # 8. Warte bis zur nächsten Veröffentlichung
            rate.sleep()

        except rospy.ROSInterruptException:
            rospy.loginfo("Bildpublizierer wurde gestoppt.")
            break

if __name__ == '__main__':
    try:
        publish_static_image()
    except rospy.ROSInterruptException:
        pass
