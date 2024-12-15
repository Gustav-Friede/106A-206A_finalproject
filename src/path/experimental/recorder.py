#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraRecorder:
    def __init__(self):
        # Initialisiere den ROS-Node
        rospy.init_node('camera_recorder', anonymous=True)
        
        # Topic, das den Kamerafeed veröffentlicht
        self.camera_topic = rospy.get_param('~camera_topic', '/raw_birds_eye_image')
        
        # Bildkonverter (ROS -> OpenCV)
        self.bridge = CvBridge()

        # Subscriber für den Kamerafeed
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)

        # Videodatei schreiben
        self.video_writer = None
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec für AVI-Dateien
        self.output_file = rospy.get_param('~output_file', 'output.avi')
        self.fps = rospy.get_param('~fps', 30)
        self.frame_size = None

        rospy.loginfo("Camera Recorder gestartet. Abonniere Topic: %s", self.camera_topic)

    def image_callback(self, data):
        try:
            # Konvertiere das ROS-Bild in ein OpenCV-Bild
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr("Fehler beim Konvertieren des Bildes: %s", str(e))
            return

        # Initialisiere den VideoWriter, wenn noch nicht geschehen
        if self.video_writer is None:
            height, width, _ = cv_image.shape
            self.frame_size = (width, height)
            self.video_writer = cv2.VideoWriter(self.output_file, self.fourcc, self.fps, self.frame_size)
            rospy.loginfo("Videoaufnahme gestartet. Datei: %s", self.output_file)

        # Schreibe das Bild in die Videodatei
        self.video_writer.write(cv_image)

    def cleanup(self):
        # Beende den VideoWriter und schließe die Datei
        if self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo("Videoaufnahme beendet. Datei gespeichert: %s", self.output_file)

if __name__ == '__main__':
    try:
        recorder = CameraRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS-Node wurde beendet.")
    finally:
        recorder.cleanup()
