import rospy
from geometry_msgs.msg import PoseStamped
import random

def publish_end_pose():
    # ROS 1 Node initialisieren
    rospy.init_node('end_pose_publisher', anonymous=True)
    
    # Publisher für das /end_pose-Topic
    end_publisher = rospy.Publisher('/end_pose', PoseStamped, queue_size=10)
    
    # Veröffentlichungsrate (1 Hz)
    rate = rospy.Rate(0.1)

    rospy.loginfo("end pose publisher is running. Publishing to /end_pose.")

    while not rospy.is_shutdown():
        # Erstelle eine PoseStamped-Nachricht
        end_pose = PoseStamped()

        # Setze den Header
        end_pose.header.stamp = rospy.Time.now()
        end_pose.header.frame_id = "map"

        # Generiere zufällige Zielkoordinaten (z. B. innerhalb von 10x10 Metern)
        end_pose.pose.position.x = 5#random.randint(0, 99)
        end_pose.pose.position.y = 5#random.randint(0, 99)
        end_pose.pose.position.z = 0  # Planare Bewegung

        # Feste Orientierung (z. B. kein Drehmoment)
        end_pose.pose.orientation.x = 0.0
        end_pose.pose.orientation.y = 0.0
        end_pose.pose.orientation.z = 0.0
        end_pose.pose.orientation.w = 1.0

        # Nachricht veröffentlichen
        end_publisher.publish(end_pose)
        rospy.loginfo(f"Published end pose: x={end_pose.pose.position.x}, y={end_pose.pose.position.y}")

        # Warte, bis die nächste Nachricht gesendet wird
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_end_pose()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down end pose publisher.")

