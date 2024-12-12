import rospy
from geometry_msgs.msg import PointStamped
import random

def publish_goal_pose():
    # ROS 1 Node initialisieren
    rospy.init_node('goal_pose_publisher', anonymous=True)
    
    # Publisher für das /goal_pose-Topic
    goal_publisher = rospy.Publisher('/camera_pixel_ar_tags', PointStamped, queue_size=10)
    
    # Veröffentlichungsrate (1 Hz)
    rate = rospy.Rate(1)

    rospy.loginfo("Goal pose publisher is running. Publishing to /goal_pose.")

    while not rospy.is_shutdown():
        # Erstelle eine PoseStamped-Nachricht
        goal_pose = PointStamped()

        # Setze den Header
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = str(0)

        # Generiere zufällige Zielkoordinaten (z. B. innerhalb von 10x10 Metern)
        goal_pose.point.x = 175#random.randint(0, 99)
        goal_pose.point.y = 348#random.randint(0, 99)
        #goal_pose.pose.position.z = 0  # Planare Bewegung

        # Feste Orientierung (z. B. kein Drehmoment)
    #    goal_pose.pose.orientation.x = 0.0
     #   goal_pose.pose.orientation.y = 0.0
     #   goal_pose.pose.orientation.z = 0.0
     #   goal_pose.pose.orientation.w = 1.0

        # Nachricht veröffentlichen
        goal_publisher.publish(goal_pose)
        rospy.loginfo(f"Published goal pose: x={goal_pose.point.x}, y={goal_pose.point.y}")

        # Warte, bis die nächste Nachricht gesendet wird
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal_pose()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down goal pose publisher.")

