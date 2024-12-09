import rospy
from geometry_msgs.msg import PoseStamped
import random

def publish_goal_pose():
    # ROS 1 Node initialisieren
    rospy.init_node('goal_pose_publisher', anonymous=True)
    
    # Publisher für das /goal_pose-Topic
    goal_publisher = rospy.Publisher('/goal_pose', PoseStamped, queue_size=10)
    
    # Veröffentlichungsrate (1 Hz)
    rate = rospy.Rate(1)

    rospy.loginfo("Goal pose publisher is running. Publishing to /goal_pose.")

    while not rospy.is_shutdown():
        # Erstelle eine PoseStamped-Nachricht
        goal_pose = PoseStamped()

        # Setze den Header
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "map"

        # Generiere zufällige Zielkoordinaten (z. B. innerhalb von 10x10 Metern)
        goal_pose.pose.position.x = 180#random.randint(0, 99)
        goal_pose.pose.position.y = 90#random.randint(0, 99)
        goal_pose.pose.position.z = 0  # Planare Bewegung

        # Feste Orientierung (z. B. kein Drehmoment)
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        # Nachricht veröffentlichen
        goal_publisher.publish(goal_pose)
        rospy.loginfo(f"Published goal pose: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")

        # Warte, bis die nächste Nachricht gesendet wird
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal_pose()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down goal pose publisher.")

