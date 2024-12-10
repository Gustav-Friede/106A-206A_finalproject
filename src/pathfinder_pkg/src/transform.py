#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

def transform_pose(pose_in):
    # 1. Skalierungsfaktoren (Skalierung im ersten und zweiten Koordinatensystem)
    s_x, s_y = 2, 3  # Skalierungsfaktoren im ersten System
    s_x2, s_y2 = 1, 2  # Skalierungsfaktoren im zweiten System

    # 2. Übersetzung (Translation des Ursprungs)
    T_x, T_y = -2, 4  # Verschiebung des Ursprungs im zweiten Koordinatensystem

    # 3. Rotationswinkel (in Grad)
    theta = 45  # Winkel in Grad

    # Umwandlung des Winkels in Bogenmaß
    theta_rad = np.radians(theta)

    # 4. Erstelle die Rotationsmatrix
    R = np.array([[np.cos(theta_rad), -np.sin(theta_rad)],
                  [np.sin(theta_rad), np.cos(theta_rad)]])

    # 5. Extrahiere die Position und Orientierung der Pose (Position und Quaternions)
    x1, y1 = pose_in.pose.position.x, pose_in.pose.position.y
    qx, qy, qz, qw = pose_in.pose.orientation.x, pose_in.pose.orientation.y, pose_in.pose.orientation.z, pose_in.pose.orientation.w

    # Rotationsmatrix für Quaternion zu Rotationsmatrix (optional)
    # Hier wird angenommen, dass die Eingabe eine einfache 2D-Orientierung ist, und der Quaternion wird ignoriert
    # Alternativ könntest du `tf` verwenden, um Quaternionen in Matrizen zu konvertieren, falls nötig.
    # Hier ist es eine Annahme, dass nur die Orientierung im 2D-Kosmos geht.

    # 6. Transformation der Position:
    # Zuerst Rotation anwenden
    rotated_point = np.dot(R, np.array([x1, y1]))

    # Skalierung anwenden (skalierter Punkt im neuen System)
    scaled_point = rotated_point * np.array([s_x2 / s_x, s_y2 / s_y])

    # Translation anwenden (verschiebender Punkt im neuen System)
    transformed_position = scaled_point + np.array([T_x, T_y])

    # 7. Erstelle eine neue PoseStamped-Nachricht für das zweite Koordinatensystem
    pose_out = PoseStamped()

    # Setze die transformierte Position
    pose_out.pose.position.x = transformed_position[0]
    pose_out.pose.position.y = transformed_position[1]

    # Quaternion für keine Rotation (wir setzen es einfach auf 0)
    pose_out.pose.orientation.x = 0.0
    pose_out.pose.orientation.y = 0.0
    pose_out.pose.orientation.z = 0.0
    pose_out.pose.orientation.w = 1.0

    return pose_out

def callback(pose_in):
    # Transformation der Pose
    pose_out = transform_pose(pose_in)
    
    # Ausgabe der transformierten Pose
    rospy.loginfo("Transformierte Pose: x: %f, y: %f", pose_out.pose.position.x, pose_out.pose.position.y)

    # Publiziere die transformierte Pose
    pose_pub.publish(pose_out)

def listener():
    # 1. Initialisiere den ROS-Knoten
    rospy.init_node('pose_transformer', anonymous=True)

    # 2. Subscriber für PoseStamped-Nachrichten
    rospy.Subscriber('/input_pose', PoseStamped, callback)

    # 3. Publisher für transformierte Pose
    global pose_pub
    pose_pub = rospy.Publisher('/output_pose', PoseStamped, queue_size=10)

    # 4. ROS-Schleife starten
    rospy.spin()

if __name__ == '__main__':
    listener()
