#!/usr/bin/env python

from geometry_msgs.msg import TransformStamped, PoseStamped

import rospy
import tf.transformations
import tf2_geometry_msgs

def calculate_relative_transformation(T_2_1, T_2_3):
    # Args:
    # T_2_1: a transformation matrix from frame 2 to frame 1
    # T_2_3: a transformation matrix from frame 2 to frame 3
    # Returns:
    # T_1_3: a transformation matrix from frame 1 to frame 3

    # Requires conversion from TransformStamped to lists
    trans = [T_2_1.transform.translation.x, \
            T_2_1.transform.translation.y, \
            T_2_1.transform.translation.z]
    rot = [T_2_1.transform.rotation.x, \
            T_2_1.transform.rotation.y, \
            T_2_1.transform.rotation.z, \
            T_2_1.transform.rotation.w]
    transform = tf.transformations.concatenate_matrices( \
        tf.transformations.translation_matrix(trans), \
        tf.transformations.quaternion_matrix(rot))
    T_1_2 = TransformStamped()
    T_1_2.header.stamp = rospy.Time.now()
    inverse = tf.transformations.inverse_matrix(transform)
    T_1_2.transform.translation.x = inverse[0][3]
    T_1_2.transform.translation.y = inverse[1][3]
    T_1_2.transform.translation.z = inverse[2][3]
    q = tf.transformations.quaternion_from_matrix(inverse)
    T_1_2.transform.rotation.x = q[0]
    T_1_2.transform.rotation.y = q[1]
    T_1_2.transform.rotation.z = q[2]
    T_1_2.transform.rotation.w = q[3]

    # T_world_hmd needs to be a PoseStamped type, for convenient transformation.
    # It still holds the same data.
    P_2_3 = PoseStamped()
    P_2_3.header = T_2_3.header
    P_2_3.pose.position = T_2_3.transform.translation
    P_2_3.pose.orientation = T_2_3.transform.rotation

    # T_lighthouse0_hmd = T_2_1^-1 * T_1_3
    P_1_3 = tf2_geometry_msgs.do_transform_pose(P_2_3, \
        T_1_2)
    # Back to a tf data type
    T_1_3 = TransformStamped()
    T_1_3.transform.translation = P_1_3.pose.position
    T_1_3.transform.rotation = P_1_3.pose.orientation

    return T_1_3
