#!/usr/bin/env python

from geometry_msgs.msg import TransformStamped, PoseStamped
from math import sqrt, copysign
from sensor_msgs.msg import Joy

import openvr
import rospy
import tf.transformations
import tf2_geometry_msgs


def get_controller_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
    left = None
    right = None
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_Controller:
            role = vrsys.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_RightHand:
                right = i
            if role == openvr.TrackedControllerRole_LeftHand:
                left = i
    return left, right


def get_lighthouse_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
        lighthouse_ids = []
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_TrackingReference:
            lighthouse_ids.append(i)
    return lighthouse_ids


def get_generic_tracker_ids(vrsys=None):
    if vrsys is None:
        vrsys = openvr.VRSystem()
    else:
        vrsys = vrsys
        generic_tracker_ids = []
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vrsys.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_GenericTracker:
            generic_tracker_ids.append(i)
    return generic_tracker_ids


def from_matrix_to_pose_dict(matrix):
    pose = {}
    # From http://steamcommunity.com/app/358720/discussions/0/358417008714224220/#c359543542244499836
    position = {}
    position['x'] = matrix[0][3]
    position['y'] = matrix[1][3]
    position['z'] = matrix[2][3]
    q = {}
    q['w'] = sqrt(max(0, 1 + matrix[0][0] + matrix[1][1] + matrix[2][2])) / 2.0
    q['x'] = sqrt(max(0, 1 + matrix[0][0] - matrix[1][1] - matrix[2][2])) / 2.0
    q['y'] = sqrt(max(0, 1 - matrix[0][0] + matrix[1][1] - matrix[2][2])) / 2.0
    q['z'] = sqrt(max(0, 1 - matrix[0][0] - matrix[1][1] + matrix[2][2])) / 2.0
    q['x'] = copysign(q['x'], matrix[2][1] - matrix[1][2])
    q['y'] = copysign(q['y'], matrix[0][2] - matrix[2][0])
    q['z'] = copysign(q['z'], matrix[1][0] - matrix[0][1])
    pose['position'] = position
    pose['orientation'] = q
    return pose


def from_matrix_to_transform(matrix, stamp, frame_id, child_frame_id,
                             to_ros_reference_frame=True):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    # From http://steamcommunity.com/app/358720/discussions/0/358417008714224220/#c359543542244499836
    t.transform.translation.x = matrix[0][3]
    t.transform.translation.y = matrix[1][3]
    t.transform.translation.z = matrix[2][3]
    t.transform.rotation.w = sqrt(
        max(0, 1 + matrix[0][0] + matrix[1][1] + matrix[2][2])) / 2.0
    t.transform.rotation.x = sqrt(
        max(0, 1 + matrix[0][0] - matrix[1][1] - matrix[2][2])) / 2.0
    t.transform.rotation.y = sqrt(
        max(0, 1 - matrix[0][0] + matrix[1][1] - matrix[2][2])) / 2.0
    t.transform.rotation.z = sqrt(
        max(0, 1 - matrix[0][0] - matrix[1][1] + matrix[2][2])) / 2.0
    t.transform.rotation.x = copysign(
        t.transform.rotation.x, matrix[2][1] - matrix[1][2])
    t.transform.rotation.y = copysign(
        t.transform.rotation.y, matrix[0][2] - matrix[2][0])
    t.transform.rotation.z = copysign(
        t.transform.rotation.z, matrix[1][0] - matrix[0][1])

    if to_ros_reference_frame:
        tr = t.transform.translation
        rot = t.transform.rotation
        tr.z, tr.y, tr.x = tr.y, -tr.x, -tr.z
        rot.z, rot.y, rot.x = rot.y, -rot.x, -rot.z
    return t


def from_controller_to_joy(prev_unPacketNum,
                           pControllerState,
                           stamp,
                           frame_id):
    # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState

    d = {}
    d['unPacketNum'] = pControllerState.unPacketNum
    # on trigger .y is always 0.0 says the docs
    d['trigger'] = pControllerState.rAxis[1].x
    # 0.0 on trigger is fully released
    # -1.0 to 1.0 on joystick and trackpads
    d['trackpad_x'] = pControllerState.rAxis[0].x
    d['trackpad_y'] = pControllerState.rAxis[0].y
    # These are published and always 0.0
    # for i in range(2, 5):
    #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
    #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
    d['ulButtonPressed'] = pControllerState.ulButtonPressed
    d['ulButtonTouched'] = pControllerState.ulButtonTouched
    # To make easier to understand what is going on
    # Second bit marks menu button
    d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
    # 32 bit marks trackpad
    d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
    d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
    # third bit marks grip button
    d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
    # System button can't be read, if you press it
    # the controllers stop reporting

    j = Joy()
    j.header.frame_id = frame_id
    j.header.stamp = stamp
    # Axes
    # Trigger, Trackpad X, Trackpad Y
    j.axes = [
        d['trigger'],
        d['trackpad_x'],
        d['trackpad_y']
    ]
    # Buttons
    # Trigger, Trackpad touched, trackpad pressed, menu, grip
    j.buttons = [
        d['trigger'] == 1.0,
        d['trackpad_touched'],
        d['trackpad_pressed'],
        d['menu_button'],
        d['grip_button']
    ]
    new_msg = prev_unPacketNum != d['unPacketNum']
    return new_msg, j


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

    # T_2_3 needs to be a PoseStamped type, for convenient transformation.
    # It still holds the same data.
    P_2_3 = PoseStamped()
    P_2_3.header = T_2_3.header
    P_2_3.pose.position = T_2_3.transform.translation
    P_2_3.pose.orientation = T_2_3.transform.rotation

    # T_1_3 = T_2_1^-1 * T_1_3
    P_1_3 = tf2_geometry_msgs.do_transform_pose(P_2_3, \
        T_1_2)
    # Back to a tf data type
    T_1_3 = TransformStamped()
    T_1_3.transform.translation = P_1_3.pose.position
    T_1_3.transform.rotation = P_1_3.pose.orientation

    return T_1_3