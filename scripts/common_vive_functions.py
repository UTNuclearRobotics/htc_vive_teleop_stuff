#!/usr/bin/env python

from geometry_msgs.msg import TransformStamped, PoseStamped
from math import sqrt, copysign
from sensor_msgs.msg import Joy

import numpy as np
import openvr
import rospy
import tf.transformations
import tf2_geometry_msgs

class CommonViveFunctions:

    def __init__(self):

        # Preallocate variables that are used often
        self.t = TransformStamped()
        self.T_2_1_numpy = None
        self.T_2_3_numpy = None
        self.T_1_3_numpy = None
        self.T_numpy = None


    def get_controller_ids(self, vrsys=None):
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


    def get_lighthouse_ids(self, vrsys=None):
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


    def get_generic_tracker_ids(self, vrsys=None):
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


    def from_matrix_to_pose_dict(self, matrix):
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


    def from_matrix_to_transform(self, matrix, stamp, frame_id, child_frame_id,
                                 to_ros_reference_frame=True):
        self.t.header.stamp = stamp
        self.t.header.frame_id = frame_id
        self.t.child_frame_id = child_frame_id
        # From http://steamcommunity.com/app/358720/discussions/0/358417008714224220/#c359543542244499836
        self.t.transform.translation.x = matrix[0][3]
        self.t.transform.translation.y = matrix[1][3]
        self.t.transform.translation.z = matrix[2][3]
        self.t.transform.rotation.w = sqrt(
            max(0, 1 + matrix[0][0] + matrix[1][1] + matrix[2][2])) / 2.0
        self.t.transform.rotation.x = sqrt(
            max(0, 1 + matrix[0][0] - matrix[1][1] - matrix[2][2])) / 2.0
        self.t.transform.rotation.y = sqrt(
            max(0, 1 - matrix[0][0] + matrix[1][1] - matrix[2][2])) / 2.0
        self.t.transform.rotation.z = sqrt(
            max(0, 1 - matrix[0][0] - matrix[1][1] + matrix[2][2])) / 2.0
        self.t.transform.rotation.x = copysign(
            self.t.transform.rotation.x, matrix[2][1] - matrix[1][2])
        self.t.transform.rotation.y = copysign(
            self.t.transform.rotation.y, matrix[0][2] - matrix[2][0])
        self.t.transform.rotation.z = copysign(
            self.t.transform.rotation.z, matrix[1][0] - matrix[0][1])

        if to_ros_reference_frame:
            tr = self.t.transform.translation
            rot = self.t.transform.rotation
            tr.z, tr.y, tr.x = tr.y, -tr.x, -tr.z
            rot.z, rot.y, rot.x = rot.y, -rot.x, -rot.z
        return self.t


    def from_controller_to_joy(self,
                                prev_unPacketNum,
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


    def calculate_relative_transformation(self,
                                            T_2_1,
                                            T_2_3,
                                            frame_1_name,
                                            frame_3_name):
        # Args:
        # T_2_1: a transformation matrix from frame 2 to frame 1. Only stores the first three rows
        # T_2_3: a transformation matrix from frame 2 to frame 3. Only stores the first three rows
        # frame_1_name: ROS name of frame 1 (header.frame_id)
        # frame_3_name: ROS name of frame 3 (header.frame_id)
        # Returns:
        # T_1_3: a TransformStamped from frame 1 to frame 3

        # Input transformation matrices only include the first 3 rows, to save memory.
        # Add the fourth row and convert to numpy array.
        self.T_2_1_numpy = self.convert_HmdMatrix34_t_to_numpy(T_2_1)

        self.T_2_3_numpy = self.convert_HmdMatrix34_t_to_numpy(T_2_3)
        # T_1_3 = T_2_1^-1 * T_2_3
        self.T_1_3_numpy = np.dot(np.linalg.inv(self.T_2_1_numpy), self.T_2_3_numpy)

        # To a tf data type
        self.T_1_3 = self.from_matrix_to_transform(self.T_1_3_numpy, rospy.Time.now(), frame_1_name, frame_3_name)

        return self.T_1_3


    def convert_HmdMatrix34_t_to_numpy(self, hmd_matrix):
        # Convert this HMD transformation matrix to numpy.
        # The transformation matrix only stores the first 3 rows to save memory.
        # Returns a 4x4 transformation matrix.

        self.T_numpy = np.array([ \
            [hmd_matrix[0][0], hmd_matrix[0][1], hmd_matrix[0][2], hmd_matrix[0][3]], \
            [hmd_matrix[1][0], hmd_matrix[1][1], hmd_matrix[1][2], hmd_matrix[1][3]], \
            [hmd_matrix[2][0], hmd_matrix[2][1], hmd_matrix[2][2], hmd_matrix[2][3]], \
            [0, 0, 0, 1] \
            ])

        return self.T_numpy