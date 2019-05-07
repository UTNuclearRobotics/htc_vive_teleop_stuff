#!/usr/bin/env python

import time
import openvr
from math import sqrt, copysign
import pprint
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

"""
Get the initial HMD pose with respect to lighthouse_0.
Republish it repeatedly.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>, Andy Zelenak
"""


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


if __name__ == '__main__':
    print("===========================")
    print("Initializing OpenVR...")
    retries = 0
    max_init_retries = 4
    while retries < max_init_retries:
        try:
            openvr.init(openvr.VRApplication_Scene)
            break
        except openvr.OpenVRError as e:
            print("Error when initializing OpenVR (try {} / {})".format(
                  retries + 1, max_init_retries))
            print(e)
            retries += 1
            time.sleep(2.0)
    else:
        print("Could not initialize OpenVR, aborting.")
        print("Make sure the system is correctly plugged, you can also try")
        print("to do:")
        print("killall -9 vrcompositor vrmonitor vrdashboard")
        print("Before running this program again.")
        exit(0)

    print("Success!")
    print("===========================")

    print("VRSystem...")
    vrsystem = openvr.VRSystem()

    lighthouse_ids = get_lighthouse_ids(vrsystem)
    print("Lighthouse IDs: " + str(lighthouse_ids))

    poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
    poses = poses_t()

    pp = pprint.PrettyPrinter(indent=4)

    print("Initializing ROS...")
    rospy.init_node('HTCViveROS')
    print("Creating TransformBroadcaster...")

    br = tf2_ros.TransformBroadcaster()

    # Give a bit of time to initialize...
    rospy.sleep(3.0)
    print("Running!")

    # Get the initial headset pose, 
    # /hmd, w.r.t. /lighthouse_0.
    # Both are received with respect to /world, so it takes some algebra to get
    # T(lighthouse_0 to hmd)
    # T_lighthouse_0_hmd = T_lighthouse_0_world * T_world_hmd
    # = T_world_lighthouse_0 ^ -1 * T_world_hmd
    T_lighthouse0_hmd = TransformStamped()

    # A type for receiving data from the headset:
    poses = poses_t()
    # All-zero pose and empty frame_id is a good indicator that a msg hasn't been received
    while (T_lighthouse0_hmd.header.frame_id == ''):
        vrsystem.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            len(poses),
            poses)
        # Get hmd transform:
        # Hmd is always 0
        T_world_hmd = poses[0].mDeviceToAbsoluteTracking
        rospy.loginfo(T_world_hmd)

        # Get lighthouse_0 transform:
        T_world_lighthouse_0 = None
        for idx, _id in enumerate(lighthouse_ids):
            # Only want lighthouse_0, not lighthouse_1
            if (idx == 0):
                T_world_lighthouse_0 = poses[_id].mDeviceToAbsoluteTracking
                rospy.logwarn(T_world_lighthouse_0)

        # Invert and multiply to get T_lighthouse_0_world
        matrix = T_world_lighthouse_0.inverse() * T_world_hmd
        T_lighthouse0_hmd = from_matrix_to_transform(matrix, rospy.Time.now(), 
            "lighthouse_0", "hmd")
        rospy.loginfo(T_lighthouse0_hmd)

    # Republish this headset pose
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
        T_world_hmd.header.stamp = rospy.Time.now()
        br.sendTransform(T_world_hmd)

    openvr.shutdown()
