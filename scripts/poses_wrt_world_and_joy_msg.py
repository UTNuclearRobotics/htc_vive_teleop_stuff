#!/usr/bin/env python

import common_vive_functions
import time
import openvr
from math import sqrt, copysign
import pprint
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

"""
Getting poses and buttons into ROS.

Publishes the following poses w.r.t. the /world ROS tf frame:
/world is the parent of /lighthouse_0, /lighthouse_1, /hmd, /left_controller, and /right_controller.
Button presses in Joy topics /vive_left /vive_right .

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


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

    left_id, right_id = None, None
    print("===========================")
    print("Waiting for controllers...")
    try:
        while left_id is None or right_id is None:
            left_id, right_id = common_vive_functions.get_controller_ids(vrsystem)
            if left_id and right_id:
                break
            print("Waiting for controllers...")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Control+C pressed, shutting down...")
        openvr.shutdown()

    print("Left controller ID: " + str(left_id))
    print("Right controller ID: " + str(right_id))
    print("===========================")

    lighthouse_ids = common_vive_functions.get_lighthouse_ids(vrsystem)
    print("Lighthouse IDs: " + str(lighthouse_ids))

    generic_tracker_ids = common_vive_functions.get_generic_tracker_ids(vrsystem)
    print("Generic tracker IDs:" + str(generic_tracker_ids))

    poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
    poses = poses_t()

    pp = pprint.PrettyPrinter(indent=4)

    print("Initializing ROS...")
    rospy.init_node('HTCViveROS')
    print("Creating TransformBroadcaster...")

    br = tf2_ros.TransformBroadcaster()
    joy_left_pub = rospy.Publisher('vive_left', Joy, queue_size=1)
    prev_unPacketNum_left = 0
    joy_right_pub = rospy.Publisher('vive_right', Joy, queue_size=1)
    prev_unPacketNum_right = 0
    # Give a bit of time to initialize...
    rospy.sleep(3.0)
    print("Running!")

    # Vibration topic for each controller
    def vibration_cb(data, controller_id):
        # strength 0-3999, data contains a float expected
        # to be in between 0.0 and 1.0
        if data.data > 1.0:
            strength = 3999
        elif data.data < 0.0:
            strength = 0
        else:
            strength = int(data.data * 3999)
        vrsystem.triggerHapticPulse(controller_id, 0, strength)

    vib_left = rospy.Subscriber('vive_left_vibration', Float64, vibration_cb,
                                callback_args=left_id, queue_size=1)

    vib_right = rospy.Subscriber('vive_right_vibration', Float64, vibration_cb,
                                 callback_args=right_id, queue_size=1)

    # Internet says the tracking can be up until 250Hz
    r = rospy.Rate(250)
    while not rospy.is_shutdown():
        r.sleep()
        poses = poses_t()
        vrsystem.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            len(poses),
            poses)

        now = rospy.Time.now()
        transforms = []
        # Hmd is always 0
        matrix = poses[0].mDeviceToAbsoluteTracking
        hmd_pose = common_vive_functions.from_matrix_to_transform(matrix, now, "world", "hmd")
        transforms.append(hmd_pose)

        # print("Hmd:")
        # pp.pprint(hmd_pose)

        for idx, _id in enumerate(lighthouse_ids):
            matrix = poses[_id].mDeviceToAbsoluteTracking
            lhouse_pose = common_vive_functions.from_matrix_to_transform(matrix,
                                                   now,
                                                   "world",
                                                   "lighthouse_" + str(idx))
            transforms.append(lhouse_pose)
            # print("Lighthouse #" + str(idx) + " :")
            # pp.pprint(lhouse_pose)

        if left_id:
            matrix = poses[left_id].mDeviceToAbsoluteTracking
            left_pose = common_vive_functions.from_matrix_to_transform(matrix,
                                                 now,
                                                 "world",
                                                 "left_controller")
            transforms.append(left_pose)
            result, pControllerState = vrsystem.getControllerState(left_id)
            new_msg, j = common_vive_functions.from_controller_to_joy(prev_unPacketNum_left,
                                                pControllerState,
                                                now,
                                                "left_controller")
            prev_unPacketNum_left = pControllerState.unPacketNum
            if new_msg:
                joy_left_pub.publish(j)
            # print("Left controller:")
            # # pp.pprint(d)
            # pp.pprint(left_pose)

        if right_id:
            matrix = poses[right_id].mDeviceToAbsoluteTracking
            right_pose = common_vive_functions.from_matrix_to_transform(matrix,
                                                  now,
                                                  "world",
                                                  "right_controller")
            transforms.append(right_pose)
            result, pControllerState = vrsystem.getControllerState(right_id)
            new_msg, j = common_vive_functions.from_controller_to_joy(prev_unPacketNum_right,
                                                pControllerState,
                                                now,
                                                "right_controller")
            prev_unPacketNum_right = pControllerState.unPacketNum
            if new_msg:
                joy_right_pub.publish(j)
            # print("Right controller:")
            # # pp.pprint(d)
            # pp.pprint(right_pose)

        for idx, _id in enumerate(generic_tracker_ids):
            matrix = poses[_id].mDeviceToAbsoluteTracking
            gen_track_pose = common_vive_functions.from_matrix_to_transform(matrix,
                                                      now,
                                                      "world",
                                                      "generic_tracker_" + str(idx))
            transforms.append(gen_track_pose)
            # print("Generic tracker #" + str(idx) + " :")
            # pp.pprint(gen_track_pose)

        br.sendTransform(transforms)

    openvr.shutdown()
