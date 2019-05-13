#!/usr/bin/env python

import common_vive_functions
from geometry_msgs.msg import TransformStamped
import openvr
import pprint
import rospy
import tf2_ros
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import time

"""
Getting poses and buttons into ROS.

Publishes the following poses w.r.t. the /hmd ROS tf frame:
/hmd is the parent of /left_controller, and /right_controller.
Button presses in Joy topics /vive_left /vive_right .

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>, Andy Zelenak
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

    # Get the /left_controller pose w.r.t. /hmd
    # Both are received with respect to /world, so it takes some algebra to get
    # T(hmd to left_controller)
    # T_hmd_left_controller = T_hmd_world * T_world_left_controller
    #                   = T_world_hmd^-1 * T_world_left_controller
    T_hmd_left_controller = TransformStamped()

    # A type for receiving data from the headset:
    poses = poses_t()

    # Internet says the tracking can be up until 250Hz
    r = rospy.Rate(250)
    while not rospy.is_shutdown():
        vrsystem.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            len(poses),
            poses)
        # Get hmd transform:
        # Hmd is always 0
        matrix = poses[0].mDeviceToAbsoluteTracking
        T_world_hmd = common_vive_functions.from_matrix_to_transform(matrix, rospy.Time.now(),
                    "world", "hmd")

        # Get left_controller transform:
        T_world_left_controller = None
        if left_id:
            matrix = poses[left_id].mDeviceToAbsoluteTracking
            T_world_left_controller = common_vive_functions.from_matrix_to_transform(matrix,
                                                 rospy.Time.now(),
                                                 "world",
                                                 "left_controller")
        # T_hmd_left_controller = T_world_hmd^-1 * T_world_left_controller
        T_hmd_left_controller = common_vive_functions.calculate_relative_transformation(T_world_hmd, T_world_left_controller)
        T_hmd_left_controller.header.stamp = rospy.Time.now()
        T_hmd_left_controller.header.frame_id = "hmd"
        T_hmd_left_controller.child_frame_id = "left_controller"

        r.sleep()
        T_hmd_left_controller.header.stamp = rospy.Time.now()
        br.sendTransform(T_hmd_left_controller)

    openvr.shutdown()
