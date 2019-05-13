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
Get the initial HMD pose with respect to lighthouse_0.
Republish it repeatedly.

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

    lighthouse_ids = common_vive_functions.get_lighthouse_ids(vrsystem)
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

    # Get the initial headset pose /hmd w.r.t. /lighthouse_0.
    # Both are received with respect to /world, so it takes some algebra to get
    # T(lighthouse_0 to hmd)
    # T_lighthouse0_hmd = T_lighthouse0_world * T_world_hmd
    #                   = T_world_lighthouse_0 ^ -1 * T_world_hmd
    T_lighthouse0_hmd = TransformStamped()

    # A type for receiving data from the headset:
    poses = poses_t()
    # All-zero pose and empty frame_id is a good indicator that a msg hasn't been received
    while (T_lighthouse0_hmd.header.frame_id == '') and (not rospy.is_shutdown()):
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

        # Get lighthouse_0 transform:
        T_world_lighthouse0 = None
        for idx, _id in enumerate(lighthouse_ids):
            # Only want lighthouse_0, not lighthouse_1
            if (idx == 0):
                matrix = poses[_id].mDeviceToAbsoluteTracking
                T_world_lighthouse0 = common_vive_functions.from_matrix_to_transform(matrix, rospy.Time.now(), 
                    "world", "lighthouse_0")

        # T_lighthouse0_hmd = T_world_lighthouse0^-1 * T_world_hmd
        T_lighthouse0_hmd = common_vive_functions.calculate_relative_transformation(T_world_lighthouse0, T_world_hmd)
        T_lighthouse0_hmd.header.stamp = rospy.Time.now()
        T_lighthouse0_hmd.header.frame_id = "lighthouse_0"
        T_lighthouse0_hmd.child_frame_id = "hmd"

    # We got the initial pose -- can release control of VR now
    openvr.shutdown()

    # Republish this headset pose
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
        T_lighthouse0_hmd.header.stamp = rospy.Time.now()
        br.sendTransform(T_lighthouse0_hmd)
