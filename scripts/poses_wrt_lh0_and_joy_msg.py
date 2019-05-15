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

Publishes the following poses w.r.t. the /lighthouse0 ROS tf frame:
/lighthouse0 is the parent of /left_controller, and /right_controller.
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
        while (left_id is None or right_id is None) and (not rospy.is_shutdown()):
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

    # Get the /left_controller pose w.r.t. /lighthouse0
    # Both are received with respect to /world, so it takes some algebra to get
    # T(lighthouse0 to left_controller)
    # T_lighthouse0_left_controller = T_lighthouse0_world * T_world_controller
    #                   = T_world_lighthouse0^-1 * T_world_controller
    T_lighthouse0_left_controller = TransformStamped()

    # A type for receiving data from the headset:
    poses = poses_t()

    # Internet says the tracking can be up until 250Hz
    r = rospy.Rate(100)

    # Preallocate
    T_world_controller = None
    matrix = None
    T_world_lighthouse0 = None
    T_world_controller = None
    T_lighthouse0_right_controller = None
    T_lighthouse0_left_controller = None

    while not rospy.is_shutdown():
        r.sleep()
        vrsystem.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            len(poses),
            poses)

        # Get lighthouse0 transform
        T_world_lighthouse0 = poses[lighthouse_ids[0]].mDeviceToAbsoluteTracking

        # Get left_controller transform:
        if left_id:
            T_world_controller = poses[left_id].mDeviceToAbsoluteTracking
        # T_lighthouse0_left_controller = T_world_lighthouse0^-1 * T_world_controller
        T_lighthouse0_left_controller = common_vive_functions.calculate_relative_transformation(T_world_lighthouse0, \
            T_world_controller, "lighthouse0", "left_controller")
        br.sendTransform(T_lighthouse0_left_controller)

        # Get right_controller transform:
        if right_id:
            T_world_controller = poses[right_id].mDeviceToAbsoluteTracking
        # T_lighthouse0_right_controller = T_world_lighthouse0^-1 * T_world_controller
        T_lighthouse0_right_controller = common_vive_functions.calculate_relative_transformation(T_world_lighthouse0, \
            T_world_controller, "lighthouse0", "right_controller")
        br.sendTransform(T_lighthouse0_right_controller)

    openvr.shutdown()
