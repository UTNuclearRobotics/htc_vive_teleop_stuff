#!/usr/bin/env python

import common_vive_functions
import copy
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

# Query the VR system for a transform from HMD to lighthouse0
def get_hmd_to_lighthouse0_transform():
    T_hmd_lighthouse0 = TransformStamped()
    # All-zero pose and empty frame_id is a good indicator that a msg hasn't been received
    while (T_hmd_lighthouse0.header.frame_id == '') and (not rospy.is_shutdown()):
        vrsystem.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0,
            len(poses),
            poses)
        # Get hmd transform:
        # Hmd is always 0
        T_world_hmd = poses[0].mDeviceToAbsoluteTracking

        # Get lighthouse0 transforms
        T_world_lighthouse = poses[lighthouse_ids[0]].mDeviceToAbsoluteTracking

        # T_hmd_lighthouse0 = T_world_hmd^-1 * T_world_lighthouse0
        T_hmd_lighthouse0 = common_vive_calcs.calculate_relative_transformation(T_world_hmd, \
            T_world_lighthouse, "hmd", "lighthouse0")

    return T_hmd_lighthouse0


# Given a list of TransformStampeds, return the average of each element of the transform
def calculate_average_transformation_elements(T_list):
    avg_transform = TransformStamped()

    # Sum
    for t in T_list:
        avg_transform.transform.translation.x = avg_transform.transform.translation.x + t.transform.translation.x
        avg_transform.transform.translation.y = avg_transform.transform.translation.y + t.transform.translation.y
        avg_transform.transform.translation.z = avg_transform.transform.translation.z + t.transform.translation.z

        avg_transform.transform.rotation.x = avg_transform.transform.rotation.x + t.transform.rotation.x
        avg_transform.transform.rotation.y = avg_transform.transform.rotation.y + t.transform.rotation.y
        avg_transform.transform.rotation.z = avg_transform.transform.rotation.z + t.transform.rotation.z
        avg_transform.transform.rotation.w = avg_transform.transform.rotation.w + t.transform.rotation.w

    # Divide by number of datapoints
    avg_transform.transform.translation.x = avg_transform.transform.translation.x / len(T_list)
    avg_transform.transform.translation.y = avg_transform.transform.translation.y / len(T_list)
    avg_transform.transform.translation.z = avg_transform.transform.translation.z / len(T_list)

    avg_transform.transform.rotation.x = avg_transform.transform.rotation.x / len(T_list)
    avg_transform.transform.rotation.y = avg_transform.transform.rotation.y / len(T_list)
    avg_transform.transform.rotation.z = avg_transform.transform.rotation.z / len(T_list)
    avg_transform.transform.rotation.w = avg_transform.transform.rotation.w / len(T_list)

    return avg_transform


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

    # Object for common calculations
    common_vive_calcs = common_vive_functions.CommonViveFunctions()

    lighthouse_ids = common_vive_calcs.get_lighthouse_ids(vrsystem)
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

    # Get the initial headset pose /lighthouse_0 w.r.t. /hmd.
    # Both are received with respect to /world, so it takes some algebra to get

    # Threshold for comparison of transformation matrix elements
    MATRIX_ELEMENT_EPSILON = 0.001

    transform_datapoints_match = False
    while (not transform_datapoints_match) and (not rospy.is_shutdown()):
        # Collect several transform datapoints
        num_datapoints = 250
        transform_datapoints = []
        for i in range(num_datapoints):
            t = get_hmd_to_lighthouse0_transform()
            # Care with Python and appending references rather than copies!
            # https://ask.sagemath.org/question/25998/why-does-append-overwriteclobber-every-existing-element-of-a-list-with-the-one-that-was-just-appended/
            transform_datapoints.append(copy.deepcopy(t))

        # Calculate the average value of each element in the transformation matrices
        T_avg = calculate_average_transformation_elements(transform_datapoints)

        # Compare each of the transform datapoints to the average. If all match, the initial transform is stable
        transform_datapoints_match = True

    # We got the initial pose -- can release control of VR now
    openvr.shutdown()

    # Republish this headset pose
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        r.sleep()
        T_avg.header.stamp = rospy.Time.now()
        br.sendTransform(T_avg)
