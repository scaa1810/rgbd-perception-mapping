#!/usr/bin/env python3


import math
import numpy as np
import rospy
from gazebo_msgs.srv import GetLinkState
from scipy.spatial.transform import Rotation
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def relative_rotation(source_quaternion, target_quaternion):
    rotation_source = Rotation.from_quat(source_quaternion)
    rotation_target = Rotation.from_quat(target_quaternion)
    relative_rotation = rotation_target * rotation_source.inv()
    relative_euler_angles = relative_rotation.as_euler("xyz", degrees=False)
    return relative_euler_angles

def call_getlinkstate(link_name, reference_frame):
    try:
        client = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        response = client(link_name, reference_frame)
        q = response.link_state.pose.orientation
        return (q.x, q.y, q.z, q.w)
    except rospy.ServiceException as e:
        rospy.logwarn(e)
        return (0.0, 0.0, 0.0, 1.0)

def bot_position_integrator(coord_x, coord_y, coord_z, delta_dist, bot_orien):
    coord_x += delta_dist * (math.cos(bot_orien[0]) * math.cos(bot_orien[1]))
    coord_y += delta_dist * (math.cos(bot_orien[0]) * math.sin(bot_orien[1]))
    coord_z += delta_dist * (math.sin(bot_orien[0]))
    return coord_x, coord_y, coord_z

def main():
    rospy.init_node("mapping")
    rospy.wait_for_service("/gazebo/get_link_state")

    pub = rospy.Publisher("point_cloud", PointCloud2, queue_size=10)

    current_orien_left = call_getlinkstate("middle_left_wheel", "base_link")
    current_orien_right = call_getlinkstate("middle_right_wheel", "base_link")
    current_bot_orien = call_getlinkstate("base_link", "world")

    net_bot_orien = np.array([0.0, 0.0, 0.0])
    x_coord = y_coord = z_coord = 0.0
    bot_radius = 2.0
    cloud_points = []
    prev_orien_left = current_orien_left
    prev_orien_right = current_orien_right
    prev_bot_orien = current_bot_orien

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        current_orien_left = call_getlinkstate("middle_left_wheel", "base_link")
        current_orien_right = call_getlinkstate("middle_right_wheel", "base_link")
        current_bot_orien = call_getlinkstate("base_link", "world")

        delta_angles_left = relative_rotation(prev_orien_left, current_orien_left)
        delta_angles_right = relative_rotation(prev_orien_right, current_orien_right)
        delta_bot_in_euler = relative_rotation(prev_bot_orien, current_bot_orien)

        delta_dl = bot_radius * delta_angles_left[1]
        delta_dr = bot_radius * delta_angles_right[1]
        delta_dist = (delta_dl + delta_dr) / 2.0

        net_bot_orien += delta_bot_in_euler

        x_coord, y_coord, z_coord = bot_position_integrator(
            x_coord, y_coord, z_coord, delta_dist, net_bot_orien
        )
        cloud_points.append([x_coord, y_coord, z_coord])

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"

        if len(cloud_points) >= 100:
            cloud_msg = pc2.create_cloud_xyz32(header, cloud_points)
            pub.publish(cloud_msg)

        prev_orien_left = current_orien_left
        prev_orien_right = current_orien_right
        prev_bot_orien = current_bot_orien

        rate.sleep()




if __name__ == "__main__":
    main()
