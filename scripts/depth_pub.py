#!/usr/bin/env python3

import rospy
import os
import time
import cv2
import struct
import numpy as np
import freenect
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header

W,H = 640, 480
FX=FY = 525.0
CX,CY = 319.5, 239.5
FRAME_ID = "kinect_optical_frame"

OFFLINE = os.environ.get("KINECT_OFFLINE", "1") == "1"


def synthetic_depth():
    u = np.arange(W)
    v = np.arange(H)
    U,V = np.meshgrid(u, v)

    plane = 1000 + 0.8*(U-320) + 0.6*(V-240)
    r2 = (U-360) ** 2 + (V-220) ** 2
    bump = 400 * np.exp(-r2 / (2* 90**2))
    depth = np.clip(plane + bump + 20 * np.random.randn(H, W), 500, 2000)
    return depth.astype(np.uint16)


def real_depth_mm(retries=10, delay=0.1):
    for _ in range(retries):
        frame = freenect.sync_get_depth(format=freenect.DEPTH_MM)
        if frame is not None and frame[0] is not None:
            depth_mm = frame[0].astype(np.uint16)
            depth_mm = np.where((depth_mm == 0) | (depth_mm > 4500), 0, depth_mm)
            try:
                depth_mm = cv2.medianBlur(depth_mm, 5)
            except Exception:
                pass
            return depth_mm
        time.sleep(delay)
    raise RuntimeError(
        "Kinect not accessible: check power, udev rules, kernel driver, or USB port."
    )


def make_camera_info():
    msg = CameraInfo()
    msg.width = W
    msg.height = H
    msg.header.frame_id = FRAME_ID
    msg.K = [FX, 0, CX, 0, FY, CY, 0, 0, 1]
    msg.P = [FX, 0, CX, 0, 0, FY, CY, 0, 0, 0, 1, 0]
    msg.distortion_model = "plumb_bob"
    msg.D = [0, 0, 0, 0, 0]
    return msg


def depth_to_cloud(depth_mm):
    mask = depth_mm > 0
    vs, us = np.where(mask)
    zs = depth_mm[vs, us].astype(np.float32) / 1000.0
    xs = (us - CX) * zs / FX
    ys = (vs - CY) * zs / FY
    points = np.vstack((xs, ys, zs)).T

    header = Header(stamp=rospy.Time.now(), frame_id=FRAME_ID)
    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    data = b"".join([struct.pack("fff", *p) for p in points])
    pc2 = PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=12,
        row_step=12 * points.shape[0],
        data=data,
    )
    return pc2


def main():
    rospy.init_node("kinect_depth_pub")
    img_pub = rospy.Publisher("/camera/depth/image_raw", Image, queue_size=1)
    info_pub = rospy.Publisher("/camera/depth/camera_info", CameraInfo, queue_size=1)
    cloud_pub = rospy.Publisher("/camera/depth/points", PointCloud2, queue_size=1)
    info_msg = make_camera_info()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if OFFLINE:
            depth = synthetic_depth()
        else:
            depth = real_depth_mm()

        img = Image()
        img.header.stamp = rospy.Time.now()
        img.header.frame_id = FRAME_ID
        img.height, img.width = depth.shape
        img.encoding = "16UC1"
        img.step = img.width * 2
        img.data = depth.tobytes()

        info_msg.header.stamp = img.header.stamp
        cloud = depth_to_cloud(depth)

        img_pub.publish(img)
        info_pub.publish(info_msg)
        cloud_pub.publish(cloud)

        rate.sleep()


if __name__ == "__main__":
    main()
