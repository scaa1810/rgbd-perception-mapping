# RGB-D Perception and Odometry-Based Mapping

ROS package providing:
- Odometry-based 2D/3D trajectory mapping for a differential-drive robot
- RGB-D perception pipeline (Kinect → depth → 3D point cloud)
- Real-time obstacle monitoring with simple STOP/OK advisories



## Installation

cd ~/catkin_ws/src
git clone https://github.com/scaa1810/rgbd-perception-mapping.git
cd ~/catkin_ws && catkin_make
source devel/setup.bash
sudo apt install python3-freenect ros-noetic-rviz

## Usage

Full stack
roslaunch rgbd_nav_pkg rgbd_nav_full.launch

Monitor
rostopic echo /nav/advice
rosservice call /closest_obstacle "{}"

## Topics & Services

| Topic/Service            | Type                      | Description                    |
|--------------------------|---------------------------|--------------------------------|
| `/camera/depth/points`   | `sensor_msgs/PointCloud2` | Calibrated 3D point cloud      |
| `/camera/depth/image_raw`| `sensor_msgs/Image`       | Depth image (16UC1)            |
| `/camera/depth/camera_info`| `sensor_msgs/CameraInfo` | Kinect intrinsics              |
| `/nav/advice`            | `std_msgs/String`         | "STOP/OK" advisories @ 5Hz     |
| `/point_cloud`           | `sensor_msgs/PointCloud2` | Odometry trajectory            |
| `/closest_obstacle`      | `std_srvs/Trigger`        | Nearest obstacle distance      |


## Launch Architecture

Kinect → depth_pub → /camera/depth/points → obstacle_watch → /nav/advice
↓
Odometry → mapping_node → /point_cloud → RViz visualization

[Cam intrinsics -> depth : x=(u−cx)⋅z/fx  ,  y=(v−cy)⋅z/fy  ,  z=depth (mm)]


![Kinect 3D Point Cloud](images/kinect_viz_1.png)              ![Kinect 3D Point Cloud](images/kinect_viz_2.png)
![Real-time depth point cloud in RViz](images/rviz_viz_1.png)  ![Realtime depth point cloud in RViz](images/rviz_viz_2.png)
![Realtime mapping in sim](images/mapping.png)

## Requirements

- ROS Noetic (Ubuntu 20.04)
- Kinect v1 (`python3-freenect`)
- Differential drive with wheel/IMU odometry

## License

[MIT](LICENSE) © 2025 scaa1810
