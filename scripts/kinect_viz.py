#!/usr/bin/env python3

import time
import freenect
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  
#noqa:F401

KINECT_PARAMS={ 'fx'    : 525.0,
                'fy'    : 525.0,
                'cx'    : 319.5,
                'cy'    : 239.5,
                'width' : 640,
                'height': 480,}

class KinectPointCloudVisualizer:
    def __init__(self):
        self.downsample = 8
        self.max_depth  = 2000
        plt.ion()
        self.fig        = plt.figure(figsize=(12, 8))
        self.ax         = self.fig.add_subplot(111, projection="3d")

    def depth_to_pointcloud(self, depth_image):
        height,width = depth_image.shape
        w,h          = np.meshgrid(np.arange(0, width, self.downsample),
                                   np.arange(0, height, self.downsample),)
        w_flat       = w.flatten()
        h_flat       = h.flatten()
        depth_flat = depth_image[h_flat, w_flat]
        valid = (depth_flat > 0) & (depth_flat < self.max_depth)
        w_valid = w_flat[valid]
        h_valid = h_flat[valid]
        z_valid = depth_flat[valid].astype(np.float32)
        x = (w_valid - KINECT_PARAMS['cx']) * z_valid / KINECT_PARAMS['fx']
        y = (h_valid - KINECT_PARAMS['cy']) * z_valid / KINECT_PARAMS['fy']
        z = z_valid

        return np.column_stack([x, y, z])

    def update_visualization(self, points):
        self.ax.clear()
        if len(points) > 0:
            colors=points[:, 2]
            self.ax.scatter(
                points[:,0],
                points[:,1],
                points[:,2],
                c=colors,
                cmap="viridis",
                s=0.5,
                alpha=0.6,)
            self.ax.set_xlabel("X (mm)")
            self.ax.set_ylabel("Y (mm)")
            self.ax.set_zlabel("Z (mm)")
            self.ax.set_title(f"Kinect 3D Point Cloud - {len(points)} points")
            self.ax.set_xlim([-1000, 1000])
            self.ax.set_ylim([-1000, 1000])
            self.ax.set_zlim([300, 2000])
            self.ax.view_init(elev=60, azim=90)
        plt.draw()
        plt.pause(0.01)

    def run_realtime(self, duration=60):
        print(f"Starting real-time 3D visualization for {duration} secs..")
        print("Move around in front of the Kinect to see the 3D point cloud-")
        start_time = time.time()
        frame_count = 0

        try:
            while time.time() - start_time < duration:
                try:
                    depth, _ = freenect.sync_get_depth()
                    points = self.depth_to_pointcloud(depth)
                    self.update_visualization(points)

                    frame_count+=1

                    if not plt.get_fignums():
                        break

                except Exception as e:
                    print(f"Frame capture error: {e}")
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("\n\n Stopped by user")

        elapsed = time.time()-start_time
        fps = frame_count / elapsed if elapsed > 0 else 0
        print(f"\n Completed: {frame_count} frames in {elapsed:.1f} ({fps:.1f} FPS)")


def single_frame_demo():
    print("\n Single frame for 3D visual..")
    try:
        depth, _ = freenect.sync_get_depth()
        print(f"Frame: {depth.shape}, range {depth.min()}-{depth.max()}")

        viz= KinectPointCloudVisualizer()
        viz.downsample= 4
        points= viz.depth_to_pointcloud(depth)
        print(f"Generated {len(points)} 3D points")
        viz.update_visualization(points)

        print("Displayed.. close window to exit.")
        plt.show(block=True)

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    import sys
    print("\n 3D Point Cloud Visualizer")
    print("1. Single frame 3D point cloud")
    print("2. Real-time 3D visualization(60 secs)")
    if len(sys.argv) > 1:
        mode = sys.argv[1].strip()
    else:
        mode = input("Enter option(1-2): ").strip()
    if mode == "1":
        single_frame_demo()
    else:
        viz = KinectPointCloudVisualizer()
        viz.run_realtime(60)
