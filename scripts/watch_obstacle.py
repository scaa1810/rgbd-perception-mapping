#!/usr/bin/env python3

import struct
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

THRESH_STOP = 0.8  #meters | can change to whatever distance calibrated

def iter_points(msg):
    step = msg.point_step
    for i in range(0, len(msg.data), step):
        x, y, z = struct.unpack_from("fff", msg.data, i)
        yield x, y, z

class Watcher:
    def __init__(self):
        self.min_dist = float("inf")
        self.pub = rospy.Publisher("/nav/advice", String, queue_size=1)
        rospy.Subscriber("/camera/depth/points", PointCloud2, self.cb)
        self.srv = rospy.Service("/closest_obstacle", Trigger, self.handle_srv)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.tick)

    def cb(self, cloud):
        dmin = float("inf")
        for x, y, z in iter_points(cloud):
            if z > 0:
                d = np.sqrt(x * x + y * y + z * z)
                if d < dmin:
                    dmin = d
        self.min_dist = dmin

    def tick(self, _):
        d = self.min_dist
        if not np.isfinite(d):
            self.pub.publish("No data")
            return
        if d < THRESH_STOP:
            self.pub.publish("STOP: obstacle at %.2f m" % d)
        else:
            self.pub.publish("OK: clear (%.2f m)" % d)

    def handle_srv(self, _req):
        d = self.min_dist
        msg = "min_distance_m=%.3f" % (d if np.isfinite(d) else -1.0)
        return TriggerResponse(success=np.isfinite(d), message=msg)



if __name__ == "__main__":
    
    rospy.init_node("watch_obstacle")
    Watcher()
    rospy.spin()
