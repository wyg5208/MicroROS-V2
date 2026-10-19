"""Probe /imu: is the gyro live, and does its yaw rate match what /odom_raw claims?

The tour map smeared into concentric arcs, which is the signature of folding a scan at a
pose whose yaw is wrong. If the board's gyro reports a sane rate it can carry the heading
instead of the wheel odometry.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class Probe(Node):
    def __init__(self):
        super().__init__("imu_probe")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.imu_n = 0
        self.odom_n = 0
        self.wz = []
        self.q = None
        self.yaw_odom = None
        self.create_subscription(Imu, "/imu", self._imu, qos)
        self.create_subscription(Odometry, "/odom_raw", self._odom, qos)

    def _imu(self, m):
        self.imu_n += 1
        self.wz.append(m.angular_velocity.z)
        self.q = (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)

    def _odom(self, m):
        self.odom_n += 1
        o = m.pose.pose.orientation
        self.yaw_odom = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                                   1.0 - 2.0 * (o.y * o.y + o.z * o.z))


rclpy.init()
p = Probe()
t0 = time.monotonic()
last = t0
while time.monotonic() - t0 < 12.0 and rclpy.ok():
    rclpy.spin_once(p, timeout_sec=0.1)
    if time.monotonic() - last > 2.0:
        last = time.monotonic()
        wz = p.wz[-40:]
        print("imu=%d odom=%d  gyro_wz mean=%+.4f rad/s  min=%+.4f max=%+.4f  "
              "odom_yaw=%s  quat=%s"
              % (p.imu_n, p.odom_n,
                 sum(wz) / len(wz) if wz else float("nan"),
                 min(wz) if wz else float("nan"), max(wz) if wz else float("nan"),
                 "none" if p.yaw_odom is None else "%+.1f deg" % math.degrees(p.yaw_odom),
                 "none" if p.q is None else str(tuple(round(v, 3) for v in p.q))))
