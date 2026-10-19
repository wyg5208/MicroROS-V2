# -*- coding: utf-8 -*-
"""Full-rate recorder for the turn-release question: what is the chassis still doing at the
instant we let go, and how far does it coast after that?

`_yaw_cause.py rec` prints about every 0.3 s, which is too coarse here -- 0.3 s at 0.79 rad/s is
13.6 deg of travel, more than the whole tolerance being studied. This one keeps every /cmd_vel and
/odom_raw message with its own timestamp and then splits the run into command segments, so each
turn reports the rate at release and the coast that followed.

Run it next to a driver that fires the turns:
    _rosrun.bat _turn_rec.py 45 _rec.csv

LIMITATION found on the first real run (2026-10-19): /cmd_vel does not arrive as a smooth 8 Hz
stream -- it comes in bursts of ~5 messages sharing one arrival millisecond, so an arrival-based
time axis quantises at ~0.34 s. At 0.84 rad/s that is 16 deg of travel per quantum, which is
coarser than the 4-10 deg coast this script was written to measure. The yaw *sequence* is still
usable (order is preserved), but do not read "rate at release" off these timestamps as if it were
a 10 ms measurement -- the closed-loop's own release log would have to carry the number instead.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

BEST = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50,
                  reliability=ReliabilityPolicy.BEST_EFFORT)
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
CSV = sys.argv[2] if len(sys.argv) > 2 else "_turn_rec.csv"
CMD_EPS = 0.02          # rad/s or m/s below which the console's idle zero counts as "no command"
TAIL = 2.5              # s watched after the last command, for the coast to finish
IDLE_CLOSE = 0.20       # s of zeros that ends a turn: turns are only ~0.6 s apart, so a TAIL-sized
                        #   gap merged a whole 10-turn run into one 21 s "segment" on the first run


class Rec(Node):
    def __init__(self):
        super().__init__("turn_rec")
        self.rows = []
        self.t0 = time.monotonic()
        self.last_cmd_print = 0.0
        self.create_subscription(Twist, "/cmd_vel", self._c, BEST)
        self.create_subscription(Odometry, "/odom_raw", self._o, BEST)

    def _c(self, msg):
        self.rows.append((time.monotonic() - self.t0, "cmd",
                          msg.linear.x, msg.angular.z, float("nan")))

    def _o(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.rows.append((time.monotonic() - self.t0, "odom",
                          msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                          math.degrees(yaw)))


def unwrap(deg):
    out, prev, off = [], None, 0.0
    for d in deg:
        if prev is not None:
            while d - prev < -180.0:
                off += 360.0
                d += 360.0
            while d - prev > 180.0:
                off -= 360.0
                d -= 360.0
        prev = d
        out.append(d)
    return out


def analyse(rows):
    odom = [r for r in rows if r[1] == "odom" and r[4] == r[4]]
    yaws = unwrap([r[4] for r in odom])
    t_od = [r[0] for r in odom]

    def yaw_at(t):
        for i, tt in enumerate(t_od):
            if tt >= t:
                return yaws[i]
        return yaws[-1] if yaws else float("nan")

    def odom_at(t):
        for i, tt in enumerate(t_od):
            if tt >= t:
                return odom[i]
        return odom[-1] if odom else None

    segs, cur = [], None
    for r in rows:
        if r[1] != "cmd":
            continue
        active = abs(r[2]) > CMD_EPS or abs(r[3]) > CMD_EPS
        if active and cur is None:
            cur = [r[0], r[0], r[2], r[3]]
        elif active:
            cur[1], cur[2], cur[3] = r[0], r[2], r[3]
        elif cur is not None and r[0] - cur[1] > IDLE_CLOSE:
            segs.append(cur)
            cur = None
    if cur is not None:
        segs.append(cur)

    print("odom=%d cmd=%d 段=%d" % (len(odom), len([r for r in rows if r[1] == "cmd"]), len(segs)))
    print("%-4s %-6s %8s %8s %9s %9s %9s" %
          ("seg", "dir", "dur", "w@rel", "coast", "net", "watched"))
    for i, (t0, t1, v, w) in enumerate(segs, 1):
        # Watch only until the next turn starts: with a 0.6 s gap between commands a fixed 2.5 s
        # tail ran into the following turn and printed a 45 deg "net" for a 30 deg command.
        nxt = segs[i][0] if i < len(segs) else t_od[-1]
        tend = min(t1 + TAIL, nxt)
        y0 = yaw_at(t0)
        y_rel = yaw_at(t1)
        o_rel = odom_at(t1)
        y_end = yaw_at(tend)
        direction = ("fwd" if v > 0 and abs(w) < 0.02 else
                     "back" if v < 0 and abs(w) < 0.02 else
                     "left" if w > 0 else "right")
        print("%-4d %-6s %6.2fs %+8.3f %+8.2f %+8.2f %8.2fs" %
              (i, direction, t1 - t0, (o_rel[3] if o_rel else float("nan")),
               y_end - y_rel, y_end - y0, tend - t1))
    return segs


def main():
    rclpy.init()
    rec = Rec()
    end = time.monotonic() + SECS
    while time.monotonic() < end:
        rclpy.spin_once(rec, timeout_sec=0.01)
    rows = rec.rows
    with open(CSV, "w", encoding="utf-8", newline="") as fh:
        fh.write("t,src,v,w,yaw_deg\n")
        for r in rows:
            fh.write("%.3f,%s,%.4f,%.4f,%s\n" %
                     (r[0], r[1], r[2], r[3], "%.3f" % r[4] if r[4] == r[4] else ""))
    print("wrote %s (%d rows)" % (CSV, len(rows)))
    analyse(rows)
    rec.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
