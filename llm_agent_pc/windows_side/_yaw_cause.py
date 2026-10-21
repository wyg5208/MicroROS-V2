# -*- coding: utf-8 -*-
"""为什么 `Move forward`（我们每 tick 都发 w=0）之后 /odom_raw 的朝向跑了 30-40°？

三个来源互相独立，所以能分出"车真的转了"还是"位姿被编出来了"：

  A. /odom_raw 的 yaw        —— 底盘自己报告的朝向（建图折叠用的就是它）
  B. /imu 陀螺积分           —— 另一个物理量、另一条数据通路
  C. 雷达场景的旋转量        —— 把当前这一扫相对起步那一扫做圆周平移对齐，
                              最佳对齐的位移 = 场景相对车转了多少度

C 是关键：它不经过里程计也不经过陀螺，轮子打滑、陀螺被电机振动偏置都骗不了它。
但 C 自己也得被证明看得见旋转，所以最后加一段**原地正转**当阳性对照：那一段里
A、B、C 必须一起涨；否则说明估计器是坏的，前面那句"没转"就不算证据。

判读：
  A 涨、C 不涨   -> 里程计编造了朝向（图是折在假位姿上糊的，不是车歪着走的）
  A 涨、C 一起涨 -> 车真的物理偏航（查轮子 / 机械 / 一侧打滑）
  B 与 A 不一致  -> 顺带说明 odom 的 yaw 有没有用 IMU

建图请先关（/api/mapping false）—— 这段实验的位姿本身就是被测对象，别让它投票进图。
跑法：python -X utf8 _yaw_cause.py drives [速率] [米数] [w]      （要 rclpy，用 `_rosrun.bat`）
"""
import json
import math
import statistics
import sys
import time
import urllib.request

from drives_gate import require_drives  # noqa: E402

require_drives("自己发 cmd_vel（第二个发布者，绕开 Move 的闭环）跑一段，末尾还要原地正转做阳性对照")

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan

BEST = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                  reliability=ReliabilityPolicy.BEST_EFFORT)
CONSOLE = "http://127.0.0.1:8010"
MODE, V, GO, TURN_W, N = "drive", 0.10, 3.0, 0.30, 3
REC = 0.0
if sys.argv[1:2] == ["rec"]:
    # 记录模式：`_yaw_cause.py rec <秒>`，电机由别人（真 Move）驱动
    MODE, REC = "rec", (float(sys.argv[2]) if len(sys.argv) > 2 else 14.0)
    V = GO = TURN_W = 0.0
elif sys.argv[1:2] == ["shuttle"]:
    # 梭动模式：前进 0.25 m、倒车 0.25 m 来回若干趟，净位移≈0
    # 平移几乎归零，雷达 C 的对齐就不会被走位洗掉——此时 C 报的度数只能来自真旋转
    MODE = "shuttle"
    N = int(sys.argv[2]) if len(sys.argv) > 2 else 3
else:
    V = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
    GO = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    TURN_W = float(sys.argv[3]) if len(sys.argv) > 3 else 0.30
TURN_T = 1.75
NOTURN = "noturn" in sys.argv[4:]


def get(path):
    return json.loads(urllib.request.urlopen(CONSOLE + path, timeout=3).read())


def post(path, obj):
    req = urllib.request.Request(CONSOLE + path, data=json.dumps(obj).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=2).read()
    except Exception:                                      # noqa: BLE001
        pass


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Rec(Node):
    def __init__(self):
        super().__init__("yaw_cause")
        self.od = []          # (t, yaw, x, y, odom_twist_wz)
        self.imu = []         # (t, gyro_wz, quat_yaw)
        self.cmd = []         # (t, v, w)
        self.acc = {}         # 自上次 take_profile 以来的 1° 分箱
        self.scans = 0
        self.create_subscription(Odometry, "/odom_raw", self._o, BEST)
        self.create_subscription(Imu, "/imu", self._i, BEST)
        self.create_subscription(Twist, "/cmd_vel", self._c, BEST)
        self.create_subscription(LaserScan, "/scan", self._s, BEST)

    def _o(self, m):
        p = m.pose.pose
        self.od.append((time.monotonic(), yaw_of(p.orientation), p.position.x,
                        p.position.y, m.twist.twist.angular.z))

    def _i(self, m):
        self.imu.append((time.monotonic(), m.angular_velocity.z, yaw_of(m.orientation)))

    def _c(self, m):
        self.cmd.append((time.monotonic(), m.linear.x, m.angular.z))

    def _s(self, m):
        self.scans += 1
        amin, ainc = math.degrees(m.angle_min), math.degrees(m.angle_increment)
        for i, r in enumerate(m.ranges):
            if not math.isfinite(r) or r < 0.05 or r > 12.0:
                continue
            d = int(round((amin + i * ainc) % 360.0)) % 360
            self.acc.setdefault(d, []).append(r)

    def take_profile(self, keep=False):
        """1° 分箱的中位数轮廓。默认取完就清空——累积下去会把旋转洗平。"""
        if not self.acc:
            return None
        prof = [statistics.median(self.acc[d]) if d in self.acc else float("nan")
                for d in range(360)]
        if not keep:
            self.acc = {}
        return prof

    def lag(self, ref, cur, span=45):
        """cur 圆周平移多少度最像 ref。返回 (度, 对齐比例)。"""
        best = None
        for L in range(-span, span + 1):
            good = tot = 0
            for k in range(360):
                a, b = ref[k], cur[(k - L) % 360]
                if math.isnan(a) or math.isnan(b):
                    continue
                tot += 1
                if abs(a - b) < 0.30:
                    good += 1
            if tot < 150:
                continue
            sc = good / float(tot)
            if best is None or sc > best[1]:
                best = (L, sc)
        return best


def unwrap(rows, idx):
    out, prev, acc = [], None, 0.0
    for r in rows:
        y = r[idx]
        if prev is not None:
            acc += ((y - prev + math.pi) % (2 * math.pi)) - math.pi
        prev = y
        out.append((r[0], acc))
    return out


def fmt(lg):
    return (" %+6.1f° 齐%3.0f%%" % (lg[0], 100 * lg[1])) if lg else "   --  "


def main():
    rclpy.init()
    rec = Rec()
    st = get("/api/state")
    print("位姿 %s  建图=%s  busy=%r  电压 %.2f V" %
          (tuple(round(v, 2) for v in st["pose"]), st["mapping"], st["busy"], st["volts"]))
    if MODE == "rec":
        # 不碰电机：只录 A/B/C，让另一头的真 `Move` 当被试对象
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end and rclpy.ok():
            rclpy.spin_once(rec, timeout_sec=0.05)
        prof0 = rec.take_profile(keep=True)
        t_start = time.monotonic()
        deadline = t_start + REC
        last = 0.0
        while time.monotonic() < deadline and rclpy.ok():
            for _ in range(3):
                rclpy.spin_once(rec, timeout_sec=0.02)
            now = time.monotonic()
            if now - last > 0.25:
                last = now
                cur = rec.take_profile()
                lg = rec.lag(prof0, cur) if (cur and prof0) else None
                dw = unwrap(rec.od, 1)
                dq = unwrap(rec.imu, 2)
                dgy = sum(b[1] * (b[0] - a[0]) for a, b in zip(rec.imu, rec.imu[1:]))
                busy = ""
                volt = float("nan")
                try:
                    s = get("/api/state")
                    busy, volt = s["busy"], s["volts"]
                except Exception:                              # noqa: BLE001
                    pass
                print("t=%5.2f busy=%-8s A odom=%+7.2f°  B ∫gyro=%+7.2f°  B2 imu=%+7.2f°  "
                      "C 雷达=%s  odom_wz=%+.3f cmd=%s x=(%.2f,%.2f) 电压=%.2f" % (
                          now - t_start, repr(busy),
                          math.degrees(dw[-1][1]) if len(dw) > 1 else 0.0,
                          math.degrees(dgy),
                          math.degrees(dq[-1][1]) if len(dq) > 1 else 0.0,
                          fmt(lg),
                          rec.od[-1][4] if rec.od else 0.0,
                          ("(%.2f,%.2f)" % (rec.cmd[-1][1], rec.cmd[-1][2])) if rec.cmd else "-",
                          rec.od[-1][2] if rec.od else 0.0, rec.od[-1][3] if rec.od else 0.0, volt))
        print("\n帧数 odom=%d imu=%d scan=%d cmd=%d" %
              (len(rec.od), len(rec.imu), rec.scans, len(rec.cmd)))
        return
    if V == 0 and GO == 0:
        # 自检：车不动时 C 必须读 ~0° 且对齐率高，否则它"没转"的结论不值钱
        t_end = time.monotonic() + 4.0
        while time.monotonic() < t_end and rclpy.ok():
            rclpy.spin_once(rec, timeout_sec=0.05)
        ref = rec.take_profile(keep=True)
        for _ in range(8):
            t_end = time.monotonic() + 0.5
            while time.monotonic() < t_end and rclpy.ok():
                rclpy.spin_once(rec, timeout_sec=0.05)
            cur = rec.take_profile()
            if cur and ref:
                com = [(a, b) for a, b in zip(ref, cur)
                       if not math.isnan(a) and not math.isnan(b)]
                d = sorted(abs(a - b) for a, b in com)
                print("静止自检：分箱 ref=%d cur=%d 共同=%d  |Δ| 中位=%.3f p90=%.3f  "
                      "C 雷达 %s  前 6 对：%s" % (
                          sum(1 for v in ref if not math.isnan(v)),
                          sum(1 for v in cur if not math.isnan(v)), len(com),
                          d[len(d) // 2] if d else -1, d[int(len(d) * 0.9)] if d else -1,
                          fmt(rec.lag(ref, cur)),
                          [(round(a, 2), round(b, 2)) for a, b in com[:6]]))
            else:
                print("静止自检：轮廓为空 ref=%s cur=%s" % (ref is not None, cur is not None))
        return
    if st["busy"]:
        print("控制台正忙（%s），取消" % st["busy"])
        return
    if st["mapping"]:
        print("建图还开着：先 POST /api/mapping false 再来，否则这段假位姿会被折进图")
        return

    t_end = time.monotonic() + 2.5
    while time.monotonic() < t_end and rclpy.ok():
        rclpy.spin_once(rec, timeout_sec=0.05)
    pr = get("/api/scan")["pairs"]

    def win(cen):
        return [r for a, r in pr if abs(((a - cen + 180) % 360) - 180) <= 40]

    # 正前方 ±12° 是雷达自己的盲区（BLIND_WEDGE），用出厂 `Move` 的同一窗口判净空
    if MODE == "shuttle":
        legs = [("FWD", 1.2, 0.25, 0.0), ("HOLD", 1.1, 0.0, 0.0),
                ("REV", 1.2, -0.25, 0.0), ("HOLD", 1.1, 0.0, 0.0)]
        plan = 0.25 * 1.2 * 0.82
        f, b = win(0.0), win(180.0)
        if not f or not b or min(f) < plan + 0.30 or min(b) < plan + 0.30:
            print("前 %.2f / 后 %.2f m，来回一趟要 %.2f m 还各留 0.30 m，取消" % (
                min(f) if f else 99.0, min(b) if b else 99.0, plan))
            return
        segs = [("REST", 2.0, 0.0, 0.0)]
        for i in range(N):
            segs += [("%s%d" % (nm[0], i), du, sv, sw) for nm, du, sv, sw in legs]
    else:
        cen = 0.0 if V >= 0 else 180.0
        lab = "前方" if V >= 0 else "后方"
        front = win(cen)
        plan = abs(V) * GO * 0.82
        if not front or min(front) < plan + 0.30:
            print("%s 40° 内最近 %s m，走不完计划中的 %.2f m（要留 0.30 m 余量），取消" % (
                lab, "%.2f" % min(front) if front else "无回波", plan))
            return
        segs = [("REST", 2.0, 0.0, 0.0), ("GO", GO, V, 0.0), ("BRK", 2.0, 0.0, 0.0)]
        if not NOTURN:            # 阳性对照：证明 C 看得见真旋转（本会话已验过一次）
            segs.append(("TURN", TURN_T, 0.0, TURN_W))
    segs.append(("END", 2.0, 0.0, 0.0))
    prof0 = rec.take_profile(keep=True)      # 参考轮廓：起步前这几扫
    print("净空 前%.2f/后%.2f m，每段计划 %.2f m；参考轮廓 %d 个有效分箱（%d 扫）" % (
        min(win(0.0)) if win(0.0) else 99.0, min(win(180.0)) if win(180.0) else 99.0,
        plan, 360 - str(prof0).count("nan"), rec.scans))

    t = time.monotonic()
    marks, cursor = [], t
    for name, dur, sv, sw in segs:
        marks.append((name, cursor, cursor + dur, sv, sw))
        cursor += dur
    deadline = cursor
    t_start = t
    last = 0.0

    while time.monotonic() < deadline and rclpy.ok():
        now = time.monotonic()
        name, sv, sw = marks[0][0], marks[0][3], marks[0][4]
        for m in marks:
            if m[1] <= now:
                name, sv, sw = m[0], m[3], m[4]
        post("/api/teleop", {"v": sv, "w": sw, "hold": 0.4})
        for _ in range(3):
            rclpy.spin_once(rec, timeout_sec=0.02)
        if now - last > 0.25:
            last = now
            cur = rec.take_profile()
            lg = rec.lag(prof0, cur) if (cur and prof0) else None
            dw = unwrap(rec.od, 1)
            dq = unwrap(rec.imu, 2)
            dgy = sum(b[1] * (b[0] - a[0]) for a, b in zip(rec.imu, rec.imu[1:]))
            # 插着电源线时电压不会随负载下坠，靠电池才会——用它自证试验条件干净
            try:
                volt = get("/api/state")["volts"]
            except Exception:                                       # noqa: BLE001
                volt = float("nan")
            print("%-4s t=%4.2f  A odom=%+7.2f°  B ∫gyro=%+7.2f°  B2 imu=%+7.2f°  C 雷达=%s"
                  "  odom_wz=%+.3f cmd=%s x=(%.2f,%.2f) 电压=%.2f" % (
                      name, now - t_start,
                      math.degrees(dw[-1][1]) if len(dw) > 1 else 0.0,
                      math.degrees(dgy),
                      math.degrees(dq[-1][1]) if len(dq) > 1 else 0.0,
                      fmt(lg),
                      rec.od[-1][4] if rec.od else 0.0,
                      ("(%.2f,%.2f)" % (rec.cmd[-1][1], rec.cmd[-1][2])) if rec.cmd else "-",
                      rec.od[-1][2] if rec.od else 0.0, rec.od[-1][3] if rec.od else 0.0, volt))

    post("/api/teleop", {"v": 0.0, "w": 0.0, "hold": 0.6})
    for _ in range(40):
        rclpy.spin_once(rec, timeout_sec=0.05)

    print("\n---- 分段合计（段首到段尾）----")
    dw = unwrap(rec.od, 1)
    dq = unwrap(rec.imu, 2)
    for name, a, b, _sv, _sw in marks:
        so = [(x, y) for x, y in dw if a <= x < b]
        sq = [(x, y) for x, y in dq if a <= x < b]
        si = [r for r in rec.imu if a <= r[0] < b]
        dgy = sum(r[1] * (s[0] - r[0]) for r, s in zip(si, si[1:])) if len(si) > 1 else 0.0
        sp = [o for o in rec.od if a <= o[0] < b]
        shift = math.hypot(sp[-1][2] - sp[0][2], sp[-1][3] - sp[0][3]) if len(sp) > 1 else 0.0
        print("%-4s  A odom %+7.2f°   B ∫gyro %+7.2f°   B2 imu %+7.2f°   平移 %.2f m   "
              "样本 o=%d i=%d" % (
                  name,
                  math.degrees(so[-1][1] - so[0][1]) if len(so) > 1 else 0.0,
                  math.degrees(dgy),
                  math.degrees(sq[-1][1] - sq[0][1]) if len(sq) > 1 else 0.0,
                  shift, len(so), len(si)))
    print("\n帧数 odom=%d imu=%d scan=%d cmd=%d" %
          (len(rec.od), len(rec.imu), rec.scans, len(rec.cmd)))
    st = get("/api/state")
    print("结束位姿 %s  吻合=%s  冻结=%s" % (
        tuple(round(v, 2) for v in st["pose"]),
        round(st["reloc"]["agree"], 3) if st["reloc"]["agree"] is not None else None,
        st["reloc"]["frozen"]))


main()
