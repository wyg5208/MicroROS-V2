# -*- coding: utf-8 -*-
"""弧线（Move direction=arc_left / arc_right）的离线判据。全程零动作：不连车、不连 :8000、不连大模型。

跑法（要 numpy，所以用 `_rosrun.bat` 那个解释器，不是 C:/Python314）：
    C:/Users/Leo/AppData/Local/Programs/Python/Python310/python.exe -X utf8 _arc_test.py
    或 C:\\microros-agent\\pc_side\\_rosrun.bat _arc_test.py

四段，各钉一件事：
A `nav.arc_band` —— 车画得出的半径带，两端由「底盘真给的速度」决定，不是命令值；
B `nav.arc_plan` —— 反馈的方向不许反、命令不许落进轮子不转的带宽、不许越过 MAX_W；
C `agent.clamp_motion` —— 一条弧同时受距离闸和转角闸管（radius 是几何不是运动量，压不住它就压弧长）；
D `Car.arc` 真闭环 —— 喂一个按 nav.achieved() 积分的合成底盘，看它收在命令半径上；再把底盘改成
  「完全不听 angular.z」（这台车历史上真这样过），看它当场认错而不是报「完成 0.40 米弧线」。

D 段的底盘只按线上下发的 (v, w) 走路，所以控制律写反会表现成「路径不对」，不会表现成「文字对不上」。
"""
import math
import os
import sys
import threading
import types
import time
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _stub_ros():
    """没有 ROS 环境也跑得起来：这份判据只要 Car.arc 的数学，一个节点都不建、一个话题都不发。

    桥本体在模块层只用 `class Car(Node)`，publisher/QoS 都在 `Car.__init__` 里，而这个桩永远不会
    被实例化 —— 所以桩只需要提供"能 subclass / 能 import"的名字，不需要任何行为。
    """
    mods = {"rclpy": ["ok", "init", "shutdown", "spin"],
            "rclpy.node": ["Node"],
            "rclpy.qos": ["HistoryPolicy", "QoSProfile", "ReliabilityPolicy"],
            "geometry_msgs.msg": ["Twist", "Vector3"],
            "nav_msgs.msg": ["Odometry"],
            "sensor_msgs.msg": ["Imu", "LaserScan"],
            "std_msgs.msg": ["Float32", "UInt16"]}
    for name, attrs in mods.items():
        if "." in name:
            pkg = name.split(".", 1)[0]
            if pkg not in sys.modules:
                head = types.ModuleType(pkg)
                head.__path__ = []
                sys.modules[pkg] = head
        mod = sys.modules.get(name) or types.ModuleType(name)
        for a in attrs:
            if not hasattr(mod, a):
                setattr(mod, a, type(a, (), {"__init__": lambda self, *k, **kw: None}))
        sys.modules[name] = mod


try:
    import rclpy  # noqa: F401
except ImportError:
    _stub_ros()

import car_mcp_server as m       # noqa: E402
import nav                        # noqa: E402
import agent                      # noqa: E402

fails = []


def ck(label, got, want):
    ok = got == want
    print(("OK  " if ok else "FAIL ") + label)
    if not ok:
        print("      got=%r want=%r" % (got, want))
        fails.append(label)


def close(label, got, want, tol):
    ok = abs(got - want) <= tol
    print(("OK  " if ok else "FAIL ") + label)
    if not ok:
        print("      got=%r want=%r +/-%s" % (got, want, tol))
        fails.append(label)


V = 0.25                          # 和 --linear-speed 默认值一致
RMIN, RMAX = nav.arc_band(V)

print("\n== A. 能画出来的半径带 ==")
print("   命令 %.2f m/s 时：%.2f ~ %.2f m" % (V, RMIN, RMAX))
ck("A0 带是正的、且窄端小于缓端（弧长/转角换算的前提）", RMIN > 0.0 and RMIN < RMAX, True)
head = nav.ACT_GAIN_W * nav.MAX_W - nav.ACT_W_BIAS
close("A1 窄端 = v_act/(gain·MAX_W − 静摩擦)：这是「底盘给得出」的极限，不是命令的极限",
      RMIN, nav.ACT_GAIN_V * V / head, 1e-9)
slope = nav.ACT_GAIN_W * nav.turn_floor() - nav.ACT_W_BIAS
close("A2 缓端 = 要求 w_cmd 恰好等于 turn_floor 的那个半径（再缓就落到轮子不转的带宽里）",
      RMAX, nav.ACT_GAIN_V * V / slope, 1e-9)
ck("A3 比缓端还缓的半径确实画不出来（命令 1.67 m 实际走成直线：开环 w=v/R 的坑就在这）",
   RMAX < 1.67, True)
b2 = nav.arc_band(2 * V)
ck("A4 带随命令速度线性放大（半径 = v/w，两个轴同倍）",
   [round(b2[0] / RMIN, 3), round(b2[1] / RMAX, 3)], [2.0, 2.0])
old_volts = nav.ACT_VOLTS
nav.ACT_VOLTS = 9.0               # 掉压：v 和 w 一起缩，圆还是圆
ck("A5 电压掉到 9 V 半径带不变（别以为低压会让同一条命令更弯；静摩擦不随电压缩）",
   nav.arc_band(V) == (RMIN, RMAX), True)
nav.ACT_VOLTS = old_volts
old_bias = nav.ACT_W_BIAS
nav.ACT_W_BIAS = 5.0              # 静摩擦大到达不到起转点：斜率变负
ck("A6 斜率非正时缓端退化成 inf（宁可放行让闭环自己判，也不给出一个负的可用带）",
   nav.arc_band(V)[1] == float("inf"), True)
nav.ACT_W_BIAS = old_bias

print("\n== B. 一拍该下发什么（反馈方向 / 死区 / 上限）==")
w0, k0, straight0 = nav.arc_plan(0.5, 0.0, V, 1.0)
ck("B0 误差为零时不多塞一把弯：曲率就是 1/R", round(k0, 6), round(1.0 / 0.5, 6))
ck("B1 这一拍不是「直走」", straight0, False)
va, wa = nav.achieved(V, w0)
close("B2 命令是按「反解静摩擦」给的：底盘真给的转角速度 = v_act/R（开环 w=v/R 做不到这条）",
      wa, va / 0.5, 1e-9)
ck("B3 左转（side=+1）下发正 angular.z", w0 > 0.0, True)
wR, _kR, _sR = nav.arc_plan(0.5, 0.0, V, -1.0)
ck("B4 右转同半径同速度：只有符号相反，幅度一致（左右拟合常数当前相等）",
   round(abs(wR), 9), round(w0, 9))
ws = [nav.arc_plan(0.5, e, V, 1.0)[0] for e in (-0.2, 0.0, 0.2, 0.6)]
ck("B5 半径误差 e=|P−C|−R 越正（车越往外包）就多弯：反馈方向不许写反",
   all(ws[i] <= ws[i + 1] + 1e-12 for i in range(len(ws) - 1)), True)
ck("B6 e 为负（已经弯过头、在圈内侧）时曲率指令往回收", ws[0] < ws[1], True)
w_neg, k_neg, st_neg = nav.arc_plan(0.5, -2.0, V, 1.0)
ck("B7 收回到 κ≤0 就当「这一拍直走」，不给负曲率（负曲率是往回绕，不是这条弧）",
   [w_neg, k_neg, st_neg], [0.0, 0.0, True])
w_tight, _k, _s = nav.arc_plan(RMIN * 0.4, 0.5, V, 1.0)
ck("B8 再急也不越过 MAX_W（越过就是命令一个底盘给不出的速率，误差只会涨）",
   w_tight <= nav.MAX_W + 1e-12, True)
w_soft, _k2, st_soft = nav.arc_plan(RMAX * 1.5, 0.0, V, 1.0)
ck("B9 缓端之外：需要的命令落进轮子不转的带宽 → 这一拍当直走，不硬给一个转",
   [w_soft, st_soft], [0.0, True])
ck("B10 radius≤0 或速度为 0 时不炸，直接给「直走」",
   [nav.arc_plan(0.0, 0.0, V, 1.0), nav.arc_plan(0.5, 0.0, 0.0, 1.0)],
   [(0.0, 0.0, True), (0.0, 0.0, True)])

# 收尾速度那道地板：弧线不像直线可以随便减速——速度慢到某个值，这一拍需要的转角就掉进
# 轮子不转的带宽，B9 那条会正确地判「直走」，于是每条弧的最后几厘米都是直线。
floor_at_rmax = nav.arc_min_speed(RMAX)
close("B11 地板和半径带是同一个不等式的两面：arc_min_speed(缓端半径) 就是算带子用的那个速度",
      floor_at_rmax, V, 1e-12)
ck("B12 半径越缓（越大）要的速度越高，且 radius=0 不炸",
   [nav.arc_min_speed(1.0) > nav.arc_min_speed(0.5), nav.arc_min_speed(0.0)], [True, 0.0])
for rr in (RMIN, 0.5, RMAX):
    lo = nav.arc_min_speed(rr) * 1.001        # 从上方贴住地板：正好相等时落在 `arc_plan` 的拒发线上
    w_at, _k, st_at = nav.arc_plan(rr, 0.0, lo, 1.0)
    close("B13 R=%.2f m：贴到地板时反解出的转角正好落在起转点上（地板就是那条线的倒影）" % rr,
          w_at, nav.turn_floor(), 1e-3)
    ck("B14 R=%.2f m：地板之上这一拍弯得动（不是直走）" % rr, st_at, False)
    ck("B15 R=%.2f m：少给一成就不弯了（不是随手留的余量）" % rr,
       nav.arc_plan(rr, 0.0, nav.arc_min_speed(rr) * 0.9, 1.0)[2], True)
old_k, nav.volt_scale = nav.volt_scale, lambda: 0.80
w_low, _kl, st_low = nav.arc_plan(0.5, 0.0, V, 1.0)
ck("B16 掉压不许改命令：圆的半径是 v 与 w 的比，`achieved()` 把两条轴一起缩",
   round(w_low, 9), round(w0, 9))
va, wa = nav.achieved(V, w_low)
close("B17 拿正着算的那个模型复核这条反解：低电量下真给出的 v/w 仍是命令半径 0.5 m",
      va / wa, 0.5, 1e-9)
ck("B18 且这一拍不是「直走」（电量不该把一条弧降级成直线）", st_low, False)
nav.volt_scale = old_k

print("\n== C. 逐轮那两把闸压不压得住弧线 ==")


def clamp(dist_cap, ang_cap, direction="arc_left", radius=0.5, distance=1.5):
    cfg = SimpleNamespace(max_distance=dist_cap, max_angle=ang_cap, allow_tools=None)
    args = {"direction": direction, "radius": radius, "distance": distance, "angle": 0.0}
    return agent.clamp_motion(cfg, "Move", args)


CAP_S = round(math.radians(45.0) * 0.5, 3)
a, note = clamp(1.5, 45.0)
ck("C1 转角闸 45° 换成弧长压住了它（0.5 m 半径 × 45° = %s m，不是原来的 1.5）" % CAP_S,
   round(a["distance"], 3), CAP_S)
ck("C2 压了就必须让模型看见，且说清是按扫角折的", ("扫角" in note and "0.5" in note), True)
ck("C3 radius 是几何量，不动它", a["radius"], 0.5)
a2, note2 = clamp(0.2, 45.0)
ck("C4 距离闸更小时由它先压到 0.2，扫角那条不许再缩一次（否则一条弧被两把闸重复夹）",
   [a2["distance"], "扫角" in note2], [0.2, False])
a3, note3 = clamp(1.5, 90.0, direction="forward", radius=0.5)
ck("C5 直线方向不受弧线那条影响（distance 仍按 1.5 米夹，也不许多嘴一句扫角）",
   [a3["distance"], "扫角" in note3], [1.5, False])
a4, _n4 = clamp(1.5, 45.0, direction="arc_right")
ck("C6 左右弧同样受压（符号写在 direction 里，不靠 distance 的正负）",
   round(a4["distance"], 3), CAP_S)
ck("C7 压完仍然几何自洽：弧长/半径 不会超过扫角上限",
   a["distance"] / a["radius"] <= math.radians(45.0) + 1e-9, True)
a5, note5 = clamp(0.0, 0.0)
ck("C8 两把闸都压到 0 = 这一班什么都别动（0 是个数，不是缺省）",
   [a5["distance"], a5["radius"], bool(note5)], [0.0, 0.5, True])
a6, _n6 = clamp(1.5, 45.0, radius=0)
ck("C9 radius 没给/给 0 时不许拿它换算（除零会折成 0 距离，等于悄悄禁掉这一句）",
   [a6["distance"], a6["radius"]], [1.5, 0])
a7, _n7 = clamp(1.5, 45.0, distance=-1.5)
ck("C10 负弧长同样夹、符号保留（对称性）", round(a7["distance"], 3), -CAP_S)
a8, _n8 = clamp(1.5, 45.0, distance="abc")
ck("C11 参数不像数时一个字不改（不猜、也不炸；截不断是 clamp 的老规矩）",
   [a8["distance"], a8["radius"]], ["abc", 0.5])


print("\n== D. 真跑 Car.arc（合成底盘，零动作）==")
m.CMD_PERIOD, m.STALL_SECS, m.STALL_GRACE = 0.01, 0.25, 0.0


class Alive(object):
    """循环里的 `and rclpy.ok()` 在没建节点的离线跑里是 False，和 _stall_test.py 同一处理。"""

    @staticmethod
    def ok():
        return True


m.rclpy = Alive


class Chassis(object):
    """一台只按线上下发的 (v, w) 走路的合成底盘。

    `obey=False` 是那台「直行巡航不听 angular.z」的固件（见记忆里那条点刹假象）：它照走直线，
    转角当耳边风。`wall` 是体坐标里钉着的一个东西，用来试「净空探测要往内侧挪」那条。
    脱困与受阻定性用桩替掉（_stall_test.py 同一手法）：那两招的账已由别的判据钉过。
    """

    # 从真桥身上借，不抄一份：判据要跑的就是那段闭环，抄了就不再是"测它"而是"测我的想象"。
    arc = m.Car.arc

    def __init__(self, obey=True, wall=None, stop_after=None):
        self.lock = threading.Lock()
        self.x = self.y = self.yaw = 0.0
        self.w_rate = 0.0
        self.t_prev = time.monotonic()
        self.pose = (0.0, 0.0, 0.0, self.t_prev)
        self.imu = (0.0, self.t_prev, (0.0, 0.0, 0.0, 1.0))
        self.args = SimpleNamespace(safe_distance=0.25, linear_speed=V, move_timeout=15.0)
        self.acting = False
        self.abort_reason, self.nav_stop, self.stop_src = "", False, ""
        self.last_error = ""
        self.cmds = []
        self.obey, self.wall, self.stop_after = obey, wall, stop_after

    def drive(self, v, w):
        self.cmds.append((v, w))
        if self.obey:
            va, wa = nav.achieved(v, w)
        else:
            va, wa = nav.achieved(v, 0.0)[0], 0.0
        # 合成世界的钟表跟着墙钟走。闭环那边积分陀螺用的是两个时间戳的真差，而 Windows 的
        # sleep 粒度比 CMD_PERIOD 大一倍以上：照 CMD_PERIOD 步进、照 wa 报速率，陀螺积出来的
        # 转角会比里程计多一倍，凭空点着一条「两处朝向不一致」——那是判据在测自己的时钟。
        now = time.monotonic()
        dt = max(1e-6, now - self.t_prev)
        self.t_prev = now
        self.yaw = m.wrap_pi(self.yaw + wa * dt)
        self.x += va * math.cos(self.yaw) * dt
        self.y += va * math.sin(self.yaw) * dt
        self.w_rate = wa
        self.pose = (self.x, self.y, self.yaw, now)
        self.imu = (self.w_rate, now,
                    (0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)))
        if self.stop_after is not None and len(self.cmds) >= self.stop_after:
            self.nav_stop = True
            self.stop_src = "human"

    def stop_stream(self, secs):
        pass

    def window_min(self, bearing, width):
        if self.wall is None:
            return None
        return self.wall[1] if abs(m.signed_deg(bearing - self.wall[0])) <= width else None

    def _why_stalled(self, kind, sign, gyro_rad, secs):
        return "blocked", ["测试桩：定性由判据给定"]

    def _escape(self, kind, sign, attempt, speed, label):
        return "假脱困（没动车）", 0.0


R, need = 0.5, 0.40
c1 = Chassis()
ok1, msg1 = c1.arc(R, need, 1.0, V)
print("   正常底盘：", msg1)
ck("D1 听得懂 angular.z 的底盘：这条弧完成", ok1, True)
ck("D2 交还了电机所有权（acting 不释放，控制台那条零速流会一直避让，等于把车挂住）",
   c1.acting, False)
re_r = float(msg1.split("实际半径约 ")[1].split(" m")[0]) if "实际半径约" in msg1 else -1.0
close("D3 报的「实际半径」收在命令半径上（反馈把静摩擦吃掉的 1.2~1.9 倍补回来了）", re_r, R, 0.10)
got_m = float(msg1.split("已完成弧线 ")[1].split("/")[0]) if "已完成弧线" in msg1 else -1.0
close("D4 走过的米数≈命令弧长", got_m, need, 0.06)
ck("D5 下发的转角一直是正号（左转中途没反过来甩）", all(w >= 0.0 for _v, w in c1.cmds), True)
sw = abs(m.wrap_pi(c1.pose[2]))
close("D6 扫过的角≈弧长/半径（0.40/0.5 = 45.8°）",
      math.degrees(sw), math.degrees(need / R), 4.0)
ck("D7 车确实在绕左侧那个圆心：位移的 y 分量为正", c1.pose[1] > 0.02, True)
ck("D7b 收尾没有直走：整程零拍落进轮子不转的带宽（`nav.arc_min_speed` 那道地板的存在理由），"
   "且回包里不许出现「边走边直走」",
   [any(w == 0.0 for _v, w in c1.cmds), "直走" in msg1], [False, False])

c2 = Chassis(obey=False)
ok2, msg2 = c2.arc(R, need, 1.0, V)
print("   不听 angular.z 的固件：", msg2)
ck("D8 底盘不认转角时这条不许报「完成」——它走的是一条直线", ok2, False)
ck("D9 失败文本认的是「车没照命令的半径弯」，不是含糊的超时", "车没照命令的半径弯" in msg2, True)
ck("D10 并给出实际半径（直行时这个数会大到离谱，就是它的数字样子）",
   float(msg2.split("实际半径约 ")[1].split(" m")[0]) > 50.0, True)
ck("D11 还告诉下一步（别拿去建图、别原样重发）",
   ("别原样重发" in msg2 and "别拿去建图" in msg2), True)

c3 = Chassis()
ok3, msg3 = c3.arc(3.0, 0.3, 1.0, V)
print("   半径太缓：", msg3)
ck("D12 带外半径当场拒", ok3, False)
ck("D13 拒的时候一个命令都没发（「当场」的意思就是车没动过）", len(c3.cmds), 0)
ck("D14 太缓 → 指人/模型改用 forward", "forward" in msg3, True)
c4 = Chassis()
ok4, msg4 = c4.arc(0.05, 0.3, 1.0, V)
ck("D15 半径太急也当场拒，并说清是转角上限",
   [ok4, "linear-speed" in msg4, len(c4.cmds)], [False, True, 0])
ck("D16 拒的文本里带着这一轮真能用的半径带（不是第二份写死的副本）",
   ("%.2f ~ %.2f" % (RMIN, RMAX)) in msg4, True)

wall = (60.0, 0.18)               # 体坐标左前方 0.18 m，低于 --safe-distance 0.25
c5 = Chassis(wall=wall)
ok5, msg5 = c5.arc(R, 0.35, 1.0, V)
print("   左前有墙、往左转：", msg5)
ck("D17 左转在左前方的墙上中止", [ok5, "内有障碍" in msg5], [False, True])
ck("D18 中止文本说的是「左前 30°」而不是「正前方」（弧线的下一步不在鼻子上）",
   "左前 30°" in msg5, True)
c6 = Chassis(wall=wall)
ok6, msg6 = c6.arc(R, 0.35, -1.0, V)
print("   同一面墙、往右绕：", msg6)
ck("D19 同一面墙，往右绕不该被它拦住（净空探测随弯向挪）", "内有障碍" in msg6, False)
ck("D20 且这一条是真完成的（不是「没撞上所以也没走」）",
   [ok6, "已完成弧线" in msg6], [True, True])

c7 = Chassis(stop_after=12)
ok7, msg7 = c7.arc(R, 0.6, 1.0, V)
ck("D21 行进中急停走的是人工那条道：文本是「收到停止指令」，不是机器判定",
   [ok7, "收到停止指令" in msg7, "机器判定" in msg7], [False, True, False])
ck("D22 急停时同样交还电机", c7.acting, False)

car8 = Chassis()
mcp = m.build_server(car8, SimpleNamespace(linear_speed=V, angular_speed=1.0,
                                           move_timeout=15.0, safe_distance=0.25))
desc, schema, move = mcp.tools["Move"]
ck("D23 工具 schema 里真的多了 radius（模型那边看得见这个参数，不靠提示词口头约定）",
   schema["properties"].get("radius", {}).get("type"), "number")
ck("D24 工具描述里的半径带就是 nav.arc_band 算出来的那对（写死过一次：改了 --linear-speed 描述没跟着改）",
   ("%.2f to %.2f" % (RMIN, RMAX)) in desc, True)
r_no = move(direction="arc_left", distance=0.3)
ck("D25 漏给 radius 时是「教你怎么补」，不是抛异常（小模型第二次就会带上）",
   ["radius" in r_no, r_no.startswith("失败"), len(car8.cmds)], [True, True, 0])
r_bad = move(direction="spin", distance=0.3)
ck("D26 方向写错时的可选值列全六个（旧文本只列四个，模型照着它学不到弧线）",
   all(k in r_bad for k in ("forward", "backward", "turn_left", "turn_right",
                            "arc_left", "arc_right")), True)
r_zero = move(direction="arc_left", distance=0.0, radius=R)
ck("D27 弧长 0 是「没动」，既不当错误也不发命令", [r_zero.startswith("成功"), len(car8.cmds)],
   [True, 0])
r_arc = move(direction="arc_left", distance=0.3, radius=R)
ck("D28 真从 Move 走一遍：分派到弧线闭环、车动了、回包带「成功」",
   [r_arc.startswith("成功"), "已完成弧线" in r_arc, len(car8.cmds) > 5], [True, True, True])

print("\n%s" % ("ARC-TEST PASS" if not fails else "ARC-TEST FAIL：%s" % fails))
sys.exit(1 if fails else 0)
