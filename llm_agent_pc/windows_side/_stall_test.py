"""停滞判据与受阻定性的离线验收（不动车，不需要车在线）。

跑法：`_rosrun.bat _stall_test.py [_t36d_clean.txt _t35_clean.txt ...]`

三段：
1. `stalled()` 的真值表 -- 命令速度、末段爬行、被堵死、正常转向，各走一遍；
2. 用真车录下来的日志逐行回放判据，看正常动作里它到底会不会误报（这是上车前最想要的一条）；
3. `_why_stalled()` 的融合定性：只雷达看到 / 只地图看到 / 都没看到 / 陀螺在转，四种场景各给一个标签。
"""
import math
import os
import re
import sys
import threading
import time
from types import SimpleNamespace

import car_mcp_server as m
import nav

ROWS = re.compile(r"t=\s*([-\d.]+).*?cmd=\(([-\d.]+),([-\d.]+)\)\s+x=\(([-\d.]+),([-\d.]+)\)")
fails = []


def check(name, got, want):
    ok = got == want
    print("%-58s %s  (got %r, want %r)" % (name, "PASS" if ok else "FAIL", got, want))
    if not ok:
        fails.append(name)


# ---- 1. 真值表 ---------------------------------------------------------------
ACH = nav.achieved
print("== stalled() 真值表 ==")
v_full = ACH(0.25, 0.0)[0]                    # ~0.205 m/s，全速段应有的实测
v_crawl = ACH(0.06, 0.0)[0]                   # ~0.049 m/s，每条命令末尾故意给的爬行档
w_turn = ACH(0.0, 1.0)[1]                     # ~0.756 rad/s，转向全速
check("直行全速、被堵死（1.6 s 走了 0.00 m）",
      m.stalled(v_full, 0.0, 1.6, "linear"), True)
check("直行全速、真在走（0.2 m/s × 1.6 s）",
      m.stalled(v_full, v_full, 1.6, "linear"), False)
check("直行末段爬行 0.06 档、确实在动（不该误报）",
      m.stalled(v_crawl, v_crawl, 1.6, "linear"), False)
check("直行末段爬行、其实完全没动（该报）",
      m.stalled(v_crawl, 0.0, 1.6, "linear"), True)
check("直行中途只挪了 2 cm（低于绝对下限 3 cm，该报）",
      m.stalled(v_full, 0.02 / 1.6, 1.6, "linear"), True)
check("直行 3 cm 窗口边界（恰好在绝对下限上，不报）",
      m.stalled(v_full, m.STALL_MIN_M / 1.6, 1.6, "linear"), False)
check("转向全速、车身纹丝不动（该报）",
      m.stalled(w_turn, 0.0, 1.6, "angular"), True)
check("转向全速、正常在转（不该报）",
      m.stalled(w_turn, w_turn, 1.6, "angular"), False)
check("转向命令 0（MapSpin deg=0 的静止折叠，不该报）",
      m.stalled(0.0, 0.0, 1.6, "angular"), False)
check("窗口极短（0.1 s）时绝对下限要 3 cm，慢速段不误报",
      m.stalled(v_full, 0.02 / 0.1, 0.1, "linear"), False)
check("signed_deg 把 300° 折成 -60°（脱困转向要拧短的那一边）", m.signed_deg(300.0), -60.0)
check("signed_deg 不动 60°", m.signed_deg(60.0), 60.0)

# 收尾速度律：末段命令必须落在底盘真的会执行的区间里。老律子在 45° 命令上把车停在 42.96°
# （容差 1.8°、死区下沿 2.1°），命令 0.11 rad/s < ACT_W_DEAD 0.118 => 交付 0，误差再也收不下去，
# 白烧完 15 s 死线。这条用例就是把那个不动点钉住。
print("\n== 末段速度律：不许命令一个底盘不会执行的角度 ==")
for deg in (15.0, 30.0, 45.0, 90.0):
    need = math.radians(deg)
    tol = max(0.02, 0.04 * need)                 # 与 _closed_loop 的释放判据同一条
    cmd = m.tail_cmd("angular", 1.0, tol * 1.05, need)      # 刚落在释放线外的那一档
    old = min(1.0, max(0.06, 1.0 * min(1.0, tol * 1.05 / need * 2.5)))
    check("转 %.0f° 末档命令交付 %.2f rad/s（老律子给 %.3f -> 交付 %.2f）" % (
        deg, nav.achieved(0.0, cmd)[1], old, nav.achieved(0.0, old)[1]),
        nav.achieved(0.0, cmd)[1] > 0.0, True)
check("直行末档仍然用 0.06 m/s 的爬行档（没有线性死区，别改）",
      m.tail_cmd("linear", 0.25, 0.001, 0.3), 0.06)
check("全速段不受影响", m.tail_cmd("angular", 1.0, math.radians(40.0), math.radians(45.0)), 1.0)

# 末档命令与停滞判据必须互相兼容：下限若低于起转点，判据会把"车真的在爬"当成顶住不动；
# 若下限太高，顶住不动的车每窗都能爬到 25% 的额度（odom 撒谎时更糟）。
_floor = nav.turn_floor()
_want = abs(nav.achieved(0.0, _floor)[1]) * m.STALL_SECS
check("末档命令 %.2f rad/s 交付 %.2f°/s（高于起转点 %.2f）" % (
      _floor, math.degrees(abs(nav.achieved(0.0, _floor)[1])), nav.ACT_W_CUTON),
      abs(nav.achieved(0.0, _floor)[1]) > 0.0, True)
check("末档正常爬速不许判成停滞",
      m.stalled(_want, 0.6 * _want, m.STALL_SECS, "angular"), False)
check("末档命令下顶住不动仍要判成停滞",
      m.stalled(_want, 0.0, m.STALL_SECS, "angular"), True)

# ---- 2. 真车日志回放 ---------------------------------------------------------
print("\n== 真车录制回放（正常动作里会不会误报） ==")
files = sys.argv[1:] or [f for f in ("_t36d_clean.txt", "_t35_clean.txt") if os.path.exists(f)]
for fn in files:
    rows = []
    with open(fn, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            mm = ROWS.search(line)
            if mm:
                rows.append(tuple(float(g) for g in mm.groups()))
    if not rows:
        print("%s: 没解析到行，跳过" % fn)
        continue
    # Two replay-only artifacts the 20 Hz controller cannot produce, both of which read as
    # "commanded but not moving" here: the parked tail before the first command, and the settle
    # gap after the last one. The recorder prints whatever `/cmd_vel` it happened to see last, so
    # a window that straddles either edge must be re-based, not counted.
    real = loose = 0
    ref_t, ref_x, ref_y = rows[0][0], rows[0][3], rows[0][4]
    kind = "linear" if any(abs(r[1]) > 0.05 for r in rows) else "angular"
    for i, (t, vc, wc, x, y) in enumerate(rows):
        moving = max(abs(vc), abs(wc)) > 0.01
        idle_before = i == 0 or max(abs(rows[i - 1][1]), abs(rows[i - 1][2])) <= 0.01
        if not moving or idle_before:
            ref_t, ref_x, ref_y = t, x, y
            continue
        moved = math.hypot(x - ref_x, y - ref_y)
        step = m.STALL_MIN_M if kind == "linear" else m.STALL_MIN_W
        if moved > (step if kind == "linear" else math.degrees(step)):
            ref_t, ref_x, ref_y = t, x, y
            continue
        win = t - ref_t
        if win < m.STALL_SECS:
            continue
        # The recorder is a 3.5 Hz bystander, but as long as the commanded velocity stayed
        # non-zero the controller's window is the same one it would have run, so the sample rate
        # alone cannot make a trip fake.
        want = ACH(abs(vc), abs(wc))[0 if kind == "linear" else 1]
        got = moved / win
        if m.stalled(want, got, win, kind):
            if win > 0.4:
                loose += 1
            else:
                real += 1
            print("   t=%.1f s 命令 %.2f 实测 %.2f（%.1f s 没动）" % (t, want, got, win))
            ref_t, ref_x, ref_y = t, x, y
    print("%s: %d 行，kind=%s，判据触发 %d 次（连续样本 %d、跨过录制间隙 %d）"
          % (fn, len(rows), kind, real + loose, real, loose))

# ---- 3. 融合定性 -------------------------------------------------------------
print("\n== _why_stalled() 三路融合 ==")


def fake_car(pairs, world=None, pose=(0.0, 0.0, 0.0)):
    """A Car-shaped object with only the senses _why_stalled touches."""
    car = m.Car.__new__(m.Car)
    car.lock = threading.Lock()
    car.pose = (pose[0], pose[1], pose[2], time.monotonic())
    car.scan_pose = car.pose
    car.pairs = list(pairs)
    car.scan_t = time.monotonic()
    car.args = SimpleNamespace(safe_distance=0.25, linear_speed=0.25,
                               angular_speed=1.0, move_timeout=15.0)
    car.console = SimpleNamespace(world=world) if world is not None else None
    return car


def walled(ahead=None, folds=4, res=0.05):
    """A nav.World with one wall straight ahead at `ahead` metres (None = nothing drawn)."""
    w = nav.World(res=res)
    if ahead is None:
        return w
    pts = [(b, ahead) for b in range(-20, 21, 4)]
    for _ in range(folds):
        w.integrate((0.0, 0.0, 0.0), pts)
    return w


open_ring = [(a, 2.0) for a in (-60.0, -30.0, 30.0, 60.0, 90.0, -90.0, 150.0, -150.0)]
near = open_ring + [(0.0, 0.22), (10.0, 0.25), (-10.0, 0.25)]

label, parts = fake_car(near)._why_stalled("linear", 1.0, 0.0, 1.6)
check("只雷达看到近障 -> blocked", label, "blocked")
print("   ", "；".join(parts))

label, parts = fake_car(open_ring, walled(0.55))._why_stalled("linear", 1.0, 0.0, 1.6)
check("雷达盲区但地图画着墙 -> blocked", label, "blocked")
print("   ", "；".join(parts))

label, parts = fake_car(open_ring, walled(None))._why_stalled("linear", 1.0, 0.0, 1.6)
check("雷达空、地图空 -> unknown（不许瞎猜被挡住）", label, "unknown")
print("   ", "；".join(parts))

label, parts = fake_car(open_ring, walled(None))._why_stalled(
    "linear", 1.0, math.radians(11.0), 1.6)
check("什么都没看到但陀螺转了 11° -> slipping", label, "slipping")
print("   ", "；".join(parts))

label, parts = fake_car(near, None)._why_stalled("linear", 1.0, 0.0, 1.6)
check("没有控制台（--web-port 0）时只看雷达 -> blocked", label, "blocked")
print("   ", "；".join(parts))

check("_map_ahead 在无图时返回 None",
      fake_car(near, None)._map_ahead(None, (0.0, 0.0, 0.0), 1.0), None)

# _gap(): 有图走地图，没图退回雷达扇区；两边的读数都必须是带符号的（脱困时按它决定往哪边拧）
car = fake_car(near, walled(0.55))
off, src, cm = car._gap(*car.scan_sample())
print("   有图：缺口 %+.0f°（%s）" % (off, src))
check("_gap 有图时用地图、给出带符号方位", (src, abs(off) > 25.0), ("地图+雷达", True))
car2 = fake_car(near, None)
off2, src2, cm2 = car2._gap(*car2.scan_sample())
print("   无图：缺口 %+.0f°（%s）" % (off2, src2))
check("_gap 无图时退回雷达、不假装看过地图", (src2, cm2 is None), ("只有雷达（地图不可用）", True))
check("_gap 无图时不指向雷达里的近障扇区（正前 ±20°）", abs(off2) > 25.0, True)

# The synthetic world only draws the wall; its sides stay unknown (cost ~0.45), so "widest gap"
# is allowed to pick an unknown direction but must not pick the one with an obstacle in it.
car = fake_car(near, walled(0.55))
cm = car._fuse(*car.scan_sample())
ob = nav.open_bearing(cm, car.pose)
off = (ob + 180.0) % 360.0 - 180.0
print("   最空旷方位 体坐标 %.0f°（墙画在 ±20° 内）" % off)
check("最空旷缺口不指向画着墙的扇区", abs(off) > 25.0, True)

# ---- 4. 脱困阶梯的接线：一次命令里到底试了几下、多久放弃 -----------------------
print("\n== _closed_loop 的阶梯（冻结位姿 = 车顶着东西不动）==")
m.STALL_SECS, m.STALL_GRACE, m.CMD_PERIOD = 0.25, 0.0, 0.02   # 快进快出，判据本身不动


class Alive(object):
    """The loop's `and rclpy.ok()` guard is False without a node, so the offline run needs one."""

    @staticmethod
    def ok():
        return True


m.rclpy = Alive


class Frozen(object):
    """Only what _closed_loop touches. The pose never advances, so every window is a stall."""

    # Bound from the real node rather than copied: the odom-vs-gyro witness is part of what the
    # loop decides with, and a stub copy could disagree with the thing under test.
    _odom_lie_deg = m.Car._odom_lie_deg

    def __init__(self, label, moved):
        self.lock = threading.Lock()
        self.pose = (0.0, 0.0, 0.0, time.monotonic())
        self.imu = (0.0, time.monotonic(), (0.0, 0.0, 0.0, 1.0))
        self.args = SimpleNamespace(safe_distance=0.0, linear_speed=0.25, angular_speed=1.0,
                                    move_timeout=15.0)
        self.acting = False
        self.abort_reason, self.nav_stop, self.stop_src = "", False, ""
        self.last_error = ""
        self.yaw_lie, self.yaw_lie_t = 0.0, 0.0
        self.slip_m, self.slip_t = 0.0, 0.0
        self.cmds = []
        self.escapes = []
        self.label, self.moved = label, moved

    def drive(self, v, w):
        self.cmds.append((v, w))

    def stop_stream(self, secs):
        pass

    def scan_sample(self):
        return (0.0, 0.0, 0.0), []

    def window_min(self, bearing, width):
        return None

    def _net_progress(self, x0, y0, yaw0, kind, sign):
        return 0.0

    def _hold_w(self, yaw0, imu0):
        return 0.0, 0.0

    def _coast_lead(self, need):
        return 0.0

    def _why_stalled(self, kind, sign, gyro_rad, secs):
        return self.label, ["测试桩：由判据给定的定性"]

    def _escape(self, kind, sign, attempt, speed, label):
        self.escapes.append((kind, sign, attempt, label))
        return "桩脱困动作", self.moved


t0 = time.monotonic()
f = Frozen("blocked", m.ESCAPE_BACK)          # 倒退 0.14 m 用光了一次命令的位移预算
ok, msg = m.Car._closed_loop(f, "linear", 0.5, 0.25)
secs = time.monotonic() - t0
check("顶墙 0.5 m：命令判失败", ok, False)
check("  只试 1 次脱困就收手（位移预算 0.14 m 用完）", len(f.escapes), 1)
check("  没有把 15 s 死线烧完（%.1f s 就停了）" % secs, secs < 5.0, True)
check("  回包写清楚是谁判的、试了什么", ("轮子在转但车没动" in msg and "已自动试了 1 次脱困" in msg
                                       and "测试桩" in msg and "桩脱困动作" in msg), True)
print("   ", msg)
check("  外层命令结束时放开了电机所有权", f.acting, False)

t0 = time.monotonic()
f2 = Frozen("slipping", 0.0)                  # 打滑：倒退预算没花掉，可以给到第 2 招
ok, msg = m.Car._closed_loop(f2, "angular", math.radians(90.0), 1.0)
check("打滑转 90°：试满 ESCAPE_TRIES=%d 次后收手" % m.ESCAPE_TRIES, len(f2.escapes), m.ESCAPE_TRIES)
check("  定性原样传给了阶梯（打滑的信息要能影响到第 1 招）", f2.escapes[0][2:4], (1, "slipping"))
print("   ", msg)


class Slipping(Frozen):
    """The book-at-the-wheel case: encoders count metres, the room behind does not recede.

    Before the ring became a second progress meter this condition could not be detected at all --
    the watchdog's only ruler was the odometer that the slipping wheels were busy inflating, so
    the loop saw a car making normal progress and stayed quiet until the command 'completed'.
    """

    def __init__(self, label, rate):
        Frozen.__init__(self, label, 0.0)
        self.t0, self.rate = time.monotonic(), rate

    def _net_progress(self, x0, y0, yaw0, kind, sign):
        return self.rate * (time.monotonic() - self.t0)

    def window_min(self, bearing, width):
        # ahead stays clear so the safe-distance abort is not what ends the move; behind never
        # changes, which is the whole evidence that the car is not going anywhere.
        return 3.0


f3 = Slipping("slipping", 0.25)               # 里程计按命令速度往前走，身后的雷达一动不动
ok, msg = m.Car._closed_loop(f3, "linear", 0.5, 0.25)
check("里程计在涨、雷达不认账：停滞判据必须打得着（以前结构上就够不着）", ok, False)
check("  确实走了脱困阶梯", len(f3.escapes) >= 1, True)
check("  多记的位移被记账并盖了时间戳（建图靠它跳过这些帧）",
      (f3.slip_m > 0.05, f3.slip_t > 0.0), (True, True))
print("   ", msg, "｜多记 %.2f m" % f3.slip_m)

# ---- 5. 阶梯本身：每一招给的是哪个动作、有没有把 acting 交出去 -----------------
print("\n== _escape() 的招式选择 ==")


class Ladder(object):
    """Records the manoeuvres _escape asks for instead of driving them."""

    def __init__(self, astern=1.0, off=-150.0):
        self.args = SimpleNamespace(safe_distance=0.25, linear_speed=0.25, angular_speed=1.0)
        self.calls = []
        self.astern, self.off = astern, off

    def scan_sample(self):
        return (0.0, 0.0, 0.0), []

    def window_min(self, bearing, width):
        return self.astern if abs(bearing) > 150.0 else 1.0

    def _gap(self, pose, pairs):
        return self.off, "桩", None

    def _closed_loop(self, kind, target, speed, escape=True):
        self.calls.append((kind, round(target, 4), escape))
        return True, "桩"

    def _burst(self, v, w, secs):
        self.calls.append(("burst", round(math.degrees(w), 1), secs))
        return 0.0, abs(math.degrees(w)), 0.0


def ladder(kind, sign, attempt, label, **kw):
    c = Ladder(**kw)
    note, moved = m.Car._escape(c, kind, sign, attempt, 1.0, label)
    return c.calls, note, moved


calls, note, _ = ladder("angular", 1.0, 1, "blocked")
check("转向卡住第 1 招 = 反向转 20°，且是嵌套调用", calls, [("angular", round(-math.radians(20.0), 4), False)])
print("   ", note)
calls, note, _ = ladder("angular", 1.0, 1, "slipping")
check("打滑时不许先反向转（车身在动，拧回去只是把线搅得更紧）→ 直接倒",
      calls, [("linear", round(-m.ESCAPE_BACK * 0.7, 4), False)])
print("   ", note)
calls, note, _ = ladder("angular", -1.0, 2, "blocked")
check("转向第 2 招 = 松开顶着的劲儿倒退", calls, [("linear", round(-m.ESCAPE_BACK * 0.7, 4), False)])
calls, note, _ = ladder("linear", 1.0, 1, "blocked")
check("直行第 1 招 = 倒退整个预算", calls, [("linear", round(-m.ESCAPE_BACK, 4), False)])
print("   ", note)
check("  倒退要花掉位移预算", _, m.ESCAPE_BACK)
calls, note, _ = ladder("linear", 1.0, 1, "blocked", astern=0.10)
check("背后 0.10 m 有东西：不许倒退，一招都不动", calls, [])
print("   ", note)
calls, note, _ = ladder("linear", 1.0, 2, "blocked", off=-150.0)
check("直行第 2 招 = 朝缺口拧，且拧短的那一边（-150° → 负角速度）",
      calls[0][0] == "burst" and calls[0][1] < 0.0, True)
print("   ", note)

f3 = Frozen("blocked", 0.14)
m.Car._closed_loop(f3, "linear", 0.5, 0.25, escape=False)   # 脱困动作自己套在里面时
check("嵌套调用（escape=False）不再自己触发阶梯：试了几次 %d" % len(f3.escapes), len(f3.escapes), 0)
check("  回包不许写'已自动试了'", "已自动试了" in f3.last_error, False)
check("  嵌套调用不许把 acting 交还出去（#36 的点刹）", f3.acting, True)

f4 = Frozen("blocked", 0.14)
f4.nav_stop, f4.stop_src = True, "human"
ok, msg = m.Car._closed_loop(f4, "linear", 0.5, 0.25, escape=False)
check("  脱困途中按下的急停不会被下一招收音（人工停止优先）",
      (ok, "收到停止指令" in msg, len(f4.escapes)), (False, True, 0))

f5 = Frozen("blocked", 0.14)
f5.nav_stop, f5.stop_src = True, "human"
m.Car._closed_loop(f5, "linear", 0.5, 0.25)                 # 外层命令才允许重新开账
check("  外层新命令会把上一条的停止标志清掉（否则一次判定卡死所有后续 Move）",
      (f5.nav_stop, f5.stop_src), (False, ""))

print("\n%s" % ("全部通过" if not fails else "失败 %d 项: %s" % (len(fails), fails)))
sys.exit(1 if fails else 0)
