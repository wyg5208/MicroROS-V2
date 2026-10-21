"""Windows-side ROS2 -> MCP bridge for the MicroROS-V2 car.

Runs on the laptop next to the micro-ROS Agent: rclpy talks to ROS2 topics that
the ESP32 board exposes, and the same topic surface is published to the LLM as
MCP tools over streamable-http on :8000 (same contract the vendor's
mcp_service used, so llm_agent_pc/agent.py needs no changes).

Requires: ROS2 Humble Windows environment sourced, Python 3.10 (rclpy only, no pip deps)
"""
import argparse
import inspect
import json
import math
import os
import threading
import time
import urllib.request
import uuid
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, UInt16

import nav   # costmap + A* + trajectory scoring; must live in this process, see nav.py

CMD_PERIOD = 0.05          # board expects a continuous cmd_vel stream
ODOM_TOPIC = "/odom_raw"   # the board's own odometry; /odom only exists in the vendor VM stack
# Heading hold for straight moves: a `Move forward` that publishes w=0.0 on every tick wanders,
# and the sign flips from run to run -- so what was missing is feedback, not a wheel trim to tune
# out. The floor matters more than the gain: this board ignores a turn command below roughly
# 0.12 rad/s (nav.ACT_W_CUTON, measured across the sweep), so a trim smaller than that is a
# command to stand still and the hold silently does nothing.
HOLD_DEAD = math.radians(2.0)    # heading error tolerated before correcting
HOLD_KP = 6.0                    # rad/s commanded per rad of error (reaches the cap at 3 deg)
HOLD_MIN = 0.22                  # rad/s: the smallest correction the chassis actually obeys
HOLD_MAX = 0.35                  # rad/s: a third of CAP_W, so a hold can never look like a turn
# That gain fit is NOT restricted to a standing car, which is worth writing down because the
# opposite was believed for two days: `_cruise_w.py` commands v=+-0.20 with w=+-0.30 for 1.2 s
# and the heading moves 16-18 deg per leg, ~0.24 rad/s out of 0.30 commanded -- the same ratio as
# at rest. An earlier variant of this hold slowed the move to 0.10 m/s "to buy turning authority";
# that was chasing an artifact (see the console's cmd_vel stream, fixed 2026-10-18: idle zero
# velocity was being published into the middle of every Move, which both braked the car 5 times a
# second and swamped the trim), and the slowdown cost 40% of the move time for no gain.
# With a clean stream and this hold, a 0.5 m hop ends with a 0.7-1.1 deg peak heading error and
# 0.21 m/s sustained -- it used to take 10 s and wander 3-6 deg, which is what made `Move` look
# like it needed a longer timeout than --move-timeout.
# A released turn keeps sliding, and the slide is a property of the rate, not of the angle:
# measured 2026-10-18 with a clean cmd_vel stream, `Move turn_right 30` let go at 96% of the angle
# while the board was still doing ~0.79 rad/s and settled +3.97 deg past the target; turn_left 30
# settled +3.44 deg. Both are 0.08 s of the release rate, which is why the early release below is
# quoted as rate x time -- the old "~5 deg of coast" looked like a constant only because every
# test happened to release at the same full rate, and it is wrong for the 15 deg turns it was
# fitted on (5 of 15 is a third of the command).
COAST_TAU = 0.08                   # s the board keeps sliding after the last commanded twist
# The proportional tail -- 2.5 * err / need of full speed -- has a fixed point if it can decelerate
# below what the wheels execute: measured on a -45 deg turn, released at 42.96/45.00 deg (tolerance
# 1.8 deg) and burned the whole 15 s deadline doing nothing. nav.achieved() predicts 2.12 deg of
# stuck error for that deadband, which is the 2.04 deg the car actually stopped short by -- the
# arithmetic and the bench agree, so turns floor at nav.turn_floor() instead of crawling into it.
# Stall watchdog for a single-step Move. Before this, the only answer a blocked car had was the
# --move-timeout deadline: it kept pushing into the thing for 15 s and then handed the model the
# sentence "可能打滑或被挡住", i.e. the diagnosis was delegated to a guess. The tests below are
# relative to nav.achieved(cmd) rather than absolute, because the loop deliberately crawls at the
# end of every command (v floors at 0.06 m/s) and a fixed threshold calls that a blockage.
STALL_SECS = 1.6                   # s of commanded motion that must buy this much progress...
STALL_FRAC = 0.25                  # ... or the car is not moving, whatever the odometer is doing
STALL_MIN_M = 0.03                 # m: absolute floor, so the crawl tail cannot trip on odom noise
STALL_MIN_W = math.radians(6.0)    # rad: same floor for a turn
STALL_GRACE = 0.8                  # s: the board queues commands for ~0.2-0.3 s and a Move starts
                                   # from standstill -- measured while fixing the coast lead
ESCAPE_TRIES = 2                   # escape manoeuvres per Move before it gives up and reports
ESCAPE_BACK = 0.14                 # m of backing off, the whole distance budget for one Move
ESCAPE_SECS = 1.2                  # s a single burst manoeuvre may take (~30 deg of re-aim)
# Measured 2026-10-19 with a book at the drive wheels: the car did not turn, the gyro said 8 deg
# of wander, and wheel odometry reported 40 deg -- while the stall watchdog stayed silent because
# the same lying odometer was the only progress meter it had. Both facts below come from that.
SLIP_YAW_DEG = 15.0                # deg of odom-vs-gyro heading disagreement that proves odom is
                                   # inventing rotation (slip / one wheel dragging)
SLIP_FRAC = 0.3                    # the ring may only overrule odom's progress when the surface the
                                   # car is moving *away from* reports less than this fraction of it
# Measured on this board (docs 16/17 disagree with the hardware): /cmd_ptz_angle .x is the
# pitch servo S1 and .y is the yaw servo S2, and neither zero is "looking forward".
# The references below are anchored to the user's eyes, not to a photo guess: at servo (80,-45)
# the camera sits 20 deg low and 20 deg right of straight ahead, and *increasing* .y swings it
# left. Both were only observable from the floor -- a frame of the ceiling and a frame of the
# floor look equally plausible when you have never seen the car from outside.
PTZ_SERVO_PITCH_LEVEL = 60.0   # .x value where the camera looks horizontal
PTZ_SERVO_YAW_CENTRE = -25.0   # .y value where the camera looks straight ahead
PTZ_PITCH_LIMIT = (-40.0, 15.0)   # user-facing, + = tilt up
PTZ_YAW_LIMIT = (-45.0, 65.0)     # user-facing, + = turn right
# Measured on this car: the PTZ module dips into the lidar's scan plane, so the slice
# straight ahead only ever returns 0.06-0.11 m (it tracks the gimbal pitch, and stays put
# through a full 360 degree spin). That slice is unknown, not free space.
SELF_RANGE_MAX = 0.15                 # closer than this is the car itself
BLIND_WEDGE = (-24.0, 20.0)           # lidar-frame degrees the gimbal occludes
# Bearing sense is counter-clockwise positive, i.e. standard ROS, and /odom_raw yaw agrees
# with it. Proved with a trash bin placed on the car's left at 0.5 m: it returned at raw
# +33 deg. An earlier cross-correlation test suggested the opposite, but its expected sign
# was backwards (when the car turns left, a fixed object's sensor-relative bearing must
# DECREASE), and negating the scan would smear every wall 2x during a turn.
SCAN_SIGN = 1.0
SCAN_SWEEP = 1.0 / 15.0               # s for a full 360 sweep; /scan publishes at ~15 Hz
PROTOCOL_VERSION = "2025-03-26"

# Move(direction="arc_left"/"arc_right"). The fan the clearance check looks through is moved to the
# side the car is bending toward, because the next metre of a curve is not off the nose.
ARC_LOOK_DEG = 30.0
# How far the swept angle may disagree with (arc length / radius) before the "arc" is called what
# it is: a flat path. Both numbers come from the same odometer, so on a real arc they agree to a
# couple of degrees -- the slack is the ±5 cm position quantisation and a sampled tick, not model
# uncertainty. A wide tolerance here is exactly what lets "it drove 0.3 m of straight line" pass.
ARC_SWEEP_TOL_DEG = 12.0
ARC_SWEEP_TOL_FRAC = 0.35         # ... plus this share of the expected sweep, for long arcs
ARC_STRAIGHT_FRAC = 0.4               # share of ticks whose curvature ask fell in the dead band
# ... and the floor that keeps them out of it: nav.arc_min_speed() is the mathematical limit, and
# driving right at a limit is driving where one rounding step means "this tick goes straight".
ARC_TAIL_MARGIN = 1.15


def stalled(want, got, win, kind):
    """True when a window of commanded motion bought no movement worth the name.

    Two tests, and both have to say "not moving": relative to what the chassis should have
    delivered (so a blocked car trips at any speed) and absolute over the window (so the crawl
    at the end of every command, where 0.06 m/s is deliberate, is not called a blockage).
    Kept as a free function so `_stall_test.py` can hand it the numbers from a recording.
    """
    step_m = STALL_MIN_M if kind == "linear" else STALL_MIN_W
    return want > 0.0 and got < min(STALL_FRAC * want, step_m / max(win, 1e-6))


def signed_deg(d):
    """Fold a 0..360 bearing into -180..180, so 'the gap is at 300°' becomes a 60° twist left."""
    return (float(d) + 180.0) % 360.0 - 180.0


def tail_cmd(kind, speed, err, need):
    """The command the closed loop uses when it is closing on its target.

    Proportional to the remaining error, but never below what the chassis will execute: a turn
    command under the executed band is a command to stand still, which used to park the loop in a
    fixed point ~2 deg short of a 45 deg turn until the deadline expired.
    """
    crawl = 0.06 if kind == "linear" else nav.turn_floor()
    return min(speed, max(crawl, speed * min(1.0, err / max(need, 1e-6) * 2.5)))

_JSON_TYPES = {str: "string", int: "integer", float: "number", bool: "boolean"}


def _schema_of(func):
    """Build a JSON Schema from the tool signature (typed params, optional defaults)."""
    sig = inspect.signature(func)
    hints = func.__annotations__
    props, required = {}, []
    for name, param in sig.parameters.items():
        if param.kind in (param.VAR_POSITIONAL, param.VAR_KEYWORD):
            continue
        props[name] = {"type": _JSON_TYPES.get(hints.get(name), "string")}
        if param.default is param.empty:
            required.append(name)
        elif isinstance(param.default, (int, float, str, bool)):
            props[name]["default"] = param.default
    schema = {"type": "object", "properties": props}
    if required:
        schema["required"] = required
    return schema


class McpServer:
    """Minimal MCP streamable-http endpoint built on the standard library."""

    def __init__(self, name):
        self.name = name
        self.tools = {}
        self.sessions = set()
        self.httpd = None

    def tool(self, description=""):
        def wrap(func):
            self.tools[func.__name__] = (description or func.__doc__ or func.__name__,
                                         _schema_of(func), func)
            return func
        return wrap

    def _call(self, method, params):
        if method == "initialize":
            sid = uuid.uuid4().hex
            self.sessions.add(sid)
            return {"protocolVersion": PROTOCOL_VERSION,
                    "capabilities": {"tools": {}},
                    "serverInfo": {"name": self.name, "version": "1.0"}}, sid
        if method == "notifications/initialized":
            return None, None
        if method == "ping":
            return {}, None
        if method == "tools/list":
            return {"tools": [{"name": n, "description": d, "inputSchema": s}
                              for n, (d, s, _) in self.tools.items()]}, None
        if method == "tools/call":
            name = params.get("name")
            entry = self.tools.get(name)
            if not entry:
                return {"error": (-32602, "unknown tool: %s" % name)}, None
            _, schema, func = entry
            kwargs = dict(params.get("arguments") or {})
            # 大模型有时把数字写成字符串，按 schema 类型纠正
            for k, v in list(kwargs.items()):
                t = schema.get("properties", {}).get(k, {}).get("type")
                if isinstance(v, str) and t in ("integer", "number"):
                    try:
                        kwargs[k] = int(v) if t == "integer" else float(v)
                    except ValueError:
                        pass
            try:
                text = func(**kwargs)
                err = False
            except Exception as exc:
                text = "%s: %s" % (type(exc).__name__, exc)
                err = True
            return {"content": [{"type": "text", "text": str(text)}], "isError": err}, None
        return {"error": (-32601, "method not found: %s" % method)}, None

    def serve(self, host, port):
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *args):
                pass

            def _reply(self, code, body, extra=None):
                data = body.encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Length", str(len(data)))
                for k, v in (extra or {}).items():
                    self.send_header(k, v)
                self.end_headers()
                if data:
                    self.wfile.write(data)

            def do_POST(self):
                try:
                    n = int(self.headers.get("Content-Length", 0))
                    req = json.loads(self.rfile.read(n).decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    self._reply(400, '{"jsonrpc":"2.0","error":{"code":-32700,'
                                     '"message":"parse error"},"id":null}')
                    return
                method = req.get("method")
                sid = self.headers.get("Mcp-Session-Id")
                if sid and sid not in outer.sessions:
                    self._reply(404, '{"jsonrpc":"2.0","error":{"code":-32001,'
                                     '"message":"unknown session"},"id":null}')
                    return
                result, new_sid = outer._call(method, req.get("params") or {})
                if isinstance(result, dict) and "error" in result:
                    code, msg = result["error"]
                    self._reply(200, json.dumps({"jsonrpc": "2.0", "id": req.get("id"),
                                                 "error": {"code": code, "message": msg}}),
                                {"Content-Type": "application/json"})
                    return
                if result is None:
                    self._reply(202, "", {})
                    return
                extra = {"Content-Type": "application/json"}
                if new_sid:
                    extra["Mcp-Session-Id"] = new_sid
                self._reply(200, json.dumps({"jsonrpc": "2.0", "id": req.get("id"),
                                             "result": result}), extra)

            def do_DELETE(self):
                outer.sessions.discard(self.headers.get("Mcp-Session-Id"))
                self._reply(204, "", {})

            def do_GET(self):
                self._reply(405, "", {})

        self.httpd = ThreadingHTTPServer((host, port), Handler)
        self.httpd.daemon_threads = True
        print("MCP 服务已启动: http://%s:%d/mcp  工具: %s"
              % (host, port, ", ".join(self.tools)))
        self.httpd.serve_forever()



def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def yaw_of_tuple(q):
    """The same, for an IMU orientation kept as a plain (x, y, z, w) tuple."""
    return math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                      1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


DIR_NAMES = ((0.0, "正前"), (45.0, "左前"), (90.0, "正左"), (135.0, "左后"),
             (180.0, "正后"), (-135.0, "右后"), (-90.0, "正右"), (-45.0, "右前"))


def dir_name(deg):
    """Chinese label for a lidar-frame bearing (0 = forward, counter-clockwise positive)."""
    return min(DIR_NAMES, key=lambda kv: abs(wrap180(deg - kv[0])))[1]


class Car(Node):
    def __init__(self, args):
        super().__init__("pc_mcp_bridge")
        self.args = args
        self.lock = threading.Lock()
        self.pose = None            # (x, y, yaw, monotonic_time), relocation bias included
        self.pose_raw = None        # the same tuple straight from wheel odometry
        self.bias = (0.0, 0.0)      # metres added to odometry x/y after a carry was re-registered
        self.pose_hist = deque(maxlen=80)   # recent timed poses, for pose_at() lookups
        self.scan_pose = None       # pose interpolated to the middle of the newest sweep
        self.pairs = []             # usable (bearing_deg, range_m) after self/blind masking
        self.scan_t = None          # when /scan last arrived
        self.ptz = [0.0, 0.0]       # user-facing yaw/pitch offsets from the level, forward pose
        self.last_calib = None      # measured twist from the most recent rate_test burst
        self.volts = None           # /battery, 3S pack; the firmware locks the motors near 10.3
        self.imu = None             # (gyro_wz rad/s, monotonic, (qx,qy,qz,qw)) from /imu; see _imu_cb
        # Latest odom-vs-gyro heading disagreement, in degrees, and when it was last seen. The
        # console holds these frames out of the map: a sweep folded at a heading the wheels
        # invented is how a lie becomes a wall. See _odom_lie_deg.
        self.yaw_lie = 0.0
        self.yaw_lie_t = 0.0
        # Second, independent lie: the encoders counted metres the room never saw recede. Same
        # reason to hold a sweep out of the map -- here it is the *position* being invented, not
        # the heading, so a straight push against a book ghosts a wall straight ahead.
        self.slip_m = 0.0
        self.slip_t = 0.0
        self.last_error = ""
        self.nav_stop = False       # set by Stop() so it can interrupt a running tour
        # Who asked for that stop. A human's 急停 and the machine's own distrust both end up
        # zeroing the motors, and the running action has to report which one unwound it --
        # otherwise a map problem is logged as "收到停止指令" and the operator goes looking for
        # a hand that was never on the button. Anything not recorded as "machine" is treated
        # as human, so an unknown source still stops the wheels.
        self.stop_src = ""
        # Set by the console's mid-motion relocation check (car_web._motion_watch) when the map
        # stops explaining the sweep while the wheels are turning. `acting` is what tells the
        # console that a machine -- not a human on the teleop button -- owns the motors right now,
        # which is the difference between pulling them and merely warning.
        self.acting = False
        self.abort_reason = ""

        qos_cmd = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        # micro-ROS endpoints are BEST_EFFORT; a RELIABLE subscription never matches them
        qos_stream = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", qos_cmd)
        self.ptz_pub = self.create_publisher(Vector3, "/cmd_ptz_angle", 10)
        self.beep_pub = self.create_publisher(UInt16, "/beep", 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, qos_stream)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, qos_stream)
        self.create_subscription(Float32, "/battery", self._battery_cb, qos_stream)
        self.create_subscription(Imu, "/imu", self._imu_cb, qos_stream)
        self.get_logger().info("bridge node up; waiting for %s from the board" % ODOM_TOPIC)

    def _battery_cb(self, msg):
        with self.lock:
            self.volts = float(msg.data)
        # The 12 V motors lose reachable duty as the 3S pack sags, so the response the fits
        # measured at one voltage is wrong at another. nav.achieved() scales by this.
        nav.ACT_VOLTS = self.volts

    def _imu_cb(self, msg):
        # The tour map smeared into concentric arcs: wheel odometry's yaw lags the physical
        # rotation, so each scan got folded at a heading the car no longer had. The gyro is the
        # only other heading source on this board, so keep it and let the calibration decide
        # whether it beats the odometry.
        q = msg.orientation
        with self.lock:
            self.imu = (float(msg.angular_velocity.z), time.monotonic(),
                        (float(q.x), float(q.y), float(q.z), float(q.w)))

    def _odom_cb(self, msg):
        p = msg.pose.pose
        with self.lock:
            yaw = yaw_of(p.orientation)
            t = time.monotonic()
            self.pose_raw = (p.position.x, p.position.y, yaw, t)
            # Correcting a carry at the source, not in the fold loop: the planner, the closed
            # loops and the console all read `self.pose`, and a bias they each added themselves
            # would be three chances to disagree about where the car is.
            self.pose = (p.position.x + self.bias[0], p.position.y + self.bias[1], yaw, t)
            self.pose_hist.append(self.pose)

    def set_bias(self, bx, by):
        """Reset where odometry lands in the map frame, and re-stamp everything already held.

        pose_at() interpolates a sweep's own pose out of pose_hist, so a bias that only applied
        to future messages would fold the current sweep at the old place and the next one at the
        new -- one smeared sweep every time the car is re-registered.
        """
        with self.lock:
            self._bias_locked(float(bx), float(by))

    def nudge_bias(self, dx, dy):
        """Add a small offset to the bias as one indivisible step.

        The drift suppressor spends centimetres several times a tour while a human may be
        clicking a relocation candidate at the same moment; a read-modify-write from the caller
        would let one of the two silently overwrite the other.
        """
        with self.lock:
            self._bias_locked(self.bias[0] + float(dx), self.bias[1] + float(dy))
        return self.bias

    def _bias_locked(self, bx, by):
        old = self.bias
        self.bias = (bx, by)
        dx, dy = bx - old[0], by - old[1]
        self.pose_hist = deque(((x + dx, y + dy, yaw, t)
                                for (x, y, yaw, t) in self.pose_hist), maxlen=80)
        if self.pose is not None:
            x, y, yaw, t = self.pose
            self.pose = (x + dx, y + dy, yaw, t)
        if self.scan_pose is not None:
            x, y, yaw, t = self.scan_pose
            self.scan_pose = (x + dx, y + dy, yaw, t)

    def pose_at(self, t):
        """Odometry pose interpolated at an earlier instant, or None if the buffer moved on.

        A scan is folded at the pose its first photon was measured at, not at whatever the
        wheel odometry reports 200 ms later: while turning at 26 deg/s that lag alone rotates
        the whole sweep by up to 5 degrees, and a wall 4 m away lands 35 cm off.
        """
        with self.lock:
            h = list(self.pose_hist)
        for k in range(1, len(h)):
            if h[k][3] >= t:
                a, b = h[k - 1], h[k]
                span = b[3] - a[3]
                u = 0.0 if span <= 0 else max(0.0, min(1.0, (t - a[3]) / span))
                dy = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
                return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]),
                        a[2] + u * dy, t)
        return None

    def _scan_cb(self, msg):
        amin = math.degrees(msg.angle_min)
        ainc = math.degrees(msg.angle_increment)
        n = len(msg.ranges)
        sweep = msg.time_increment * n if msg.time_increment else SCAN_SWEEP
        now = time.monotonic()
        mid = self.pose_at(now - 0.5 * sweep)
        with self.lock:
            wz = 0.0 if self.imu is None else self.imu[0]
        # Un-spin the sweep: the lidar turns through wz*sweep radians of body heading while it
        # collects one frame, and stamping every ray at the same pose smears a far wall into
        # concentric arcs while the near walls stay sharp -- exactly the tour map's failure.
        twist = wz * sweep
        pairs = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= SELF_RANGE_MAX or r >= msg.range_max:
                continue
            a = SCAN_SIGN * (amin + i * ainc) - math.degrees(twist * (1.0 - i / float(n)))
            a = (a + 180.0) % 360.0 - 180.0
            if BLIND_WEDGE[0] <= a <= BLIND_WEDGE[1]:
                continue
            pairs.append((a, r))
        with self.lock:
            self.pairs = pairs
            self.scan_t = now
            if mid is not None:
                self.scan_pose = mid

    def scan_sample(self):
        """The newest sweep paired with the pose it was measured at, not the pose now."""
        with self.lock:
            return self.scan_pose or self.pose, list(self.pairs)

    def scan_age(self):
        with self.lock:
            return (time.monotonic() - self.scan_t) if self.scan_t else None

    def sector_ring(self, n=12):
        """Nearest usable return per equal bearing sector, None where nothing was seen."""
        with self.lock:
            pairs = list(self.pairs)
        ring = [None] * n
        for a, r in pairs:
            i = int(((a + 360.0) % 360.0) / (360.0 / n)) % n
            if ring[i] is None or r < ring[i]:
                ring[i] = round(r, 2)
        return ring

    def window_min(self, center_deg, half_deg):
        with self.lock:
            pairs = list(self.pairs)
        best = None
        for a, r in pairs:
            if abs(wrap180(a - center_deg)) <= half_deg and (best is None or r < best):
                best = r
        return best

    # ---- motion primitives -------------------------------------------------
    def drive(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.cmd_pub.publish(t)

    def stop_stream(self, hold_s=0.4):
        """Keep sending zero velocity so the board's command watchdog clears."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < hold_s:
            self.drive(0.0, 0.0)
            time.sleep(CMD_PERIOD)

    def move_linear(self, distance, speed):
        return self._closed_loop("linear", distance, speed)

    def turn(self, angle_deg, speed):
        return self._closed_loop("angular", math.radians(angle_deg), speed)

    def arc(self, radius, distance, side, speed):
        """Drive `distance` metres along a circle of `radius` metres, curving toward `side`.

        Two rulers again, and this time they measure different things: the odometer can count
        metres the room never saw (slipping), and the car can travel every one of those metres
        while going straight -- because the curvature command fell into the band the wheels ignore,
        or because the radius asked for is outside what `nav.arc_band` says this chassis can draw.
        Geometry ties the two together (swept angle = arc length / radius), so a curve that is
        really a straight line shows up as a disagreement rather than as a clean pass, and this is
        the case an open-loop `w = v/R` would have folded into the map at a heading the car never had.

        Not covered here, on purpose: reversing along an arc, and the ring-behind slip witness
        `_closed_loop` uses -- that ruler reads the wall astern, and a curving car's stern swings
        through it, so it would have to be re-derived before it means anything.
        """
        need = abs(float(distance))
        rad = abs(float(radius))
        side = 1.0 if float(side) >= 0.0 else -1.0     # +1 = 向左弯（逆时针）
        if need <= 0.0:
            return True, "弧线距离 0 米，没动"
        r_min, r_max = nav.arc_band(speed)
        if rad > r_max or rad < r_min:
            which = ("太缓：这条实际会走成直线，要直走请用 direction=forward"
                     if rad > r_max else
                     "太急：转角命令已经到上限还是弯不过来，把半径放大或调低 --linear-speed")
            self.last_error = "半径 %.2f m 画不出来（命令 %.2f m/s 时能稳住的圆是 %.2f ~ %.2f m）：%s" % (
                rad, speed, r_min, r_max, which)
            return False, self.last_error
        with self.lock:
            if self.pose is None:
                self.last_error = "没有收到 %s 数据" % ODOM_TOPIC
                return False, self.last_error
            x0, y0, yaw0, t0 = self.pose
        if t0 is not None and time.monotonic() - t0 > 3.0:
            self.last_error = "%s 已停滞 %.1f 秒，检查车是否在线" % (
                ODOM_TOPIC, time.monotonic() - t0)
            return False, self.last_error

        # The reference circle is fixed once, from the pose this command started at. Re-deriving it
        # every tick would be "wherever the car says it is, that is the circle", and the feedback
        # would then chase the odometer's own drift into a spiral and call it progress.
        cx = x0 - side * rad * math.sin(yaw0)
        cy = y0 + side * rad * math.cos(yaw0)

        travelled = 0.0          # m of arc, summed along the path -- not the chord
        sw_odom = 0.0            # rad swept, signed in the commanded sense
        sw_gyro = 0.0            # ... and the same quantity from a sensor that ignores the wheels
        # The curvature witness compares INCREMENTS, not totals: an escape manoeuvre backs the car
        # up and twists it, which moves both totals in ways the circle never promised, and the first
        # such attempt would then be reported as "it drifted straight" instead of what it was.
        s_base = 0.0
        sw_base = 0.0
        prev = (x0, y0)
        prev_yaw = yaw0
        err = need
        v = w = 0.0
        straight_ticks = 0
        ticks = 0
        tries = 0
        extra_m = 0.0
        notes = []
        rate_t0 = t_start = time.monotonic()
        rate_ref = 0.0
        gyro_prev = None
        deadline = time.monotonic() + self.args.move_timeout
        ok = False
        self.acting = True
        self.abort_reason = ""
        self.nav_stop = False
        self.stop_src = ""
        try:
            while time.monotonic() < deadline and rclpy.ok():
                if self.abort_reason:
                    self.last_error = self.abort_reason
                    return False, self.last_error
                if self.nav_stop:
                    if self.stop_src == "machine":
                        self.last_error = "机器判定中止（弧线已完成 %.2f/%.2f 米）" % (travelled, need)
                    else:
                        self.last_error = "收到停止指令（弧线已完成 %.2f/%.2f 米）" % (travelled, need)
                    return False, self.last_error
                with self.lock:
                    cur, im = self.pose, self.imu
                if cur is None:
                    self.last_error = "弧线途中没有里程计数据"
                    return False, self.last_error
                x, y, yaw = cur[0], cur[1], cur[2]
                travelled += math.hypot(x - prev[0], y - prev[1])
                sw_odom += side * wrap_pi(yaw - prev_yaw)
                prev, prev_yaw = (x, y), yaw
                ticks += 1
                d_s = travelled - s_base
                d_sw = sw_odom - sw_base
                if ticks > 3 and d_s > 0.15:
                    # The curvature witness. A left arc of radius R that covers s metres must have
                    # turned s/R radians; if it has not, the metres went into a path that is flatter
                    # than the one that was asked for, and every number in the reply would lie.
                    exp_sw = d_s / rad
                    if abs(exp_sw - d_sw) > math.radians(ARC_SWEEP_TOL_DEG) + \
                            ARC_SWEEP_TOL_FRAC * exp_sw:
                        real_r = d_s / max(abs(d_sw), 1e-6)
                        self.last_error = ("车没照命令的半径弯：这段走了 %.2f m 只转过 %.0f°，实际半径约 %.1f m"
                                           "（命令 %.2f m）——已中止，这段别拿去建图，也别原样重发；"
                                           "先看是不是半径要求落在轮子不转的带宽里，或车被顶住只能直走" % (
                                               d_s, math.degrees(d_sw), real_r, rad))
                        return False, self.last_error
                err = need - travelled
                if err <= max(0.02, 0.04 * need):
                    ok = True
                    break
                # A curve's next metre is not in front of the nose: it is on the side it is bending
                # toward, so the clearance fan moves with the sign of the turn.
                clear = self.window_min(side * ARC_LOOK_DEG, 40.0)
                if clear is not None and clear < self.args.safe_distance:
                    self.last_error = ("行进方向（%s %.0f°）%.2f m 内有障碍，已中止（完成 %.2f/%.2f 米）" % (
                        "左前" if side > 0 else "右前", ARC_LOOK_DEG, clear, travelled, need))
                    return False, self.last_error
                # An arc's tail cannot ease down the way a straight run does: below `arc_min_speed`
                # the curvature this circle asks for needs a turn command inside the dead band, and
                # `arc_plan` rightly refuses it -- so the last centimetres would go straight, which
                # is both a wrong path and a false curvature alarm. The margin is not slack for its
                # own sake: at the bare limit the wheels sit exactly on the cut-on, so one rounding
                # step either way spends the tick going straight and leaves no authority to correct.
                v = max(tail_cmd("linear", speed, err, need),
                        min(speed, ARC_TAIL_MARGIN * nav.arc_min_speed(rad)))
                w, _kappa, straight = nav.arc_plan(rad, math.hypot(x - cx, y - cy) - rad, v, side)
                straight_ticks += 1 if straight else 0
                self.drive(v, w)
                time.sleep(CMD_PERIOD)
                now = time.monotonic()
                with self.lock:
                    im = self.imu
                if im is not None and now - im[1] < 0.5:
                    if gyro_prev is not None:
                        sw_gyro += side * im[0] * max(0.0, min(0.3, im[1] - gyro_prev))
                    gyro_prev = im[1]
                if travelled - rate_ref > STALL_MIN_M and not straight:
                    rate_t0, rate_ref, gyro_prev = now, travelled, None
                    continue
                win = now - rate_t0
                if win < STALL_SECS or now - t_start < STALL_GRACE:
                    continue
                exp_v, _exp_w = nav.achieved(v, w)
                got = (travelled - rate_ref) / win
                if stalled(exp_v, got, win, "linear"):
                    label, parts = self._why_stalled("linear", side, abs(sw_gyro), win)
                    if tries >= ESCAPE_TRIES or extra_m >= ESCAPE_BACK:
                        if tries:
                            self.last_error = ("弧线走不动了（命令 %.2f m/s，实测 %.2f m/s，%.1f s 没进展）："
                                               "%s。已自动试了 %d 次脱困（%s）还是不通，停下来等人工处理" % (
                                                   exp_v, got, win, "；".join(parts), tries, "，".join(notes)))
                        else:
                            self.last_error = "弧线走不动了（命令 %.2f m/s，实测 %.2f m/s）：%s" % (
                                exp_v, got, "；".join(parts))
                        return False, self.last_error
                    tries += 1
                    note, moved = self._escape("linear", side, tries, speed, label)
                    notes.append("第%d次脱困：%s" % (tries, note))
                    extra_m += moved
                    # Re-base both rulers on where the escape left the car, and refuse to bank the
                    # back-off as arc progress: the car did not gain 0.14 m of curve by losing it.
                    with self.lock:
                        cur = self.pose
                    if cur is not None:
                        prev, prev_yaw = (cur[0], cur[1]), cur[2]
                    travelled = max(0.0, travelled - moved)
                    s_base, sw_base = travelled, sw_odom
                    rate_t0, rate_ref, gyro_prev = time.monotonic(), travelled, None
        finally:
            self.acting = False
            self.stop_stream(0.4)

        if ok:
            # The board keeps its own motion queue, so the car travels and turns a little past the
            # tick the loop let go. Bill what it actually did -- and only until the pose stops moving.
            deadline = time.monotonic() + 2.0
            last = travelled
            while time.monotonic() < deadline:
                time.sleep(0.25)
                with self.lock:
                    cur = self.pose
                if cur is None:
                    break
                travelled += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
                sw_odom += side * wrap_pi(cur[2] - prev_yaw)
                prev, prev_yaw = (cur[0], cur[1]), cur[2]
                if travelled - last < 0.003:
                    break
                last = travelled
        doubt = []
        if sw_odom > 0.1 and abs(sw_odom - sw_gyro) > math.radians(SLIP_YAW_DEG):
            doubt.append("里程计朝向与陀螺差到 %.0f°，这段的朝向和位移都别当真" %
                         math.degrees(abs(sw_odom - sw_gyro)))
        if straight_ticks > ARC_STRAIGHT_FRAC * max(ticks, 1):
            doubt.append("有 %.0f%% 的拍子半径要求落进轮子不转的带宽里，这一整段其实是在边走边直走" % (
                100.0 * straight_ticks / max(ticks, 1)))
        doubt = ("；注意：" + "，".join(doubt)) if doubt else ""
        real_r = travelled / max(abs(sw_odom), 1e-6) if travelled > 0.05 else 0.0
        did = ("；中途受阻 %d 次（%s）" % (tries, "，".join(notes))) if notes else ""
        tail = "已完成弧线 %.2f/%.2f 米（半径 %.2f m %s，实测 %.2f 米、扫过 %.0f°、实际半径约 %.2f m）%s%s" % (
            travelled, need, rad, "向左" if side > 0 else "向右", travelled,
            math.degrees(abs(sw_odom)), real_r, did, doubt)
        if ok:
            return True, tail
        spent = max(0.1, time.monotonic() - t_start)
        self.last_error = ("弧线超时未完成（%.2f/%.2f 米，%.1f s 内平均 %.2f m/s），没触发停滞判定说明车在动只是慢；"
                           "已试脱困 %d 次%s%s。下一步建议：ScanRing 看环视、MapInfo 换目标，或 Stop 后人工检查" % (
                               travelled, need, spent, travelled / spent, tries,
                               ("（%s）" % "，".join(notes)) if notes else "", doubt))
        return False, self.last_error

    def drive_tour(self, budget, out_dir="", world=None, trail=None):
        """Walk the room's outline while mapping it, and stop where the mapping began.

        Every tick: fold the live lidar ring into a growing odom-frame grid, cut a costmap
        around the car from that grid plus the current returns, aim at the bearing with the
        most still-unmapped ground (ties resolve left, so the tour runs counter-clockwise),
        A* there, and let the trajectory scorer pick the arc. Reversing is one of the
        candidate arcs and the loop will take it, which is the difference between this and
        the wall-follower it replaces: a corner that used to be answered by pushing into it
        now gets backed out of.
        """
        with self.lock:
            if self.pose is None:
                return "失败: 没有收到 %s 数据" % ODOM_TOPIC
            if self.volts is not None and self.volts < 10.5:
                return "失败: 电池 %.2f V，固件低压锁会让车走一半停住" % self.volts
            x0, y0 = self.pose[0], self.pose[1]
        self.nav_stop = False
        self.abort_reason = ""
        self.stop_src = ""
        self.acting = True            # a tour run from MCP has no console `busy` claim; the
                                      # relocation check needs to know a machine owns the motors
        # The web console hands in its own grid so the map on screen keeps growing while the
        # car walks; the MCP tool leaves both None and gets a private, throwaway map.
        world = nav.World() if world is None else world
        pilot = nav.Pilot(world, (x0, y0))
        t0 = time.monotonic()
        last, travelled = (x0, y0), [(x0, y0)]
        last_mark = t0
        dist = 0.0
        reason = "时间到"
        try:
            while time.monotonic() - t0 < budget and rclpy.ok():
                if self.abort_reason:
                    # said out loud rather than reported as "收到停止指令": a tour that stopped
                    # because the map stopped matching is a different event from anyone pressing
                    # Stop, and the caller deciding whether to retry needs to tell them apart
                    reason = self.abort_reason
                    break
                if self.nav_stop:
                    reason = ("机器判定中止（未写明原因）" if self.stop_src == "machine"
                              else "收到停止指令")
                    break
                age = self.scan_age()
                pose, pairs = self.scan_sample()
                if age is None or age > 1.5 or pose is None:
                    reason = "雷达或里程计断了"
                    break
                dist += math.hypot(pose[0] - last[0], pose[1] - last[1])
                last = (pose[0], pose[1])
                t = time.monotonic() - t0
                if t > 25.0 and dist > 4.0 and math.hypot(pose[0] - x0, pose[1] - y0) < 0.5:
                    reason = "绕行一周回到出发点"
                    break
                # one decision per tick, in the same Pilot the simulator validates
                v, w = pilot.step(t, pose, pairs)
                if t - (last_mark - t0) > 0.2:
                    travelled.append(last)
                    if trail is not None:
                        trail.append((pose[0], pose[1], pose[2]))
                    last_mark = t0 + t
                self.drive(v, w)
                time.sleep(CMD_PERIOD)
        finally:
            self.acting = False     # must be released: while it stayed set the console believed
                                    # an MCP command owned the motors forever after the tour, which
                                    # disabled the standing carry check ("电机被 MCP 命令 占用" for
                                    # the rest of the session) and let the machine yank teleop out
                                    # of a human's hands on every later motion verdict
            self.stop_stream(0.6)
        if out_dir:
            nav.save_world(world, out_dir, travelled)
        with self.lock:
            p = self.pose
        gx, gy = pilot.goal if pilot.goal else (0.0, 0.0)
        return ("%s：走了 %.2f m，建图 %d 帧/占据 %d 格，终点 (%.2f, %.2f) 离出发点 %.2f m，"
                "最后目标 %s (%.2f, %.2f)，倒车脱困 %d 次，卡死重启 %d 次，规划失败 %d 次%s"
                % (reason, dist, world.frames, int((world.grid() >= 0.65).sum()),
                   p[0], p[1], math.hypot(p[0] - x0, p[1] - y0), pilot.src or "-",
                   gx, gy, pilot.recovers, pilot.stalls, pilot.noreplan,
                   ("，地图已存 " + out_dir) if out_dir else ""))

    def spin_map(self, deg, out_dir=""):
        """Turn on the spot and fold every scan into an occupancy grid.

        The blind wedge is fixed to the body, so one rotation moves every wall out of it and a
        car that never translates can still see the whole room. The result is drawn in the
        odometry frame, which is exactly what makes it a pose check: the heading arrow says
        which way the board *thinks* it is facing.
        """
        with self.lock:
            if self.pose is None:
                return "失败: 没有收到 %s 数据" % ODOM_TOPIC
            start_pose = (self.pose[0], self.pose[1], math.degrees(self.pose[2]))
            start_yaw_rad = self.pose[2]
        age = self.scan_age()
        if age is None or age > 1.5:
            return "失败: /scan 无数据（%.1f 秒前）" % (-1.0 if age is None else age)
        rate = 0.35 if deg >= 0 else -0.35
        if abs(deg) < 1e-6:
            rate = 0.0            # deg 0 = fold a few seconds of scans without moving: a spin
            turn_s = 3.0          # smears the map into arcs here, because the odom yaw lags
        else:                     # the physical rotation, so a stationary fold is the clean one
            turn_s = abs(math.radians(deg)) / abs(rate)
        self.nav_stop = False
        self.abort_reason = ""
        self.stop_src = ""
        self.acting = True            # same claim as the tour: MCP MapSpin owns the wheels without
                                      # taking the console's busy flag, and the freeze logic decides
                                      # whether a human has the stick by reading this
        world = nav.World()
        travelled = [start_pose[:2]]
        t0 = time.monotonic()
        last_fold = t0
        reason = "转完"
        turned = 0.0
        prev_yaw = start_yaw_rad
        # Same stall watchdog as a Move: a spin that the wheels cannot deliver used to burn its
        # whole time budget and then report the shortfall after the fact.
        ref_t, ref_turned = t0, 0.0
        gyro_int, gyro_prev = 0.0, None
        flips, notes = 0, []
        try:
            while time.monotonic() - t0 < turn_s + 4.0 and rclpy.ok():
                if self.abort_reason:
                    reason = self.abort_reason
                    break
                if self.nav_stop:
                    reason = ("机器判定中止（未写明原因）" if self.stop_src == "machine"
                              else "收到停止指令")
                    break
                now = time.monotonic()
                pose, pairs = self.scan_sample()
                if pose is None:
                    reason = "里程计断了"
                    break
                turned += math.atan2(math.sin(pose[2] - prev_yaw),
                                     math.cos(pose[2] - prev_yaw))
                prev_yaw = pose[2]
                if now - last_fold > 0.15 and pairs:
                    world.integrate(pose, pairs)
                    travelled.append((pose[0], pose[1]))
                    last_fold = now
                self.drive(0.0, rate)
                time.sleep(CMD_PERIOD)
                now = time.monotonic()
                with self.lock:
                    im = self.imu
                if im is not None and now - im[1] < 0.5:
                    if gyro_prev is not None:
                        gyro_int += abs(im[0]) * max(0.0, min(0.3, im[1] - gyro_prev))
                    gyro_prev = im[1]
                if abs(turned - ref_turned) > STALL_MIN_W:
                    ref_t, ref_turned, gyro_int, gyro_prev = now, turned, 0.0, None
                    continue
                win = now - ref_t
                # the stall test is about magnitude, but which half of the gains to ask about is
                # decided by the sign the car is actually turning on
                exp = abs(nav.achieved(0.0, rate)[1])
                if win < STALL_SECS or now - t0 < STALL_GRACE or exp <= 0.0:
                    continue
                if stalled(exp, abs(turned - ref_turned) / win, win, "angular"):
                    label, parts = self._why_stalled("angular", 1.0 if rate > 0 else -1.0,
                                                     gyro_int, win)
                    if flips == 0 and label != "slipping":
                        # the user's own rule: if twisting one way is hung, twist back first
                        flips += 1
                        rate = -rate
                        notes.append("转不动（%s），已反向继续转" % "；".join(parts))
                        ref_t, ref_turned, gyro_int, gyro_prev = now, turned, 0.0, None
                        continue
                    reason = "轮子在转但车身没转（命令 %.0f°/s，实测 %.0f°/s，%.1f s）：%s" % (
                        math.degrees(exp), math.degrees(abs(turned - ref_turned) / win), win,
                        "；".join(parts))
                    break
        finally:
            self.acting = False
            self.stop_stream(0.6)
        if out_dir:
            nav.save_world(world, out_dir, travelled)
        with self.lock:
            p = self.pose
        return ("%s：指令 %.2f 圈、实际转了 %+.1f°，建图 %d 帧/占据 %d 格，出发点 odom "
                "(%.2f, %.2f) 朝 %.0f°，现在 (%.2f, %.2f) 朝 %.0f°%s%s"
                % (reason, abs(math.radians(deg)) / (2 * math.pi), math.degrees(turned),
                   world.frames, int((world.grid() >= 0.65).sum()),
                   start_pose[0], start_pose[1], start_pose[2],
                   p[0], p[1], math.degrees(p[2]),
                   ("，地图已存 " + out_dir) if out_dir else "",
                   ("；" + "，".join(notes)) if notes else ""))

    def rate_test(self, vx, wz, seconds):
        """Stream one fixed twist open-loop and report what the odometry actually did.

        The planner's MAX_V/MAX_W are an assumption about the chassis, and a 360-degree
        MapSpin commanded at 0.35 rad/s turned the car only 67 degrees in 18 s. Planning arcs
        the car cannot follow is how a good mechanism still drives badly, so the envelope gets
        measured instead of guessed.
        """
        with self.lock:
            if self.pose is None:
                return "失败: 没有收到 %s 数据" % ODOM_TOPIC
            x0, y0, yaw0 = self.pose[0], self.pose[1], self.pose[2]
        self.last_calib = None
        t0 = time.monotonic()
        n = 0
        dyaw = 0.0
        gyro_dyaw = 0.0
        gyro_n = 0
        with self.lock:
            prev = self.pose[2]
            prev_imu = None if self.imu is None else (self.imu[0], self.imu[1])
        con = getattr(self, "console", None)
        was_mapping = con.mapping if con else None
        if con:
            # Folding a scan during an open-loop sweep is the worst case for the map: the turn is
            # exactly where the odometry's yaw is known to be wrong (overshoot, coast), and every
            # degree of that error pastes a wall somewhere it never was. Two 6.5 s bursts at
            # 0.35 rad/s grew the saved grid from 615 occupied cells to 2066, with a fan of ghost
            # walls around the car, and the map still "matched" at 98% -- self-consistent wrong.
            con.mapping = False
        try:
            while time.monotonic() - t0 < seconds and rclpy.ok():
                self.drive(vx, wz)
                n += 1
                time.sleep(CMD_PERIOD)
                with self.lock:
                    cur = self.pose[2]
                    imu = None if self.imu is None else (self.imu[0], self.imu[1])
                # accumulate stepwise: one atan2 of the endpoints wraps, so a 226-degree turn
                # reports as -133.6 and the whole calibration looks like a sign error
                dyaw += math.atan2(math.sin(cur - prev), math.cos(cur - prev))
                prev = cur
                if imu and prev_imu:
                    span = imu[1] - prev_imu[1]
                    if 0.0 < span < 0.5:
                        gyro_dyaw += 0.5 * (prev_imu[0] + imu[0]) * span
                        gyro_n += 1
                if imu:
                    prev_imu = imu
        finally:
            self.stop_stream(0.5)
            if con:
                con.mapping = was_mapping
        dt = time.monotonic() - t0
        with self.lock:
            x1, y1 = self.pose[0], self.pose[1]
        dx, dy = x1 - x0, y1 - y0
        along = dx * math.cos(yaw0) + dy * math.sin(yaw0)
        side = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
        self.last_calib = {"v": vx, "w": wz, "v_act": along / dt, "w_act": dyaw / dt, "dt": dt,
                           "gyro_w": gyro_dyaw / dt, "gyro_n": gyro_n}
        return ("指令 vx=%.2f wz=%.2f 持续 %.1f s（%d 帧）→ 实际 前进 %.3f m、横移 %.3f m、"
                "里程计转向 %+.1f°；实测线速度 %.3f m/s（指令的 %.0f%%）、角速度 %+.3f rad/s = "
                "%+.1f°/s（指令的 %.0f%%）；陀螺仪 %+.1f°/s（%d 帧）"
                % (vx, wz, dt, n, along, side, math.degrees(dyaw),
                   along / dt, 100.0 * (along / dt) / vx if vx else 0.0,
                   dyaw / dt, math.degrees(dyaw) / dt,
                   100.0 * (dyaw / dt) / wz if wz else 0.0,
                   math.degrees(gyro_dyaw / dt) if gyro_n else 0.0, gyro_n))

    def _net_progress(self, x0, y0, yaw0, kind, sign):
        """Signed displacement from the pose a move started at, in the commanded direction."""
        with self.lock:
            cur = self.pose
        if cur is None:
            return 0.0
        x, y, yaw = cur[0], cur[1], cur[2]
        if kind == "linear":
            # project on the heading the move started on: lateral drift is not progress
            return sign * ((x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0))
        return sign * wrap_pi(yaw - yaw0)

    def _wait_settle(self, x0, y0, yaw0, kind, sign, budget=2.0):
        """Poll until the pose stops changing, then report net progress.

        The board keeps a motion queue of its own, so it coasts a near-constant 5 degrees
        past the angle the controller stopped commanding. Reporting the hand-off point made
        Move claim 15 degrees for a turn that ended at 20.
        """
        last = self._net_progress(x0, y0, yaw0, kind, sign)
        stop = 0.002 if kind == "linear" else math.radians(0.3)
        deadline = time.monotonic() + budget
        while time.monotonic() < deadline:
            time.sleep(0.25)
            now = self._net_progress(x0, y0, yaw0, kind, sign)
            if abs(now - last) < stop:
                return now
            last = now
        return last

    def _odom_lie_deg(self, yaw0, imu0):
        """Degrees by which odom's heading has left the gyro's since the move began, or None.

        Both are integrators, but only one of them integrates what the wheels turned. Measured
        2026-10-19 against a book at the drive wheel: the car never turned, the gyro read 8 deg of
        wander, odom reported 40 -- and the map paid for it, because the fold loop trusts odom.
        """
        with self.lock:
            cur, im = self.pose, self.imu
        if cur is None or imu0 is None or im is None or time.monotonic() - im[1] > 0.5:
            return None
        odom_d = wrap_pi(cur[2] - yaw0)
        imu_d = wrap_pi(yaw_of_tuple(im[2]) - imu0)
        return abs(math.degrees(wrap_pi(odom_d - imu_d)))

    def _hold_w(self, yaw0, imu0):
        """The wz that keeps a straight move on the heading it started at, plus its error.

        `_net_progress` projects displacement onto yaw0, so a heading that has wandered is not
        billed as progress: the car arcs, the reported metres shrink and -- the reason this is
        not cosmetic -- the live scan gets folded at a heading the wheels no longer have.
        """
        with self.lock:
            cur, im = self.pose, self.imu
        if cur is None:
            return 0.0, 0.0
        # Steer on the IMU, not on wheel odometry: measured 2026-10-18, a car that skidded 20 deg
        # -- which the gyro *and* the lidar scene both read as 20 -- reported 3.8 deg of yaw,
        # because odom's heading is encoder-derived and encoders only count what the wheels turn.
        # A hold that reads that number is blind in exactly the episode it exists to fix.
        if im is not None and imu0 is not None and time.monotonic() - im[1] < 1.0:
            err = wrap_pi(yaw_of_tuple(im[2]) - imu0)
        else:
            err = wrap_pi(cur[2] - yaw0)
        if abs(err) <= HOLD_DEAD:
            return 0.0, abs(err)
        floor = max(HOLD_MIN, nav.turn_floor())
        if floor > HOLD_MAX:
            # The whole band the chassis obeys is now wider than a hold is allowed to be: a
            # stronger correction would fight the straight line it is supposed to keep, and a
            # weaker one is a command to stand still. Say nothing rather than pretend.
            return 0.0, abs(err)
        mag = min(HOLD_MAX, max(floor, HOLD_KP * abs(err)))
        return (-mag if err > 0.0 else mag), abs(err)

    def _coast_lead(self, need):
        """How much further this turn will slide after the loop lets go, in radians.

        The board is not braked by the last command, it is braked by the zero stream that follows
        it, and that takes a fixed fraction of a second -- so the distance to release early is the
        rate times that fraction. `need` only bounds the guess: letting a lead ever reach halfway
        across the command would turn "short by a degree" into "short by half the turn" on the
        small angles the planner uses.
        """
        with self.lock:
            im = self.imu
        if im is None or time.monotonic() - im[1] > 0.5:
            return 0.0                 # no fresh gyro: fall back to releasing at the target
        return min(abs(im[0]) * COAST_TAU, 0.5 * need)

    def _fuse(self, pose, pairs):
        """A costmap under the car: the console's growing map, with the live ring pasted on top.

        Same two-layer trick Pilot uses, and the reason a stall verdict can say more than "the
        lidar sees something": a return 0.2 m out that the map already drew as a wall is a blockage,
        while a direction with no return *and* no map is the gimbal blind wedge, not open floor.
        Returns None when there is no map to consult (--web-port 0), and callers must then fall
        back to the lidar alone rather than refuse to answer.
        """
        con = getattr(self, "console", None)
        world = getattr(con, "world", None) if con is not None else None
        if world is None:
            return None      # no map to consult: say so instead of reporting an empty one as clear
        try:
            cm = nav.Costmap(world.grid(), world.res, world.ox, world.oy)
            cm.rebuild(pose, pairs)
        except Exception:
            return None
        return cm

    def _map_ahead(self, cm, pose, sign, reach=0.9):
        """Metres to the first cell the map calls blocked along this heading, None if unseen."""
        if cm is None:
            return None
        x, y, yaw = pose[0], pose[1], pose[2]
        for t in [i * 0.05 for i in range(2, int(reach / 0.05) + 1)]:
            d = t if sign > 0 else -t
            if cm.at(x + d * math.cos(yaw), y + d * math.sin(yaw)) >= nav.COST_BLOCK:
                return t
        return None

    def _lidar_gap(self, bins=8, cap=1.5):
        """Body-frame bearing to the widest of a few wedges using the live ring alone.

        The fallback when there is no map to consult. `window_min` returns None for a wedge with
        no echo, which is *unknown*, not open -- the nose wedge is the gimbal's own blind spot --
        so an unreadable direction scores last instead of winning.
        """
        best, best_d = 0.0, -2.0
        for i in range(bins):
            b = 360.0 * i / bins
            d = self.window_min(b if b <= 180.0 else b - 360.0, 40.0)
            d = -1.0 if d is None else min(d, cap)
            if d > best_d:
                best_d, best = d, b
        return signed_deg(best), best_d

    def _gap(self, pose, pairs):
        """(bearing -180..180°, how it was found, the costmap or None) for escape manoeuvres."""
        cm = self._fuse(pose, pairs)
        if cm is not None:
            return signed_deg(nav.open_bearing(cm, pose)), "地图+雷达", cm
        off, _ = self._lidar_gap()
        return off, "只有雷达（地图不可用）", None

    def _why_stalled(self, kind, sign, gyro_rad, secs):
        """Name the blockage out of three senses, and pick which manoeuvre it argues for.

        "blocked" means at least one sense can point at the thing and none contradicts it.
        "slipping" is the honest answer when the wheels are driven, the body is not going anywhere
        and nothing is in front -- that is a cable, a rug edge or a wheel off the floor, and no
        amount of steering fixes it, so the caller should back off once and then tell a human.
        """
        pose, pairs = self.scan_sample()
        parts = []
        if pose is None:
            return "unknown", ["里程计没有数据，无法判断受阻原因"]
        cm = self._fuse(pose, pairs)
        lidar = self.window_min(0.0 if sign > 0 else 180.0, 40.0)
        wall = self._map_ahead(cm, pose, sign)
        name = "转向" if kind != "linear" else ("行进方向" if sign > 0 else "倒退方向")
        # sense 1: the live sweep
        if lidar is None:
            parts.append("雷达在%s没测到回波（正前 ±22° 是云台盲区，细杆与矮障测不到）" % name)
        else:
            parts.append("雷达%s最近 %.2f m" % (name, lidar))
        # sense 2: the map the car drew
        if cm is None:
            parts.append("地图不可用（控制台未启动），只有雷达一个依据")
        elif wall is None:
            parts.append("地图上 %s 0.9 m 内没有墙" % name)
        else:
            parts.append("地图上 %s %.2f m 处画着障碍" % (name, wall))
        # sense 3: is the body even rotating? A car pressed nose-first into a wall does not turn;
        # a wheel that has lost grip (or caught the power cable) does, and the gyro sees it while
        # the encoder-derived progress says nothing moved.
        deg = math.degrees(gyro_rad)
        spinning = deg > 6.0
        if spinning:
            parts.append("陀螺说这 %.1f s 里车身转了 %.0f°，是轮胎打滑或被线拽住，不是被墙顶住" % (secs, deg))
        blocked = ((lidar is not None and lidar < 0.35) or wall is not None) and not spinning
        label = "blocked" if blocked else ("slipping" if spinning else "unknown")
        if blocked:
            parts.append("判定：被挡住")
        elif spinning:
            parts.append("判定：轮胎打滑或被线拽住（车身在动，位移没有）")
        else:
            parts.append("判定：说不清。轮子在转、车没动，可雷达和地图都没看见东西，陀螺也没记录到"
                         "打滑。这时候先怀疑两件事：一是【USB 手柄插在控板上且摇杆有输入】——固件里"
                         "手柄优先级高于 /cmd_vel，话题会被整段忽略，表现就是「什么都正常但车不动」；"
                         "二是【电池低于 ~10.3 V 的低压锁】，固件会让车走一半停住。GetStatus 能看电压")
        return label, parts

    def _burst(self, v, w, secs):
        """Stream one twist for `secs` and report what actually moved: m, °(odom), °(gyro).

        Used for the manoeuvres in between, where the point is to escape, not to arrive, so the
        caller only needs to know whether the car obeyed at all.
        """
        with self.lock:
            p0 = self.pose
        if p0 is None:
            return 0.0, 0.0, 0.0
        x0, y0, yaw0 = p0[0], p0[1], p0[2]
        t0 = time.monotonic()
        gyro, prev = 0.0, None
        self.acting = True
        try:
            while time.monotonic() - t0 < secs and rclpy.ok():
                if self.nav_stop or self.abort_reason:
                    break
                with self.lock:
                    cur, im = self.pose, self.imu
                if cur is None:
                    break
                if im is not None and time.monotonic() - im[1] < 0.5:
                    if prev is not None:
                        gyro += abs(im[0]) * max(0.0, min(0.3, im[1] - prev))
                    prev = im[1]
                self.drive(v, w)
                time.sleep(CMD_PERIOD)
        finally:
            # whatever ends this burst -- deadline, abort, or an exception on the way through
            # the costmap -- the last twist on the wire has to be zero
            self.stop_stream(0.3)
        with self.lock:
            cur = self.pose
        if cur is None:
            return 0.0, 0.0, math.degrees(gyro)
        return (math.hypot(cur[0] - x0, cur[1] - y0),
                math.degrees(abs(wrap_pi(cur[2] - yaw0))), math.degrees(gyro))

    def _escape(self, kind, sign, attempt, speed, label):
        """One bounded manoeuvre, chosen by what the blockage looks like. Returns a sentence.

        The budget is deliberately tiny -- at most two of these and 0.14 m per Move -- because a
        car that keeps trying is what the operator called "顶住了墙还在拼命走". Anything past the
        second try is a decision about the room and belongs to the model or a human, not to a
        reflex inside the driver.
        """
        pose, pairs = self.scan_sample()
        if kind == "angular":
            if attempt == 1 and label != "slipping":
                # a turn that will not turn is usually hung on something -- a chair leg, the power
                # cable, the edge of a rug -- and the same twist the other way is what unwinds it.
                ok, msg = self._closed_loop("angular", -sign * math.radians(20.0), speed, escape=False)
                return ("反向转 20° %s（%s）" % ("成功" if ok else "失败", msg)), 0.0
            ok, msg = self._closed_loop("linear", -ESCAPE_BACK * 0.7,
                                        self.args.linear_speed, escape=False)
            return ("松开顶着的劲儿倒 %.2f m %s（%s）" % (
                ESCAPE_BACK * 0.7, "成功" if ok else "失败", msg)), ESCAPE_BACK * 0.7
        if attempt == 1:
            astern = self.window_min(180.0, 40.0)
            if astern is not None and astern < self.args.safe_distance:
                # backing into the thing behind is not escaping; go straight to re-aiming
                return "背后 %.2f m 也有东西，不敢倒退，改为朝缺口转向" % astern, 0.0
            ok, msg = self._closed_loop("linear", -ESCAPE_BACK,
                                        self.args.linear_speed, escape=False)
            return ("倒退 %.2f m %s（%s）" % (
                ESCAPE_BACK, "成功" if ok else "失败", msg)), ESCAPE_BACK
        # second try: point at the widest gap so the retry approaches a different corridor
        off, src, _ = self._gap(pose, pairs)
        rate = math.copysign(min(0.7, 0.6 * self.args.angular_speed), off)
        _, deg, gdeg = self._burst(0.0, rate, ESCAPE_SECS)
        return ("朝最空旷缺口（%s，车体系 %+.0f°）转了 %.0f°，陀螺 %.0f°" % (src, off, deg, gdeg)), 0.0

    def _closed_loop(self, kind, target, speed, escape=True):
        with self.lock:
            if self.pose is None:
                self.last_error = "没有收到 %s 数据" % ODOM_TOPIC
                return False, self.last_error
            x0, y0, yaw0, t0 = self.pose
            im = self.imu
        # The heading this move must keep is where the gyro says the car pointed when it began,
        # not where the wheel encoders say -- see _hold_w.
        imu0 = None
        if im is not None and time.monotonic() - im[1] < 1.0:
            imu0 = yaw_of_tuple(im[2])
        if t0 is not None and time.monotonic() - t0 > 3.0:
            self.last_error = "%s 已停滞 %.1f 秒，检查车是否在线" % (
                ODOM_TOPIC, time.monotonic() - t0)
            return False, self.last_error

        sign = 1.0 if target > 0 else -1.0
        need = abs(target)
        # Progress is the net displacement from the pose this move started at, not a running
        # sum of per-poll |delta|. A sum of absolute steps is a total variation, so the servo
        # overshoot and its correction both get billed: measured here, 24 turn_left 15
        # commands that reported 367 degrees between them had physically turned 164.
        travelled = 0.0
        emax = 0.0              # worst heading error this move had to pull back
        deadline = time.monotonic() + self.args.move_timeout
        ok = False
        # Stall watchdog state: the progress and the gyro angle as of the start of the current
        # observation window. Re-based every time the car makes progress, after an escape attempt,
        # and never during the grace period a launch needs (the board queues ~0.2-0.3 s of twist).
        rate_t0 = t_start = time.monotonic()
        rate_ref = 0.0
        gyro_int = 0.0          # rad of body rotation since rate_t0, from the gyro
        gyro_prev = None
        tries = 0
        extra_m = 0.0
        notes = []
        # Slip witness: the ring distance to the wall the car is moving *away from*. Wheel odometry
        # used to be the watchdog's only progress meter, and the odometer is precisely what lies
        # while a wheel is slipping, so progress now has to be agreed by something that reads the
        # room instead of the motors. Either side missing means fall back to odom completely.
        # Forward only, measured 2026-10-19: a 0.30 m reverse the map and the relocation score both
        # confirm as real got denied 0.19 m of it, because straight ahead the gimbal swallows the
        # 0..20 deg of sweep (BLIND_WEDGE) and the *nearest* return in an 80 deg fan can be swapped
        # out by a chair corner as the car moves. Behind the car there is no wedge and the wall
        # stays the same object, which is what makes the ruler usable at all.
        ring_watch = kind == "linear" and sign > 0
        away = 180.0
        ring_ref = self.window_min(away, 40.0) if ring_watch else None
        lie_max = 0.0           # deg of odom-vs-gyro heading disagreement this move reached
        denied = 0.0            # m of odom progress the ring refused, closed out window by window
        denied_now = 0.0        # ... including the window still running when the move ends
        self.acting = True
        # A fresh command gets a clean bill of health: without this, one mid-motion relocation
        # verdict would abort every later Move until the process restarted. An escape manoeuvre
        # nested inside a Move must NOT do this -- clearing these would eat a human's 急停 that
        # arrived while the car was backing off.
        if escape:
            self.abort_reason = ""
            self.nav_stop = False   # likewise: a Stop() that landed before this command starts
            self.stop_src = ""
        try:
            while time.monotonic() < deadline and rclpy.ok():
                if self.abort_reason:
                    self.last_error = self.abort_reason
                    return False, self.last_error
                travelled = self._net_progress(x0, y0, yaw0, kind, sign)
                lie = self._odom_lie_deg(yaw0, imu0)
                if lie is not None and lie > SLIP_YAW_DEG:
                    # Sticky on purpose: _hold_vote reads the timestamp, so one honest map frame
                    # is worth more than a hundred folded at an invented heading.
                    lie_max = max(lie_max, lie)
                    self.yaw_lie = lie
                    self.yaw_lie_t = time.monotonic()
                # Without this the console's 急停 only fights the servo for 0.6 s: the loop
                # below re-commands velocity every tick, so a human could not interrupt a Move
                # even though the machine's own relocation abort can.
                if self.nav_stop:
                    if self.stop_src == "machine":
                        # still stop the wheels -- but do not blame a hand that was never there
                        self.last_error = "机器判定中止（未写明原因，已完成 %.2f/%.2f）" % (
                            travelled, need)
                    else:
                        self.last_error = "收到停止指令（已完成 %.2f/%.2f）" % (travelled, need)
                    return False, self.last_error
                err = need - travelled
                if kind == "angular":
                    err -= self._coast_lead(need)
                # the tolerance is coarse on purpose: the loop samples at 20 Hz and a 0.79 rad/s
                # turn covers 1.4 deg between two samples, so anything tighter than ~1 deg of
                # release point is not a control decision, it is sampling noise
                if err <= max(0.02, 0.04 * need):
                    ok = True
                    break
                if kind == "linear" and self.args.safe_distance > 0:
                    clear = self.window_min(0.0 if sign > 0 else 180.0, 40.0)
                    if clear is not None and clear < self.args.safe_distance:
                        self.last_error = "行进方向 %.2f m 内有障碍，已中止（完成 %.2f/%.2f）" % (
                            clear, travelled, need)
                        return False, self.last_error
                v = sign * tail_cmd(kind, speed, err, need)
                if kind == "linear":
                    w, herr = self._hold_w(yaw0, imu0)
                    emax = max(emax, herr)
                    self.drive(v, w)
                else:
                    w = v
                    self.drive(0.0, v)
                time.sleep(CMD_PERIOD)
                # ---- stall watchdog: is the chassis obeying, and is the car going anywhere?
                now = time.monotonic()
                with self.lock:
                    im = self.imu
                if im is not None and now - im[1] < 0.5:
                    if gyro_prev is not None:
                        gyro_int += abs(im[0]) * max(0.0, min(0.3, im[1] - gyro_prev))
                    gyro_prev = im[1]
                step_m = STALL_MIN_M if kind == "linear" else STALL_MIN_W
                claim = travelled - rate_ref
                ver = claim
                if ring_watch and claim > 2 * step_m and ring_ref is not None:
                    away_now = self.window_min(away, 40.0)
                    if away_now is not None:
                        seen = away_now - ring_ref
                        if seen < SLIP_FRAC * claim and claim - max(seen, 0.0) > 0.05:
                            # The wall behind has not receded, so the wheels turned without the car
                            # going anywhere. Bill the room, not the encoders. The absolute floor is
                            # the lidar's own noise: a 2 cm lag on a short claim is not a diagnosis.
                            ver = max(seen, 0.0)
                            self.slip_t = now
                denied_now = denied + max(0.0, claim - ver)
                if denied_now > denied:
                    self.slip_m = denied_now
                # A claim between one and two steps is too small for the room to arbitrate, but it
                # is still enough for the odometer to bank -- and banking it is what used to reset
                # the clock before the ring was ever asked, so the slip witness could never engage
                # (found by _stall_test.py's slipping case). Hold the window open until the claim is
                # big enough to judge; real progress reaches it in a few ticks.
                if ver > step_m and not (ring_watch and ring_ref is not None
                                         and step_m < claim <= 2 * step_m):
                    denied = denied_now
                    self.slip_m = denied
                    rate_t0, rate_ref, gyro_int, gyro_prev = now, travelled, 0.0, None
                    ring_ref = self.window_min(away, 40.0) if ring_watch else None
                    continue
                win = now - rate_t0
                if win < STALL_SECS or now - t_start < STALL_GRACE:
                    continue
                # sign-aware: left and right have their own gains, and the watchdog has to
                # compare the car against the rate this command should have bought
                exp_v, exp_w = nav.achieved(v, w)
                want = abs(exp_v if kind == "linear" else exp_w)
                got = ver / win
                if stalled(want, got, win, kind):
                    label, parts = self._why_stalled(kind, sign, gyro_int, win)
                    if kind == "linear":
                        meas = "命令 %.2f m/s，实测 %.2f m/s，已经 %.1f s 没动" % (want, got, win)
                        if denied_now > 0.05:
                            # Say which of the two rulers was overruled: "the wheels turned and the
                            # room says they did not" is the diagnosis, not just the symptom.
                            meas += "，里程计多记 %.2f m 被身后的雷达驳回" % denied_now
                    else:
                        meas = "命令 %.0f°/s，实测 %.0f°/s，已经 %.1f s 没转" % (
                            math.degrees(want), math.degrees(got), win)
                    if not escape or tries >= ESCAPE_TRIES or extra_m >= ESCAPE_BACK:
                        # an escape manoeuvre nested in here never runs its own ladder -- one
                        # reflex per Move, the rest is the model's or a human's call
                        if tries:
                            self.last_error = "轮子在转但车没动（%s）：%s。已自动试了 %d 次脱困（%s）还是不通，" \
                                              "停下来等人工处理（电源线缠住？地毯边？轮子悬空？）" % (
                                                  meas, "；".join(parts), tries, "，".join(notes))
                        else:
                            self.last_error = "轮子在转但车没动（%s）：%s。停下来等人工处理" % (
                                meas, "；".join(parts))
                        return False, self.last_error
                    tries += 1
                    note, moved = self._escape(kind, sign, tries, speed, label)
                    notes.append("第%d次脱困：%s" % (tries, note))
                    extra_m += moved
                    # Re-base on where the escape left the car, not on the pre-escape reading:
                    # backing off 0.14 m makes the net progress go backwards, and the next window
                    # has to be measured from there.
                    travelled = self._net_progress(x0, y0, yaw0, kind, sign)
                    denied, self.slip_m = denied_now, denied_now
                    rate_t0, rate_ref, gyro_int, gyro_prev = time.monotonic(), travelled, 0.0, None
                    ring_ref = self.window_min(away, 40.0) if ring_watch else None
        finally:
            # Only the outermost command releases the claim on the motors: an escape manoeuvre
            # nested inside a Move must not hand the console's idle stream back mid-command, that
            # is exactly the zero-velocity braking fixed on 2026-10-18.
            if escape:
                self.acting = False
            self.stop_stream(0.4)
        if ok:
            # the board coasts a few degrees past the angle it stopped being commanded to
            # hold, so quote where the pose actually ended up, not where we let go
            travelled = self._wait_settle(x0, y0, yaw0, kind, sign)
        # A move the room disagrees with must never be quoted as a clean pass: this string is what
        # the model reads before it plans the next step, and "完成 0.39/0.40" is a lie it will
        # build on. Measured 2026-10-19 -- that is exactly how a 40 deg invented heading reached
        # the map and the operator.
        doubt = []
        if lie_max > SLIP_YAW_DEG:
            doubt.append("里程计朝向与陀螺差到 %.0f°，这段的朝向和位移都别当真（建图已跳过这些帧）" % lie_max)
        if denied_now > 0.05:
            doubt.append("轮子打滑：里程计多记了 %.2f m，雷达不认" % denied_now)
        doubt = ("；注意：" + "，".join(doubt)) if doubt else ""
        unit = "米" if kind == "linear" else "度"
        got = travelled if kind == "linear" else math.degrees(travelled)
        want = need if kind == "linear" else math.degrees(need)
        if ok:
            # the peak heading error is quoted because it is the number this loop used to not
            # have: a straight move that reported 0.39 m of a commanded 0.40 while arcing 7 deg
            # looked like a clean pass, and the map paid for it
            note = "" if kind != "linear" else "，航向峰差 %.1f°" % math.degrees(emax)
            did = ("；中途受阻 %d 次（%s）" % (tries, "，".join(notes))) if notes else ""
            return True, "已完成 %s %.2f %s（实测 %.2f %s%s）%s%s" % (
                kind, want, unit, got, unit, note, did, doubt)
        # same unit as the success path: this once read "1.50/1.57" for a 90 degree turn and
        # looked like a metres figure from a linear move. The average rate is quoted over the time
        # actually spent, and the stall watchdog did not fire -- so this is the slow-but-moving
        # case, and "被挡住" would have been a lie to hand the model.
        spent = max(0.1, time.monotonic() - t_start)
        self.last_error = "超时未完成（%.2f/%.2f %s，%.1f s 内平均 %.2f %s/s），没触发停滞判定说明车在动只是慢；" \
                          "已试脱困 %d 次%s%s。下一步建议：ScanRing 看环视、MapInfo 换目标，或 Stop 后人工检查" % (
                              got, want, unit, spent, got / spent, unit,
                              tries, ("（%s）" % "，".join(notes)) if notes else "", doubt)
        return False, self.last_error


def grab_jpeg(camera_ip, port, timeout):
    """Pull one frame out of the ESP32 camera's multipart MJPEG stream."""
    url = "http://%s:%d/stream" % (camera_ip, port)
    buf = bytearray()
    with urllib.request.urlopen(url, timeout=timeout) as r:
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = r.read(8192)
            if not chunk:
                break
            buf += chunk
            if len(buf) > 6 * 1024 * 1024:
                break
            start = buf.find(b"\xff\xd8")
            if start < 0:
                continue
            end = buf.find(b"\xff\xd9", start)
            if end > start:
                return bytes(buf[start:end + 2])
    return None


def build_server(car, args):
    mcp = McpServer("microros_v2_pc")
    cache = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cache")
    os.makedirs(cache, exist_ok=True)
    band = nav.arc_band(args.linear_speed)      # 写进工具描述，别让模型照一份写死的半径带规划

    @mcp.tool(description=(
        "Move the chassis by odometry feedback. direction: forward | backward | turn_left | "
        "turn_right | arc_left | arc_right. Linear moves use distance (metres), turns use "
        "angle (positive degrees). Arcs curve while they travel: distance is the metres "
        "travelled ALONG the curve and radius (metres) is how sharp it bends -- at this "
        "car's --linear-speed the only radii it can actually hold are %.2f to %.2f m, so "
        "start around 0.5 m, and one circle needs several calls because each is capped by the "
        "single-move distance limit. An arc aborts if the metres travelled and the angle swept "
        "stop agreeing, which is how it detects that it drifted straight instead of curving. "
        "Keep each call small: <=0.5 m or <=90 deg. "
        "Linear moves abort on their own when the lidar sees the corridor close in. "
        "If the wheels are driven but the car stops making progress for ~1.6 s it aborts early, "
        "says which sense (lidar / map / gyro) calls it 被挡住 versus 打滑或被线拽住, and tries one "
        "bounded escape -- back off when astern is clear, twist the other way for a turn -- before "
        "failing. The failure text names the diagnosis, what it tried, and the next move; read it "
        "instead of retrying the same command." % (band[0], band[1])))
    def Move(direction: str, distance: float = 0.0, angle: float = 0.0,
             radius: float = 0.0) -> str:
        d = (direction or "").strip().lower()
        if d in ("arc_left", "arc_right"):
            r = float(radius)
            if r <= 0.0:
                return ("失败: %s 还要带 radius（米，正数）。这一轮车能稳住的半径是 %.2f ~ %.2f m，"
                        "先按 %.2f m 试；distance 是沿弧线走的米数，不是弦长" % (
                            d, band[0], band[1], (band[0] + band[1]) / 2.0))
            ok, msg = car.arc(r, abs(float(distance)), 1.0 if d == "arc_left" else -1.0,
                              args.linear_speed)
            return ("成功: " if ok else "失败: ") + msg
        if d == "forward":
            ok, msg = car.move_linear(abs(float(distance)), args.linear_speed)
        elif d == "backward":
            ok, msg = car.move_linear(-abs(float(distance)), args.linear_speed)
        elif d == "turn_left":
            ok, msg = car.turn(abs(float(angle)), args.angular_speed)
        elif d == "turn_right":
            ok, msg = car.turn(-abs(float(angle)), args.angular_speed)
        else:
            return ("失败: direction 只能是 forward/backward/turn_left/turn_right/"
                    "arc_left/arc_right")
        return ("成功: " if ok else "失败: ") + msg

    @mcp.tool(description=(
        "Immediately stop the chassis and publish zero velocity. Also aborts a running Tour."))
    def Stop() -> str:
        car.nav_stop = True         # first, or the tour loop re-commands motion next tick
        car.stop_src = "human"      # this entry point is only ever a person (or the chat's
                                    # 急停 button); the machine stops through car_web.stop_all
        car.stop_stream(0.6)
        return "已停车"

    @mcp.tool(description=(
        "Get loose on purpose: reads the lidar AND the map in all four body directions, names the "
        "widest gap, backs off (only when astern is clear) and turns toward that gap. Call it after "
        "a Move says 轮子在转但车没动 or 受阻 and before you give up on a direction; it returns what "
        "the car now sees, so the next Move can be planned against that instead of against hope. "
        "Move already runs one bounded try of this by itself."))
    def Unstick(back: float = 0.14, aim: bool = True) -> str:
        with car.lock:
            if car.pose is None:
                return "失败: 没有收到 %s 数据" % ODOM_TOPIC
        pose, pairs = car.scan_sample()
        ring = {n: car.window_min(a, 40.0) for n, a in
                (("前", 0.0), ("左", 90.0), ("后", 180.0), ("右", -90.0))}
        seen = dict((k, v) for k, v in ring.items() if v is not None)
        out = ["四周：%s" % "，".join("%s %.2f m" % (k, v) for k, v in sorted(seen.items()))
               if seen else "四周都没有回波（盲区或空旷，不能当成没东西）"]
        off, src, _cm = car._gap(pose, pairs)
        out.append("最空旷缺口 %+.0f°（依据：%s）" % (off, src))
        metres = max(0.05, min(0.20, float(back)))
        astern = ring["后"]
        if astern is None or astern < metres + 0.10:
            out.append("背后只有 %s，不倒退（不敢倒进没测清的东西里）" % (
                "没测到回波" if astern is None else "%.2f m" % astern))
        else:
            ok, msg = car.move_linear(-metres, args.linear_speed)
            out.append("倒退 %.2f m：%s" % (metres, msg))
        if aim and abs(off) > 15.0:
            turn = max(-45.0, min(45.0, off))
            ok, msg = car.turn(turn, args.angular_speed)
            out.append("朝缺口转 %+.0f°：%s" % (turn, msg))
        pose2, pairs2 = car.scan_sample()
        off2, _src2, _ = car._gap(pose2, pairs2)
        out.append("现在最空旷缺口 %+.0f°" % off2)
        return "；".join(out) + "。下一步自己定：能走就 Move，还走不动就 Stop 后告诉用户现场情况。"

    @mcp.tool(description=(
        "Drive a counter-clockwise lap of the room, mapping it as it goes. "
        "Fuses the growing occupancy grid with the live lidar into a costmap, A*-plans toward "
        "the least-explored bearing, and scores candidate arcs (including reversing) so it "
        "turns and backs off instead of pushing into obstacles. Ends back at the start, on "
        "Stop(), or when the seconds budget runs out. Saves the map to out_dir if given."))
    def Tour(seconds: int = 180, out_dir: str = "") -> str:
        return car.drive_tour(float(seconds), out_dir)

    @mcp.tool(description=(
        "Spin in place (deg, + = counter-clockwise) and build an occupancy map of the room "
        "from the lidar while staying put. Rotating sweeps the camera-gimbal blind wedge, so "
        "every wall gets seen. The map is in the odometry frame and marks where the board "
        "believes it is and which way it faces. Check ScanRing/camera clearance first."))
    def MapSpin(deg: float = 360.0, out_dir: str = "") -> str:
        return car.spin_map(float(deg), out_dir)

    @mcp.tool(description=(
        "Open-loop chassis calibration: stream one fixed (vx m/s, wz rad/s) twist for "
        "seconds and report the odometry's real achieved speeds. Use to find the board's "
        "velocity ceiling before trusting a planner's speed limits."))
    def RateTest(vx: float = 0.0, wz: float = 0.0, seconds: float = 4.0) -> str:
        return car.rate_test(max(-0.25, min(0.25, float(vx))),
                             max(-1.2, min(1.2, float(wz))),
                             max(0.5, min(15.0, float(seconds))))

    @mcp.tool(description=(
        "Capture one frame from the car camera and save it on the PC. "
        "Returns the absolute image path."))
    def SeeWhat() -> str:
        if not args.camera_ip:
            return "失败: 未配置 --camera-ip"
        try:
            data = grab_jpeg(args.camera_ip, args.camera_port, args.camera_timeout)
        except Exception as exc:
            return "失败: 无法读取摄像头流 http://%s:%d/stream (%s)" % (
                args.camera_ip, args.camera_port, exc)
        if not data:
            return "失败: 流里没有解析到 JPEG 帧"
        path = os.path.join(cache, "see_%d.jpg" % int(time.time() * 1000))
        with open(path, "wb") as f:
            f.write(data)
        return "已拍摄，图像路径: %s (%d bytes)" % (path, len(data))

    @mcp.tool(description=(
        "Point the 2-DoF camera. yaw/pitch are relative degrees from the current view: "
        "yaw + turns right (-45..65), pitch + tilts up (-40..15)."))
    def AdjustCameraView(pitch: int = 0, yaw: int = 0) -> str:
        with car.lock:
            nyaw = min(PTZ_YAW_LIMIT[1], max(PTZ_YAW_LIMIT[0], car.ptz[0] + float(yaw)))
            npitch = min(PTZ_PITCH_LIMIT[1], max(PTZ_PITCH_LIMIT[0],
                                                 car.ptz[1] + float(pitch)))
            car.ptz = [nyaw, npitch]
        msg = Vector3()
        msg.x = PTZ_SERVO_PITCH_LEVEL - npitch
        msg.y = PTZ_SERVO_YAW_CENTRE - nyaw
        msg.z = 0.0
        for _ in range(5):
            car.ptz_pub.publish(msg)
            time.sleep(0.03)
        return "云台角度 -> yaw=%.1f, pitch=%.1f" % (nyaw, npitch)

    @mcp.tool(description=(
        "Read the lidar as a ring of equal bearing sectors (0°=正前, 逆时针为正). "
        "Each value is the nearest distance in metres; null means nothing usable was "
        "measured, which includes the ~44° wedge straight ahead that the camera gimbal "
        "occludes. Use CheckAhead before driving forward."))
    def ScanRing(sectors: int = 12) -> str:
        age = car.scan_age()
        if age is None or age > 3.0:
            return "失败: /scan 无数据（%.1f 秒前）。确认雷达上电、micro-ROS Agent 在线。" % (
                -1.0 if age is None else age)
        n = max(4, min(24, int(sectors)))
        ring = car.sector_ring(n)
        step = 360.0 / n
        parts = []
        for i, d in enumerate(ring):
            deg = i * step + step / 2.0
            parts.append("%s(%.0f°)=%s" % (dir_name(deg), wrap180(deg),
                                           "%.2f" % d if d is not None else "无"))
        return "雷达 %d 扇区 (每 %.0f°): %s" % (n, step, ", ".join(parts))

    @mcp.tool(description=(
        "Measure clearance straight ahead by yawing +-30 degrees and looking back down "
        "the corridor, which works around the gimbal blind wedge. Yaws back to the "
        "original heading. Does not move the wheels."))
    def CheckAhead() -> str:
        age = car.scan_age()
        if age is None or age > 3.0:
            return "失败: /scan 无数据，无法测前方"
        with car.lock:
            yaw0 = car.pose[2] if car.pose else None
        if yaw0 is None:
            return "失败: 没有 %s，无法回正" % ODOM_TOPIC
        seen = []
        for off in (30.0, -30.0):
            ok, msg = car.turn(off, args.angular_speed)
            if not ok:
                car.stop_stream(0.3)
                return "转向失败: %s（已测 %s）" % (msg, seen)
            d = car.window_min(-off, 20.0)
            seen.append(d)
        with car.lock:
            resid = math.degrees(wrap_pi(yaw0 - car.pose[2]))
        if abs(resid) > 3.0:
            car.turn(resid, args.angular_speed)
        got = [d for d in seen if d is not None]
        if not got:
            return "前方左右两个方向 8 m 内都没测到回波（可能确实空旷）"
        return "前方净空: 左视 %.2f m / 右视 %.2f m -> 取 %.2f m。注意正前方 ±22° 是云台遮挡的盲区，细杆状障碍仍可能测不到。" % (
            seen[0] if seen[0] is not None else -1,
            seen[1] if seen[1] is not None else -1, min(got))

    @mcp.tool(description="Return link health: odom age, PTZ angle, battery, nearest obstacle.")
    def GetStatus() -> str:
        with car.lock:
            pose = car.pose
            ptz = list(car.ptz)
            volts = car.volts
        if pose is None:
            return "底盘离线: 从未收到 %s。确认 micro-ROS Agent 已启动、车上电、同一网络。" % ODOM_TOPIC
        age = time.monotonic() - pose[3]
        parts = ["odom %.1f 秒前 (x=%.2f y=%.2f yaw=%.0f°)%s" % (
            age, pose[0], pose[1], math.degrees(pose[2]),
            "" if age < 2.0 else " [疑似断连]"),
            "云台 yaw=%.0f pitch=%.0f" % (ptz[0], ptz[1]),
            # below ~10.3 V the firmware locks the motors, so a long motion job needs this
            # number up front rather than explained afterwards
            "电池 %s V%s" % ("%.2f" % volts if volts else "无",
                            "" if not volts else ("（低压锁风险）" if volts < 10.6 else ""))]
        sage = car.scan_age()
        if sage is None or sage > 3.0:
            parts.append("雷达无数据")
        else:
            ring = car.sector_ring(12)
            vals = [d for d in ring if d is not None]
            parts.append("雷达 %.1f 秒前, 可见最近 %s m, 12 方向 %s" % (
                sage, "%.2f" % min(vals) if vals else "无", ring))
        return " | ".join(parts)

    @mcp.tool(description=(
        "The car's global self-impression: where it sits inside the map it built, how much of "
        "the map is still unmeasured and in which direction (nearest frontier plus a 12-sector "
        "histogram of the unknown), how well the lidar agrees with the claimed pose right now, "
        "what the drift suppressor has spent, **which places a human has asked it to remember "
        "(with distance and bearing -- this is where 'where is the sofa' is answered)**, and "
        "whether exploration is finished. Read-only and safe during a running Tour. Call this "
        "before deciding where to go next -- it answers 'what is left to see' and 'what do I "
        "know about this room' in a way ScanRing/CheckAhead (local lidar only) never can."))
    def MapInfo() -> str:
        con = getattr(car, "console", None)
        if con is None:
            return "全局地图活在网页控制台里，而它没启动（--web-port 0）：没有可查的图。"
        return con.map_view()

    @mcp.tool(description=(
        "Pin a word to where the car stands right now -- 'this is the sofa', '充电座', "
        "'客厅中间'. The label is stored in the map's own frame, so it survives a bridge "
        "restart and shows up in MapInfo (and as a numbered dot on the console map). "
        "Say it again with the same label to move it. It is deleted together with 清空地图, "
        "because a new map is a different frame. Read-only: it never touches the motors."))
    def Remember(label: str) -> str:
        con = getattr(car, "console", None)
        if con is None:
            return "控制台没启动，没有地方存这个标签。"
        return con.add_place(label)

    @mcp.tool(description=(
        "Drop a remembered label (exact name, or a 2+ character word inside it matches). "
        "Use it when a human says that note is wrong or no longer applies. Read-only."))
    def Forget(label: str) -> str:
        con = getattr(car, "console", None)
        if con is None:
            return "控制台没启动，本来也没记住任何东西。"
        return con.del_place(label)

    @mcp.tool(description="Play a beep on the buzzer for the given duration in ms.")
    def Beep(duration_ms: int = 200) -> str:
        n = UInt16()
        n.data = int(max(0, min(5000, duration_ms)))
        for _ in range(3):
            car.beep_pub.publish(n)
            time.sleep(0.02)
        return "蜂鸣 %d ms" % n.data

    return mcp


def main():
    p = argparse.ArgumentParser(description="MicroROS-V2 Windows ROS2->MCP bridge")
    p.add_argument("--ros-domain-id", type=int, default=40)
    p.add_argument("--camera-ip", default="", help="图传模块 IP，如 192.168.31.xxx")
    p.add_argument("--camera-port", type=int, default=81)
    p.add_argument("--camera-timeout", type=float, default=6.0)
    p.add_argument("--linear-speed", type=float, default=0.25)
    p.add_argument("--angular-speed", type=float, default=1.0)
    p.add_argument("--move-timeout", type=float, default=15.0)
    p.add_argument("--safe-distance", type=float, default=0.25,
                   help="abort a linear move when the corridor closes to under this")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--web-port", type=int, default=8010,
                   help="手动控制台端口（0 表示不启动）")
    args = p.parse_args()

    os.environ.setdefault("ROS_DOMAIN_ID", str(args.ros_domain_id))
    rclpy.init()
    car = Car(args)
    spin = threading.Thread(target=lambda: rclpy.spin(car), daemon=True)
    spin.start()

    mcp = build_server(car, args)
    if args.web_port:
        import car_web        # deferred: car_web imports this module for the PTZ constants
        car_web.load_gains()
        car_web.serve(car, args)
    try:
        mcp.serve(args.host, args.port)
    except KeyboardInterrupt:
        pass
    finally:
        car.stop_stream(0.3)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
