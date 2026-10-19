"""Manual control console for the MicroROS-V2 car, served from inside the bridge process.

A freshly created rclpy node receives nothing from this board, so every topic has to be read by
the one long-lived bridge. That is why this is a thread inside car_mcp_server.py talking to the
Car object directly, and not a separate app subscribing to /scan and /odom_raw.

Two things beyond a remote control are built in on purpose:

* Measurement. Every teleop burst is timed against what the odometry actually did, so commanded
  and achieved velocity get fitted from ordinary driving rather than from a dedicated test
  session. The first pass of this found the chassis delivers ~90% of a commanded twist and
  ignores turn commands below ~0.13 rad/s, which is what the planner had to learn.
* A watchdog. The browser tab, the WiFi link and the ESP32 can each die in the middle of a
  command, so motion requires a refresh every TELEOP_HOLD seconds or the stream goes to zero.
"""

import json
import math
import os
import threading
import time
import urllib.request
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

import nav
from car_mcp_server import (ODOM_TOPIC, PTZ_PITCH_LIMIT, PTZ_SERVO_PITCH_LEVEL,
                            PTZ_SERVO_YAW_CENTRE, PTZ_YAW_LIMIT, grab_jpeg)
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt16

HERE = os.path.dirname(os.path.abspath(__file__))
GAINS_FILE = os.path.join(HERE, "nav_gains.json")
FEEDBACK_FILE = os.path.join(HERE, "feedback_log.jsonl")
# One line per measured burst, so a restart between two calibration sessions does not throw
# away the long sweeps -- see Console._add_sample.
CALIB_FILE = os.path.join(HERE, "calib_samples.jsonl")

CAP_V = 0.25            # console limits, m/s and rad/s
CAP_W = 0.90
PLACE_MAX = 40          # remembered labels; beyond this Forget first -- the map_view line has
PLACE_LABEL_MAX = 24    # to stay readable for a 9B model, and a label longer than this is a note
TELEOP_HOLD = 0.35      # s a teleop command survives without a refresh
STREAM_DT = 0.05        # the board wants a continuous cmd_vel stream at this period

# Carry detector (see Console._watch). A person standing in front of a still car replaces maybe
# a third of the sweep with a new surface; a car that has been picked up replaces nearly all of
# it, and an offset search separates those two because the second one *matches again* somewhere
# else. The thresholds are set for that gap, not for sensitivity.
STILL_WINDOW = 2.0      # s of odometry history the "is it still?" question looks at
STILL_TICKS = 4         # samples it must hold first; the loop runs at ~0.6 s, so a 2 s window
                        # tops out at five -- asking for six would never let it answer
STILL_TOL = 0.02        # m of odom wander still counted as standing still
WANT_MARGIN = 0.25      # an offset must explain this much more of the sweep to be believed
BEST_FLOOR = 0.55       # ... and reach this absolute agreement
LOST_FLOOR = 0.30       # below this, no offset explains the sweep: the map itself is suspect
SUSPECT_TICKS = 3       # consecutive bad ~0.5 s ticks before acting

# Step 2: the detector found an offset -- re-register the car onto it instead of just stopping.
# Moving the pose is a far more destructive guess than freezing the map, so three separate bars
# guard it: the candidate must explain almost the whole sweep on its own (FLOOR), it must beat
# every *other* placement by a clear distance (AMBIG -- rectangular rooms happily fit a sweep to
# two parallel walls), and the next sweep must confirm it at the new pose (VERIFY). Fail any of
# the three and the car goes back to where it was and stops.
RELOC_FLOOR = 0.85      # a correction may be tried only if this much of the sweep lands on wall
RELOC_AMBIG = 0.12      # ... and no rival placement within the search may come this close
RELOC_VERIFY = 0.70     # after the trial, the sweep at the corrected pose must reach this
RELOC_TRIES = 2         # trial/confirm rounds before the machine gives up and asks a human
RELOC_TTL = 8.0         # s to wait for a confirming sweep before revoking the trial

# Moving-window drift suppression, see _drift_correct. Narrow by design: drift is centimetres,
# and a wide window here would let the probe "relocalise" into a different part of the room
# instead of measuring how far the odometry has slipped. every=3 instead of the carry check's 6
# doubles the support while the car rolls.
DRIFT_REACH = 0.6       # m
DRIFT_STEP = 0.05       # m
DRIFT_EVERY = 3         # lidar returns used per sweep
# The three gates below are the boundary between "the residual is a measurement" and "the
# residual is the argmax of a flat surface". Measured over 21 moving samples on the real car:
# the 12 sweeps that still matched the grid kept the peak within +-9 cm of the claimed pose with
# 8 mm scatter, and the 9 that did not flew to +-0.6 m and pinned themselves on the edge of the
# search window. Correcting on those would drive the car into noise.
DRIFT_BEST = 0.80       # a real peak must exist ...
DRIFT_ZERO = 0.75       # ... and the claimed pose must still explain most of the sweep
DRIFT_MAX = 0.25        # beyond this it is a relocation, which is _carry_check's job
# The residual is an estimate of *how far off the pose is right now*, not of error ever
# accumulated, so it is averaged (DRIFT_ALPHA), never summed: summing it wound a 1.33 m bank out of
# 2.1 m of driving on the real car (measured, live) and left the controller running at its rate cap
# for the rest of the tour. And the deadband does not have to sit above the estimator's scatter --
# an average kills scatter however tight the band is. What it has to sit above is the *correlated*
# part, because low-frequency wander is exactly what real drift looks like at these scales. Both
# numbers come from _drift_tune.py, which sweeps (alpha, deadband) against three signals at once: a
# +-9 cm alternating wobble (must pay out nothing), the real car's 21 moving samples (well
# registered, must stay near-silent), and a synthetic lap whose odometry gains 3 cm per metre (must
# engage and sharpen the map). 0.25/0.02 passes all three -- wobble 0 corrections, real log 2
# corrections totalling 1.7 cm, lap wall thickness 1.33 -> 1.00 cells with the pose error halved.
# 0.40 reacts sooner but starts paying out on the wobble; 0.15 never engages at all.
DRIFT_ALPHA = 0.25      # weight of one accepted sweep in the running estimate
DRIFT_DEAD = 0.02       # m of estimated error before anything moves
DRIFT_GAIN = 0.5        # damped: evidence must be seen to repeat before it steers the car
DRIFT_CAP = 0.05        # m one nudge may ever cover, however big the estimate got
DRIFT_MIN = 0.005       # m: below this a nudge is not worth a log line or a re-registered sweep
                        # There is no cooldown on purpose: spacing payouts out was measured
                        # (_drift_test.py, synthetic lap) and made the map *worse*, because the
                        # error grows and smears the folds while the corrector waits its turn.
DRIFT_RATE = 0.05       # m of correction per m driven -- odometry's own error budget (a lap
                        # closes ~3% short), so a constant matcher bias cannot walk the pose away
DRIFT_STALE = 5.0       # s a measurement may sit in the estimate before it is old news
DRIFT_TOTAL = 1.2       # m the pose may end up away from raw odometry before the suppressor
                        # concedes. The rate cap allows it to be reached in 24 m of driving at
                        # full tilt, an order of magnitude past a tour.

# Step 4: ask the carry detector's question *while the wheels are turning*. Until now every
# moving branch handed off to _drift_probe, whose entire design is centimetres (DRIFT_MAX=0.25,
# reach 0.6 m), so a car picked up mid-Move, or one whose odometry came apart in a slide on the
# tile, kept driving on a pose nobody believed until it happened to stop. Two things made that
# safe to change here rather than two steps ago: the matcher now reports `outside` (how much of
# the sweep lands on ground the map has never measured), without which "nothing matches" cannot
# be told apart from "the car is exploring", which is the same score and the opposite event; and
# the gates below are deliberately stricter than the stopped-car ones, because acting mid-motion
# pulls the motors out from under a running command.
MOTION_REACH = 1.6      # m: the carry detector's window, not the drift probe's
MOTION_STEP = 0.10      # m: coarse on purpose -- "is the world still where I claim it is"
MOTION_EVERY = 6        # lidar returns per sweep; half the drift probe's, and 1 ms not 6
MOTION_ROLL = 0.15      # m the car must cover since the last check to earn a new sample, so a
                        # slow crawl cannot count the same noisy sweep three times
MOTION_BAD = 2          # consecutive refusals before the motors are pulled
MOTION_FRESH = 0.35     # below this share of unmapped returns, "nothing fits" means the pose is
                        # wrong; above it the car is over new ground and gets left alone
MOTION_HOLD = 1.0       # s a suspicious frame may keep its sweep out of the map. A suspicion that
                        # never expires is a mapping freeze, and the detector only speaks while the
                        # wheels turn -- standing there, the honest sweep says so itself and clears
                        # the flag. This is the backstop for the case where it never gets asked.
# The other half of the question. scan_offset asks the lidar "does each return land on a wall";
# nothing there asks the map "is a wall you claim in the way actually there", and a phantom patch
# answers both questions badly: it drags `zero` down and offers the argmax a mass of cells to
# slide onto. nav.through_wall measures the rays that see straight through mapped walls, and a
# relocation must improve that number, not merely the score. Measured in _reloc_test.py: an
# honest pose on a correct map contradicts 0.9% of its own sweep; a real 0.8 m carry reads 42%
# where odometry claims and 0% at the recovered offset; and the false alarm this exists for -- a
# tour that ground itself into furniture and drew phantom walls -- read 47% claimed against 80%
# at the placement the matcher wanted. Note the *level* of `t0` is not the discriminator (a real
# carry and a bad map both sit near 45%); only the contrast is, so both numbers are kept.
THROUGH_BETTER = 0.15   # the candidate must cut the see-through share by at least this
# ... and the veto only gets to hold that verdict while the map can still answer the question.
# If even the matcher's *best* placement contradicts this share of the sweep, no pose in the
# window explains the lidar, so "it isn't a carry" is a claim the grid cannot support. Measured
# 2026-10-18 on a map smeared by the chassis' 34-43° uncommanded yaw: 47% of rays saw through a
# claimed wall at a pose the matcher itself rated 99.5%, and a real +0.80 m jump was excused as
# "地图画错了". Refusing to move the pose is still the right call there; letting the wheels keep
# turning on a map that cannot locate the car is the failure that was named from the outside.
# The value is a measured boundary, not taste: replaying one real 0.8 m carry against maps smeared
# by the chassis' own uncommanded yaw (_ghost_probe.py), 16 degrees of ghost walls leaves the
# recovered pose at 0.277 see-through -- the veto fires and the car drives on, wrong by 0.8 m --
# while 8 degrees of ghosting still improves it by 0.26 and aborts normally.
THROUGH_JUNK = 0.25
# The other end of the same argument, because a veto needs a contradiction to point at. "The
# candidate does not look through these walls any less than the pose we claim" is only evidence
# against relocating when the claimed pose *is* being contradicted: _thin_probe.py measured a young
# map's honest pose at 0.03 see-through, where "no improvement" is a vacuous statement and using it
# would excuse a real carry on a map that never accused anybody. THROUGH_ACCUSE is that floor -- set
# at the improvement margin it is meant to be compared against, so the two halves of the gate read
# as one question.
THROUGH_ACCUSE = THROUGH_BETTER


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# 3x5 bitmaps, because this box has neither PIL nor cv2 and the axes have to be legible in
# the PNG itself (a client-side overlay can only ever be aligned to a frame it did not draw).
FONT = {
    "0": (".#.", "#.#", "#.#", "#.#", ".#."),
    "1": ("..#", ".##", "..#", "..#", "..."),
    "2": ("#.#", "..#", ".#.", "#..", "###"),
    "3": ("###", "..#", ".##", "..#", "###"),
    "4": ("#.#", "#.#", "###", "..#", "..#"),
    "5": ("###", "#..", "##.", "..#", "###"),
    "6": (".##", "#..", "###", "#.#", "###"),
    "7": ("###", "..#", ".#.", ".#.", ".#."),
    "8": ("###", "#.#", "###", "#.#", "###"),
    "9": ("###", "#.#", "###", "..#", "##."),
    "-": ("...", "...", "###", "...", "..."),
    ".": ("...", "...", "...", "...", ".#."),
    ",": ("...", "...", ".#.", ".#.", "#.."),
    "m": ("...", "...", "#.#", "#.#", "#.#"),
    "O": ("###", "#.#", "#.#", "#.#", "###"),
    "/": ("...", "..#", ".#.", "#..", "..."),
    " ": ("...", "...", "...", "...", "..."),
}
AX_GRID = (30, 34, 38)        # darkened grid lines
AX_ZERO = (255, 92, 160)      # the x=0 / y=0 odom axes
AX_NUM = (255, 210, 60)       # tick numbers
AX_BG = (12, 15, 19)
# Remembered places. Not the old magenta (255,90,200): against AX_ZERO's pink (255,92,160) a dot
# and an axis line are the same colour to the eye, and the first live map showed exactly that
# confusion -- the pink cross looked like it belonged to the two notes pinned under the car.
PLACE_COL = (170, 70, 255)


def ax_text(img, x, y, s, col, sc=2):
    """Blit ASCII into an RGB uint8 array at pixel (x, y), scaling each 3x5 glyph by sc."""
    h, w = img.shape[:2]
    for ci, ch in enumerate(s):
        rows = FONT.get(ch, FONT[" "])
        for r, row in enumerate(rows):
            for c, on in enumerate(row):
                if on != "#":
                    continue
                x0, y0 = (x + (ci * 4 + c) * sc, y + r * sc)
                img[max(0, y0):min(h, y0 + sc), max(0, x0):min(w, x0 + sc)] = col


def ax_stripe(img, at, vertical, col, half_w=0, blend=None):
    """Paint one stripe at pixel `at` (a column if vertical, else a row), `blend` tallies mixing."""
    lo, hi = max(0, int(at) - half_w), int(at) + half_w + 1
    seg = img[:, lo:hi] if vertical else img[lo:hi, :]
    if blend is None:
        seg[:] = col
    else:
        seg[:] = (seg * (1.0 - blend) + np.array(col, np.float32) * blend).astype(np.uint8)


def draw_axes(up, ox, oy, res, c0, r1, pose, origin=(0.0, 0.0)):
    """Stamp the printed coordinate frame onto the finished picture: grid, x=0/y=0, ticks, origin.

    Drawn server-side rather than as a browser overlay because an overlay has to guess which
    world rectangle the bytes it is sitting on cover, and the geometry of one frame paired
    with the pixels of another is exactly what makes a coordinate axis untrustworthy.

    `origin` is the odom point the user re-anchored the display to, so every number here is
    printed in the shifted frame while the pixels underneath stay in the odom frame the
    occupancy grid was built in -- translating the labels, not resampling the map.
    """
    h, w = up.shape[:2]
    mpp = res / 3.0
    x_min, y_top = ox + c0 * res - origin[0], oy + r1 * res - origin[1]
    pxc = lambda v: int(round((v - x_min) / mpp))
    pyc = lambda v: int(round((y_top - v) / mpp))
    sc = 2 if max(w, h) < 760 else 3
    step = 20.0
    for cand in (0.5, 1.0, 2.0, 5.0, 10.0, 20.0):
        if cand / mpp >= 60.0:
            step = cand
            break
    num = lambda v: ("%d" % round(v)) if step >= 1.0 else ("%.1f" % v)

    def label(x, y, s, col=AX_NUM):
        x0, x1 = max(0, x - sc), min(w, x + len(s) * 4 * sc + sc)
        y0, y1 = max(0, y - sc), min(h, y + 5 * sc + 2 * sc)
        up[y0:y1, x0:x1] = AX_BG
        ax_text(up, x, y, s, col, sc)

    i = math.ceil(x_min / step)
    while i * step <= x_min + w * mpp:
        ax_stripe(up, pxc(i * step), True, AX_GRID, blend=0.45)
        i += 1
    i = math.ceil((y_top - h * mpp) / step)
    while i * step <= y_top:
        ax_stripe(up, pyc(i * step), False, AX_GRID, blend=0.45)
        i += 1
    if 0 <= pxc(0) < w:
        ax_stripe(up, pxc(0), True, AX_ZERO, half_w=1, blend=0.85)
    if 0 <= pyc(0) < h:
        ax_stripe(up, pyc(0), False, AX_ZERO, half_w=1, blend=0.85)

    i = math.ceil(x_min / step)
    while i * step <= x_min + w * mpp:
        label(pxc(i * step) + 2 * sc, h - 7 * sc, num(i * step))
        i += 1
    i = math.ceil((y_top - h * mpp) / step)
    while i * step <= y_top:
        label(2 * sc, pyc(i * step) - 3 * sc, num(i * step))
        i += 1

    ox_px, oy_px, d = pxc(0), pyc(0), math.hypot(pose[0], pose[1])
    if 0 <= ox_px < w and 0 <= oy_px < h:
        rad = 4 * sc
        for k in range(0, 360, 8):
            xx = ox_px + int(round(rad * math.cos(math.radians(k))))
            yy = oy_px + int(round(rad * math.sin(math.radians(k))))
            up[max(0, yy):yy + 2, max(0, xx):xx + 2] = AX_ZERO
        ax_stripe(up, ox_px, True, AX_ZERO, half_w=0)
        for dy in range(-2, 3):
            up[oy_px + dy, max(0, ox_px - 2 * sc):ox_px + 2 * sc] = AX_ZERO
        label(ox_px + 5 * sc, oy_px - 6 * sc, "0,0", AX_ZERO)
    else:
        m = 3 * sc
        bx = max(m, min(w - m - 1, ox_px))
        by = max(m, min(h - m - 1, oy_px))
        ang = math.atan2(oy_px - h / 2.0, ox_px - w / 2.0)
        for j in range(3 * sc):                        # a solid arrowhead pointing at (0,0)
            yy = int(round(by + j * math.sin(ang)))
            xx = int(round(bx + j * math.cos(ang)))
            span = (3 * sc - j) // 2
            up[max(0, yy - span):yy + span + 1, max(0, xx - span):xx + span + 1] = AX_ZERO
        label(max(2 * sc, min(w - 20 * sc, bx - 6 * sc)),
              max(2 * sc, min(h - 8 * sc, by + 4 * sc)), "0,0 %.1fm" % d, AX_ZERO)
    label(w // 2 - 8 * sc, 2 * sc, "%.1fm" % step if step < 1 else "%dm" % step)


class Console:
    def __init__(self, car, args):
        self.car = car
        self.args = args
        self.lock = threading.Lock()
        self.teleop = [0.0, 0.0, 0.0]      # v, w, deadline
        self.burst = None
        self.busy = ""                     # what currently owns the motors
        self.world = nav.World()
        self.mapping = True
        self.trail = deque(maxlen=4000)
        self.log = deque(maxlen=500)
        self.samples = deque(maxlen=4000)  # (v_cmd, w_cmd, v_act, w_act, dt, src, volts)
        self.origin = (0.0, 0.0)            # display-frame translation, see /api/origin
        self.map_cache = {}                # (half, lidar) -> (png, monotonic)
        self.map_geom = None               # world->pixel affine of the last rendered map
        # Carry detector and relocaliser, see _carry_check. "agree" is how much of the live sweep
        # the map can explain where the odometry says we are; "best" is the offset that would
        # explain more, and "rival" is the second, *separate* place that also fits -- the number
        # that decides whether the machine is allowed to move the pose or must ask a human.
        self.reloc = {"agree": None, "best": None, "dx": 0.0, "dy": 0.0, "why": "启动中",
                      "support": 0, "at": 0.0, "frozen": False,
                      "rival": 0.0, "rdx": 0.0, "rdy": 0.0, "cand": [], "trial": None,
                      "bias": [0.0, 0.0], "ask": False, "stale": None}
        self.watch_hist = deque(maxlen=64)
        self.watch_bad = 0
        self.watch_err_t = 0.0
        self.stale_t = 0.0
        self.reloc_tries = 0
        # Moving drift suppressor, see _drift_probe/_drift_correct. hist rows:
        # (t, dx, dy, zero, best, support, metres moved since last sample, heading deg)
        self.drift = {"res": None, "agree": None, "why": "还没在运动中测过", "last": None,
                      "est": [0.0, 0.0], "roll": 0.0, "total": 0.0, "n": 0}
        self.drift_hist = deque(maxlen=600)
        self.drift_est = [0.0, 0.0]      # m: running estimate of how far off the pose is
        self.drift_est_at = 0.0
        self.drift_roll = 0.0             # m driven since the last nudge
        self.drift_total = 0.0            # m corrected on this map
        self.drift_n = 0
        self.drift_bias0 = None           # bias when the suppressor first paid out: the budget is
                                          # about how far *it* walked the pose, not how far the
                                          # pose sits from raw odom -- that can legitimately be
                                          # metres when the saved map was drawn from a different
                                          # odom origin, and charging it here made every session
                                          # give up on its very first correction.
        # Mid-motion relocation check, see _motion_watch: the same search the stopped-car
        # detector runs, asked while the wheels are turning, with stricter gates because the
        # answer pulls the motors out from under a running command.
        self.motion = {"why": "还没在运动中查过定位", "zero": None, "best": None, "suspect": False,
                       "outside": None, "dx": None, "dy": None, "bad": 0, "at": None,
                       "last": None, "n": 0}
        self.cache_file = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "cache_console", "grid.npy")
        self.origin_file = os.path.join(os.path.dirname(self.cache_file), "origin.json")
        self.bias_file = os.path.join(os.path.dirname(self.cache_file), "bias.json")
        # Remembered places live in the same frame as the grid (pose = odom + bias), which is
        # why 清空地图 must delete them together with it: a wall-less blank grid is a new frame,
        # and "the sofa is 1.5 m left" carried over would be a coordinate from another room.
        self.places_file = os.path.join(os.path.dirname(self.cache_file), "places.json")
        self.places = []
        self.last_save = 0.0
        self.tour = None
        self.started = time.monotonic()
        self.push("控制台启动")
        try:
            os.makedirs(os.path.dirname(self.cache_file), exist_ok=True)
        except OSError:
            pass
        self._load_grid()
        self._load_places()
        self._load_samples()

    # ---------------------------------------------------------------- log ----
    def push(self, text, kind="info"):
        self.log.appendleft({"t": round(time.monotonic() - self.started, 1),
                             "kind": kind, "text": text})

    # ------------------------------------------------------------- motors ----
    def claim(self, what):
        with self.lock:
            if self.busy:
                return False
            self.busy = what
        return True

    def release(self, what):
        with self.lock:
            if self.busy == what:
                self.busy = ""

    def set_teleop(self, v, w, hold):
        v = max(-CAP_V, min(CAP_V, float(v)))
        w = max(-CAP_W, min(CAP_W, float(w)))
        now = time.monotonic()
        with self.lock:
            if self.busy:
                return False
            if self.car.acting:
                # The idle cmd_vel stream stands down while an MCP command owns the wheels, so a
                # teleop accepted here would be a stick that moves nothing: the UI would show
                # 点动中 while the car keeps walking the Move. Refuse it and say why.
                self.push("点动已忽略：电机正被 MCP 命令占用（要接管请先急停）", "error")
                return False
            active = now < self.teleop[2]
            if not active and (v or w):
                with self.car.lock:
                    yaw0 = self.car.pose[2] if self.car.pose else 0.0
                self.burst = {"t0": now, "v": v, "w": w, "dist": 0.0, "yaw": 0.0,
                              "along": 0.0, "yaw0": yaw0, "prev": None}
            self.teleop = [v, w, now + max(0.05, min(1.0, float(hold)))]
            return True

    def stop_all(self, who="machine"):
        # `who` is not decoration: a human's 急停 and the machine's own distrust both land here,
        # and the running action has to be able to tell them apart -- the human gets "收到停止指令",
        # the machine must abort with the verdict that caused it, or the log says "stop" while the
        # map is what actually went wrong.
        with self.lock:
            self.teleop = [0.0, 0.0, 0.0]
            self.car.nav_stop = True        # also aborts a running tour
            self.car.stop_src = who
        self.push("急停" if who != "machine" else "急停（机器判定）", "stop")
        threading.Thread(target=self.car.stop_stream, args=(0.6,), daemon=True).start()
        return True

    def _stream(self):
        """Publish cmd_vel: the teleop target while it is alive, zeros otherwise."""
        while True:
            now = time.monotonic()
            with self.lock:
                v, w, deadline = self.teleop
                busy = self.busy
            # `acting` is the part that was missing: an MCP `Move` never claims `busy`, so this
            # loop kept pushing zeros at 5 Hz straight into the middle of a Move. Measured on a
            # `Move forward 0.5` -- cmd 0.25 m/s with a (0.00,0.00) between every other sample,
            # odom tracking 0.05 m/s, 10 s for a hop the board does at 0.2 m/s. The board treats
            # a zero as a brake, so a Move was fighting a hand on the brake the whole way, and
            # every "长命令超时" and "巡航时底盘不听 angular.z" was partly this, not the chassis.
            if busy or self.car.acting:
                time.sleep(0.2)             # a tour, a calibration or an MCP Move owns the motors
                continue
            if now < deadline:
                self._measure(v, w)
                self.car.drive(v, w)
                time.sleep(STREAM_DT)
            else:
                if self.burst:
                    self._close_burst()
                self.car.drive(0.0, 0.0)
                time.sleep(0.2)

    def _measure(self, v, w):
        """Accumulate one teleop burst so its achieved velocities can be fitted afterwards."""
        b = self.burst
        if b is None:
            return
        with self.car.lock:
            pose = self.car.pose
        if pose is None:
            return
        x, y, yaw = pose[0], pose[1], pose[2]
        if b["prev"] is None:
            b["prev"] = (x, y, yaw)
            return
        px, py, pyaw = b["prev"]
        b["dist"] += math.hypot(x - px, y - py)
        # project onto the heading the burst started with: that is the axis the command was on
        b["along"] += ((x - px) * math.cos(b["yaw0"]) + (y - py) * math.sin(b["yaw0"]))
        b["yaw"] += wrap_pi(yaw - pyaw)
        b["prev"] = (x, y, yaw)

    def _close_burst(self):
        b, self.burst = self.burst, None
        if not b:
            return
        dt = time.monotonic() - b["t0"]
        if dt < 0.4:
            return
        v_act = b["along"] / dt
        w_act = b["yaw"] / dt
        self._add_sample(b["v"], b["w"], v_act, w_act, dt, "teleop")
        if abs(b["v"]) > 0.02 or abs(b["w"]) > 0.05:
            # Below FIT_MIN_DT this average is dominated by the ramp the board accelerates into
            # (it holds 0.2-0.3 s of queued twist), so it is not a rate the chassis ever holds.
            # The number has to say that on its own line: a 2.19 s / w=-0.30 burst printed as
            # "实测 -0.023" reads like a 13x failure, while the same command over 6 s gives
            # -0.22 -- and the fit already drops the short one, so the log was the only place
            # where the short sample looked like a fault.
            self.push("点动 %.1f s：指令 v=%.2f w=%.2f → 实测 v=%.3f w=%.3f%s"
                      % (dt, b["v"], b["w"], v_act, w_act,
                         "" if dt >= FIT_MIN_DT else
                         "（短脉冲：整段都在起步爬升里，不是稳态速率，拟合不收）"), "meas")

    # ------------------------------------------------------------- mapping ----
    def add_calib_sample(self, c):
        """Feed a calibration burst into the same sample log the teleop stream writes to."""
        if not c or c["dt"] <= 0:
            return
        self._add_sample(c["v"], c["w"], c["v_act"], c["w_act"], c["dt"], "calib")
        self.push("校正 %.1f s：指令 v=%.2f w=%.2f → 里程计 v=%.3f w=%.3f%s"
                  % (c["dt"], c["v"], c["w"], c["v_act"], c["w_act"],
                     ("，陀螺仪 w=%.3f" % c["gyro_w"]) if c.get("gyro_n") else ""), "meas")

    def _add_sample(self, v, w, v_act, w_act, dt, src):
        """Record one measured burst in the pool the fit reads, and on disk.

        The pool used to be memory-only: restarting the bridge between two calibration sessions
        silently dropped every 6-8 s sweep, and the next fit was left with only the teleop taps
        (which the duration gate then rejects, so the honest answer became "no samples").
        The pack voltage at sampling time rides along because the gains are a function of it.
        """
        volts = self.car.volts
        self.samples.append((v, w, v_act, w_act, dt, src, volts))
        try:
            with open(CALIB_FILE, "a", encoding="utf-8") as f:
                f.write(json.dumps([round(v, 4), round(w, 4), round(v_act, 4), round(w_act, 4),
                                    round(dt, 2), src,
                                    None if volts is None else round(float(volts), 2)]) + "\n")
        except OSError:
            pass

    def _load_samples(self):
        """Refill the pool from calib_samples.jsonl -- the newest SAMPLE_CAP rows win."""
        if not os.path.exists(CALIB_FILE):
            return 0
        try:
            with open(CALIB_FILE, encoding="utf-8") as f:
                lines = f.read().splitlines()[-SAMPLE_CAP:]
        except OSError:
            return 0
        n = 0
        for ln in lines:
            try:
                r = json.loads(ln)
            except ValueError:
                continue
            if not isinstance(r, list) or len(r) < 6 or r[4] is None:
                continue
            self.samples.append(tuple(r[:6]) + (r[6] if len(r) > 6 else None,))
            n += 1
        if n:
            self.push("已恢复 %d 条历史校正样本（%s）" % (n, os.path.basename(CALIB_FILE)), "meas")
        return n

    def clear_samples(self):
        """Empty the fit pool and its file copy.

        Samples are only comparable with each other if the chassis was in the same state -- after
        a firmware odom re-scale (VELOCITY_SCALE) or a motor/battery swap the old rows measure a
        different car, and pooling them is worse than having nothing to fit.
        """
        n = len(self.samples)
        self.samples.clear()
        try:
            open(CALIB_FILE, "w", encoding="utf-8").close()
        except OSError:
            pass
        self.push("已清空 %d 条校正样本，拟合池现在是空的" % n, "gain")
        return {"cleared": n, "left": len(self.samples)}

    # ------------------------------------------------------------- mapping ----
    def _fold(self):
        """Fold the live lidar into the growing odom-frame grid while the console is open."""
        last = None
        while True:
            time.sleep(0.2)
            with self.lock:
                mapping, touring = self.mapping, self.busy == "tour"
            if not mapping:
                continue
            pose, pairs = self.car.scan_sample()
            if pose is None or not pairs:
                continue
            if touring:
                # The Pilot owns the grid while a tour runs -- folding the same sweep twice would
                # double its votes -- but reading it costs nothing and this is the only moment the
                # car is *moving*, which is exactly what drift looks like. Observe, don't write.
                self._drift_probe(pose, pairs)
                self._motion_watch(pose, pairs)     # ... and what being carried looks like
                continue
            try:
                if self._carry_check(pose, pairs):
                    continue        # a sweep the grid cannot explain is not evidence, it is damage
            except Exception as exc:
                now = time.monotonic()      # a check that throws runs five times a second
                if now - self.watch_err_t > 10.0:
                    self.watch_err_t = now
                    self.push("搬动检测异常（已跳过，建图继续）：%s: %s"
                              % (type(exc).__name__, exc), "error")
            if self._hold_vote():
                # The rolling detector just said "another placement explains this sweep much
                # better" -- and folding the same sweep at the pose odometry claims is how a lie
                # becomes the map. Measured on the car 2026-10-18: with a +0.80 m injected error,
                # agreement read 0.71 with the peak at exactly -0.80 (a correct diagnosis), then
                # climbed back to 0.95 while the fold loop kept voting, so the second sample the
                # abort waits for never arrived. Hold the vote; the next sample is then measured
                # against the same ground, and an honest frame clears the flag by itself.
                continue
            self.world.integrate(pose, pairs)
            if last is None or math.hypot(pose[0] - last[0], pose[1] - last[1]) > 0.05 \
                    or abs(wrap_pi(pose[2] - last[2])) > 0.05:
                self.trail.append((pose[0], pose[1], pose[2]))
                last = (pose[0], pose[1], pose[2])
            self._save_grid()

    def _hold_vote(self):
        """True while a witness still believes this pose is the wrong place to fold at.

        Three independent ones: the rolling relocation detector (another placement explains this
        sweep much better), and two ways the bridge catches odometry inventing the pose -- the
        odom-vs-gyro heading row (measured 2026-10-19: 40 deg of rotation the gyro never saw), and
        the odom-vs-ring distance row (the wheels counted metres the room behind never receded).
        The first attacks the fold's angle, the other two its position; a ghost wall only needs
        one of them to be believed.

        Its own method because the rule is the detector's, not the fold loop's: an offline test
        that has to reproduce when a sweep is allowed to reach the grid needs the same answer, and
        a copy in the test could disagree with the thing it is testing.

        `bad` is the wrong signal to read here. It counts toward an abort and is reset by one good
        sample, so the *first* suspicious frame -- the one whose fold is doing the damage -- leaves
        it at zero, and the detector only evaluates after the car has covered MOTION_ROLL metres.
        `suspect` is the sticky opinion instead: it is set by every frame that reaches a verdict and
        cleared by a frame that does not, which is what makes "hold this sweep out" survive the gap
        between two counted samples.
        """
        with self.lock:
            at = self.motion["at"]
            lie_t = self.car.yaw_lie_t       # odom heading caught inventing rotation
            slip_t = self.car.slip_t         # odom distance caught inventing travel
            return bool((self.motion["suspect"] and at is not None
                         and time.monotonic() - at <= MOTION_HOLD)
                        or time.monotonic() - lie_t <= MOTION_HOLD
                        or time.monotonic() - slip_t <= MOTION_HOLD)

    # ------------------------------------------------------- drift probe ----
    def _drift_probe(self, pose, pairs):
        """Measure, and while the car rolls pay the pose back toward the map in small instalments.

        Wheel odometry is known to close a lap about 3% short and to lose heading while driving,
        but nobody had measured what that looks like *per sweep* against the map, so this began as
        a record-only probe (/api/drift) and the gates in DRIFT_* come out of those samples.

        Deliberately a narrow search: a carry is metres and belongs to _carry_check; drift is
        centimetres, and widening the window here would let the probe "find" a different room.
        """
        now = time.monotonic()
        r = nav.scan_offset(self.world, pose, pairs, reach=DRIFT_REACH,
                            step=DRIFT_STEP, every=DRIFT_EVERY)
        with self.lock:
            if r is None:
                self.drift["why"] = "地图太稀或回波太少（%d 点）" % len(pairs)
                return
            moved = 0.0
            if self.drift["last"] is not None:
                lx, ly, _ = self.drift["last"]
                moved = math.hypot(pose[0] - lx, pose[1] - ly)
            self.drift["last"] = (pose[0], pose[1], now)
            self.drift["why"] = None
            self.drift["res"] = [round(r["fdx"], 3), round(r["fdy"], 3)]
            self.drift["agree"] = round(r["zero"], 3)
            # refined offset first, then the grid answer it came from: if the two ever disagree by
            # more than a step, the parabola is fitting noise and the deadband below is invalid.
            self.drift_hist.append((round(now - self.started, 1), r["fdx"], r["fdy"], r["zero"],
                                    r["best"], r["support"], moved, math.degrees(pose[2])))
            # A relocation trial is the matcher busy with a metres-scale hypothesis, and a frozen
            # map is one a human has not confirmed yet; neither is a place to be trimming pose.
            busy_elsewhere = bool(self.reloc["trial"]) or self.reloc["frozen"]
        if not busy_elsewhere:
            self._drift_correct(r, moved, now)

    def _drift_correct(self, r, moved, now):
        """Average the trustworthy residuals and spend them on the pose, damped and rate-limited.

        Four failure modes had to be designed out with the numbers from the last section, not
        argued away. (1) Noise: once a sweep stops matching the grid the search surface goes flat
        and the peak wanders to the edge of the window, so only residuals that both find a strong
        peak *and* leave the claimed pose still explaining most of the sweep enter the estimate --
        that is the same evidence the drift of a stopped car is measured with. (2) Windup: the
        residual measures how far off the pose is *now*, so it is averaged (DRIFT_ALPHA), never
        summed; a summed version on the real car turned 2.1 m of driving into a 1.33 m bank and
        then spent it at the rate cap for the rest of the tour. (3) Runaway: a *constant* bias in
        the matcher (an unmodelled lidar-to-base offset would do it) would otherwise be integrated
        into a pose marching a few millimetres per tick forever, so one nudge may never exceed
        DRIFT_RATE times the metres actually driven since the last one -- the correction is allowed
        to be no faster than odometry is known to be able to be wrong. (4) Thrash: every payout
        re-registers the sweeps the map is made of, so a nudge has a minimum size worth making, an
        estimate has to clear a deadband twice its own scatter, and the session has a cap in metres
        -- past the cap the suppressor has clearly lost, and it says so and stops instead of
        quietly walking the pose across the room. What it does *not* have is a cooldown: spacing
        payouts out was measured on a synthetic lap and made the map worse, because the error keeps
        growing and smearing folds while the corrector waits its turn.
        """
        with self.lock:
            self.drift_roll += moved
            if now - self.drift_est_at > DRIFT_STALE:
                self.drift_est = [0.0, 0.0]
            if not (r["best"] >= DRIFT_BEST and r["zero"] >= DRIFT_ZERO
                    and max(abs(r["fdx"]), abs(r["fdy"])) <= DRIFT_MAX):
                self.drift["est"] = [round(v, 3) for v in self.drift_est]
                return
            self.drift_est = [(1.0 - DRIFT_ALPHA) * e + DRIFT_ALPHA * m
                              for e, m in zip(self.drift_est, (r["fdx"], r["fdy"]))]
            self.drift_est_at = now
            ex, ey = self.drift_est
            if math.hypot(ex, ey) < DRIFT_DEAD:
                self.drift.update(est=[round(ex, 3), round(ey, 3)],
                                  roll=round(self.drift_roll, 2))
                return
            # Damped payout, then capped twice over: by one step, and by what the metres driven
            # since the last payout could physically have cost in odometry error.
            cx, cy = DRIFT_GAIN * ex, DRIFT_GAIN * ey
            lim = min(DRIFT_CAP, DRIFT_RATE * self.drift_roll)
            if lim <= 0.0:                      # the car has not driven far enough to have earned
                return                          # this much error yet -- wait for the road to show it
            earned = self.drift_roll
            over = math.hypot(cx, cy) / lim
            if over > 1.0:
                cx, cy = cx / over, cy / over
            if math.hypot(cx, cy) < DRIFT_MIN:
                return                          # the road only allows a sliver: keep estimating
        bias0 = list(self.car.bias)
        if self.drift_bias0 is None:
            # Baseline is the pose the suppressor inherited, not raw odometry: a map drawn under a
            # different odom origin starts metres away by construction (+4.45, +3.56 here).
            self.drift_bias0 = bias0
        new_bias = self.car.nudge_bias(cx, cy)
        # Judged on how far the suppressor walked the pose *from where it was handed the car*:
        # a car pacing along and back pays out centimetres in both directions all afternoon without
        # ever being in the wrong place, and only a *net* walk means the suppressor has lost.
        walk = math.hypot(new_bias[0] - self.drift_bias0[0], new_bias[1] - self.drift_bias0[1])
        give_up = walk > DRIFT_TOTAL
        with self.lock:
            # What the payout did not cover stays in the estimate; the next sweeps decide the rest.
            self.drift_est = [ex - cx, ey - cy]
            self.drift_roll = 0.0
            self.drift_total += math.hypot(cx, cy)
            self.drift_n += 1
            self.drift.update(est=[round(v, 3) for v in self.drift_est],
                              roll=0.0, total=round(self.drift_total, 3), n=self.drift_n)
        self._save_bias()
        self.push("漂移修正 %+.3f, %+.3f m（雷达估计此刻差 %+.2f, %+.2f，%.1f m 行程的误差预算"
                  "只取 %.0f%%，本次共 %.2f m / %d 次，总修正 %+.2f, %+.2f）"
                  % (cx, cy, ex, ey, earned, 100 * DRIFT_GAIN,
                     self.drift_total, self.drift_n, new_bias[0], new_bias[1]), "map")
        if give_up:
            # Its own verdict, not the carry wording: the budget ran out, the car was not touched.
            self._freeze_carry(r, True, ask=True, because=(
                "漂移修正从它接手的位姿出发已累计挪了 %.2f m（花费合计 %.2f m / %d 次，预算 %.1f m），"
                "超出里程计自身可能错的量，机器停止自修并交回人工确认位置"
                % (walk, self.drift_total, self.drift_n, DRIFT_TOTAL)))

    # ------------------------------------------------------ carry detector ----
    def _through_pair(self, pose, pairs, r):
        """See-through share at the pose odometry claims and at the pose the matcher prefers.

        The pair, not one number: a carry shows up as the candidate *fixing* the contradiction,
        a bad patch of map as nobody fixing it. None when the sweep is too thin to answer, and
        callers treat that as "no veto" rather than as evidence.
        """
        t0 = nav.through_wall(self.world, pose, pairs)
        if t0 is None:
            return None, None
        t1 = nav.through_wall(self.world, (pose[0] + r["dx"], pose[1] + r["dy"], pose[2]), pairs)
        return t0, t1

    def _veto_check(self, t0, t1):
        """Does the see-through gate have standing to overrule a "the score found a better place"?

        One answer for both detectors, because the stopped car and the rolling car were reaching
        opposite verdicts on the same sweep while this lived as two inline comparisons:
          "none" -- the sweep is too thin to ask, so the score's word stands unopposed.
          "veto" -- the claimed pose *is* contradicted (t0 clears THROUGH_ACCUSE), the candidate
                    removes that contradiction no better (t1 > t0 - THROUGH_BETTER), and the map can
                    still explain the sweep somewhere (t1 <= THROUGH_JUNK). That is a wrongly drawn
                    patch: relocating onto it would chase a fiction, so keep the pose and keep
                    folding -- folding is what votes the phantom walls down.
          "junk" -- the same lack of improvement, but no placement the search can reach explains the
                    sweep. The grid has lost the right to accuse the car *or* itself; only stopping
                    is safe, and the wording must say which of the two it could not tell.
          "ok"   -- the candidate explains better, or the claimed pose was never contradicted so
                    "it gets no worse" proves nothing. The carry verdict stands.
        """
        if t0 is None or t1 is None:
            return "none"
        if t1 <= t0 - THROUGH_BETTER or t0 < THROUGH_ACCUSE:
            return "ok"
        return "junk" if t1 > THROUGH_JUNK else "veto"

    def _motion_watch(self, pose, pairs):
        """Stop the car if the map can no longer explain where it thinks it is *while rolling*.

        Called from every branch of _carry_check that used to hand off to the drift probe and
        return. It is a second, wider search over the same sweep: the drift probe asks "how many
        millimetres off am I" inside 0.6 m, this asks "is the world still where I claim it is"
        inside 1.6 m, and the two answers are not interchangeable -- on the real car a genuine
        carry pins the drift residual to the edge of its window and looks, from there, exactly
        like noise.

        Deliberately refuses to *relocate* here. Moving the pose a metre while the wheels are
        turning means the next fold lands on a hypothesis the sweep that produced it never got to
        confirm, and the map would absorb the guess. So this only pulls the motors, and the
        stopped-car detector -- which has the trial/verify/revert machinery -- takes over a second
        later from a standing pose.
        """
        now = time.monotonic()
        with self.lock:
            if self.reloc["frozen"] or self.reloc["trial"]:
                return                # somebody else already owns this hypothesis
            if self.motion["last"] is None:
                self.motion["last"] = (pose[0], pose[1], now)
                return
            lx, ly, lt = self.motion["last"]
            if now - lt < 0.2 or math.hypot(pose[0] - lx, pose[1] - ly) < MOTION_ROLL:
                return                # same ground, same answer: do not count it twice
            self.motion["last"] = (pose[0], pose[1], now)
        r = nav.scan_offset(self.world, pose, pairs, reach=MOTION_REACH,
                            step=MOTION_STEP, every=MOTION_EVERY)
        if r is None:
            with self.lock:
                # No opinion, so no veto: holding the vote here would stop the map from ever
                # growing into the ground that made the answer impossible.
                self.motion.update(suspect=False, why="地图太稀，运动中没资格判断定位", at=now)
            return
        # A coherent alternative placement is evidence of a carry wherever it comes from. But
        # "nothing fits anywhere" is *not*: that is also exactly what driving into unmapped ground
        # looks like, and the two score the same way. `outside` is what separates them, and it is
        # the reason a mid-motion abort needs a stricter bar than a stopped one does.
        # `fits` is the score's question, `unambiguous` the sharpness of its peak, and the pair
        # below the grid's. All three are needed in different mixes, and the mix that used to be
        # silent is the young map: measured in _thin_probe.py, a map folded one, two or three times
        # reads 吻合 0.03, 峰值 0.95-0.99 *and* 次优 0.94, because with only a few hundred claimed
        # cells almost every offset collects the same handful of returns. RELOC_AMBIG then refuses
        # the verdict -- and it is right to, for a *relocation*: sliding the pose onto a broad peak
        # is a guess. But this detector does not move anything, it stops. Refusing to stop because
        # the evidence is not sharp enough to *act on* is the wrong way round, so when the grid
        # answers the same question as the score -- the offset removes the contradiction the claimed
        # pose is sitting in -- that is enough to pull the motors here.
        #
        # Measured in _healed_probe.py, the same route is also what catches the *second* half of the
        # live failure. Replay the car's own 0.8 m jump against the map it already had and the score
        # dies in four frames -- 间距 0.96, 0.36, 0.32, 0.23, 0.11 -- because each frame folded at
        # the lying pose draws a fiction room the lie then matches. WANT_MARGIN is the only gate that
        # closes, and `through_wall` still has 0.46 -> 0.21 of contrast to point at while the score
        # is calling the pose fine. The grid's accusation therefore has to be a way in on its own,
        # not a tiebreaker inside the score's branch.
        offset = math.hypot(r["dx"], r["dy"])
        t0 = t1 = None
        if offset > DRIFT_MAX:
            # Ask only when the matcher actually proposes somewhere beyond odometry noise; the
            # honest frame is then free, and `t1 <= t0 - THROUGH_BETTER` can not hold at offset 0.
            t0, t1 = self._through_pair(pose, pairs, r)
        fits = r["best"] - r["zero"] >= WANT_MARGIN and offset > DRIFT_MAX
        accuse = (t0 is not None and r["best"] >= RELOC_FLOOR
                  and t1 <= t0 - THROUGH_BETTER)
        unambiguous = r["best"] >= RELOC_FLOOR and r["best"] - r["rival"] >= RELOC_AMBIG
        moved = False
        stale = False
        unverifiable = False
        if fits:
            if unambiguous:
                moved = True
                check = self._veto_check(t0, t1)
                if check == "veto":
                    moved, stale = False, True
                elif check == "junk":
                    # The veto just lost its standing. This map contradicts the sweep at every
                    # placement the search can reach, so "somebody moved the car" is unsupported
                    # and so is the opposite. Keep the pose -- nothing slides onto the argmax --
                    # but stop pretending the car knows where it is.
                    moved, stale, unverifiable = True, False, True
            elif accuse:
                # Broad peak, but the grid's answer agrees with the score's: moved.
                moved = True
        elif accuse:
            # The score has gone blind -- a diluted or half-healed map makes every offset look
            # equally good -- and the grid has not. Stopping is the only thing this branch does:
            # relocating onto a peak this blunt would fold the guess into the same map that just
            # failed to see the lie.
            moved = True
        lost = (r["zero"] < LOST_FLOOR and r["best"] < BEST_FLOOR
                and r["outside"] < MOTION_FRESH)
        with self.lock:
            self.motion.update(zero=round(r["zero"], 3), best=round(r["best"], 3),
                               outside=round(r["outside"], 3),
                               dx=round(r["dx"], 2), dy=round(r["dy"], 2), at=now)
            # Sticky until an honest frame clears it. `bad` counts toward an abort and resets on a
            # single good sample; this one says "the last time I looked, this pose did not
            # explain the sweep", which is what decides whether the sweep may vote.
            self.motion["suspect"] = bool(moved or lost)
            if stale:
                self.motion["bad"] = 0
                self.motion["why"] = ("行进中图上这块有看得穿的墙（当前位姿 %.0f%%、匹配器首选 %.0f%%），"
                                      "不算搬动；继续建图会把这些假墙投票掉" % (100 * t0, 100 * t1))
            else:
                self.motion["bad"] = self.motion["bad"] + 1 if (moved or lost) else 0
                bad = self.motion["bad"]
                if unverifiable:
                    verdict = "这块图不足以判断"
                elif moved:
                    verdict = "另一种解释更合适"
                    if not fits:
                        # Which sense pulled the motors matters to whoever reads the log: this
                        # frame is the one the score would have walked past.
                        verdict += "（打分已经看不出，是看得穿的墙发现的）"
                else:
                    verdict = "已测过的地面上对不上"
                self.motion["why"] = None if bad == 0 else (
                    "%s，连续 %d/%d 次（吻合 %.0f%%，峰值 %.0f%% 在 (%+.1f, %+.1f) m）"
                    % (verdict, bad, MOTION_BAD, 100 * r["zero"], 100 * r["best"], r["dx"], r["dy"]))
        if stale:
            # The stopped car and the rolling one reach the same verdict for the same reason, so
            # say it with the same words: one banner, one log line, one `stale` field. Called
            # outside the lock because _stale_map takes it.
            self._stale_map(r, t0, t1)
            return
        if bad < MOTION_BAD:
            return
        reason = ("行进中雷达与地图对不上（%s：吻合 %.0f%%，最好偏移 %.2f m 处 %.0f%%，"
                  "未测回波占 %.0f%%）"
                  % (verdict, 100 * r["zero"], math.hypot(r["dx"], r["dy"]), 100 * r["best"],
                     100 * r["outside"]))
        if unverifiable:
            # Say which of the two it could not tell, and say it with the numbers: the map is the
            # accused party here, not the person who might have picked the car up.
            reason += ("：这块图自己就矛盾（当前位姿 %.0f%%、匹配器首选 %.0f%% 的回波穿过了图上声称的墙），"
                       "机器没能确认是搬动还是图画错，没有改位姿，停止自动行驶并交回人工"
                       % (100 * t0, 100 * t1))
        else:
            reason += "，已停车重新定位"
        if unverifiable:
            # The map's half of the verdict still stands and still gets recorded, with the same
            # banner and the same `stale` field the stopped car writes: the grid is the accused
            # party. The sweeps that prove it are *not* folded meanwhile -- _hold_vote keeps them
            # out, because on a map this contradicted there is no reason to believe the pose they
            # would be folded at, and a lie drawn in is harder to remove than a phantom wall.
            self._stale_map(r, t0, t1, hold=True)
        with self.lock:
            busy = self.busy
            self.motion.update(bad=0, n=self.motion["n"] + 1, why=reason)
        self._machine_stop(reason, busy)

    def _machine_stop(self, reason, busy):
        """Take the wheels out of a machine-owned drive; warn a human who holds the stick.

        Two verdicts call this now -- the score's "another place fits better" and the veto's
        "this map cannot answer" -- and who gets stopped must not drift apart between them.
        """
        if not (busy or self.car.acting):
            # A human has the stick. Saying so is useful; yanking teleop control away from them
            # is not what they signed up for, and they can see the room.
            self.push("注意: " + reason + "（点动中，未代管停车）", "error")
            return
        self.car.abort_reason = reason        # the running loop quotes this as it unwinds
        self.stop_all()
        self.push(reason, "error")

    def _why(self, s):
        """Why the detector is silent right now. Without this a healthy-looking 未检测 is
        indistinguishable from a detector that stopped checking three gates ago."""
        with self.lock:
            self.reloc["why"] = s

    def _carry_check(self, pose, pairs):
        """Ask whether the grid can still explain this sweep, and answer *before* folding it in.

        The order is the whole point. A sweep taken after somebody carried the car disagrees
        with the grid for about a second, and then the grid simply grows a room around it and
        reads 100% again -- measured, not theorised, by shifting a saved map by 1.0 m and
        watching the agreement snap back while the fold loop kept running. Returns True when
        this sweep must not be folded.
        """
        with self.lock:
            busy, teleop_live = self.busy, time.monotonic() < self.teleop[2]
        # Every case the carry detector refuses to judge is a case where the car is *rolling*,
        # and that is precisely where odometry drift accumulates -- so probe those, record-only.
        # `acting` belongs here too: an MCP `Move` drives the board without claiming the console's
        # `busy`, and the standing-car path below starts a relocation trial which (a) can never
        # verify from a pose that is moving underneath it and (b) sets reloc["trial"] -- the flag
        # _motion_watch reads as "somebody else owns this". Measured 2026-10-18: a 1.1 m lateral
        # error injected 0.5 s into a Move was grabbed by that trial, the check that could have
        # stopped the car stayed silent for the whole command, and the Move finished its 0.5 m
        # with the pose still lying.
        if busy or self.car.acting:
            self._drift_probe(pose, pairs)
            self._motion_watch(pose, pairs)
            return self._gate("电机被 %s 占用" % (busy or "MCP 命令"))
        if teleop_live:
            self._drift_probe(pose, pairs)
            self._motion_watch(pose, pairs)
            self.watch_hist.clear()
            self.watch_bad = 0
            return self._gate("点动中")
        age = self.car.scan_age()
        if age is None or age > 1.0:
            return self._gate("雷达过期 %.1f s" % (age or -1))
        # A stalled /odom stream freezes the pose, and a frozen pose has a spread of exactly zero,
        # which the window below reads as "the car is parked" and then judges against the map.
        # A silent wheel encoder is not evidence the car is standing still, so name the difference
        # instead of letting a dropped stream be reported as somebody's hands.
        odom_age = time.monotonic() - pose[3]
        if odom_age > 1.0:
            return self._gate("里程计 %.1f s 没更新（位姿冻住不等于车停着）" % odom_age)
        now = time.monotonic()
        self.watch_hist.append((now, pose[0], pose[1]))
        win = [h for h in self.watch_hist if now - h[0] <= STILL_WINDOW]
        if len(win) < STILL_TICKS:
            return self._gate("刚停下，攒样本 %d/%d" % (len(win), STILL_TICKS))
        drift = max(max(p[1] for p in win) - min(p[1] for p in win),
                    max(p[2] for p in win) - min(p[2] for p in win))
        if drift > STILL_TOL:
            self.watch_bad = 0                  # rolling: the sweep is not comparable to the map
            self._drift_probe(pose, pairs)      # ... but it is exactly what drift suppression reads
            self._motion_watch(pose, pairs)     # ... and this is what a mid-move carry looks like
            return self._gate("车在动（%.1f cm）" % (drift * 100))
        r = nav.scan_offset(self.world, pose, pairs)
        if r is None:
            return self._gate("地图太稀或回波太少（%d 点）" % len(pairs))
        with self.lock:
            self.reloc.update(agree=r["zero"], best=r["best"], dx=r["dx"], dy=r["dy"],
                              rival=r["rival"], rdx=r["rdx"], rdy=r["rdy"],
                              support=r["support"], at=now, why=None,
                              # Per-sweep, not sticky: the next honest sweep has to clear the
                              # banner, or "this patch is drawn wrong" hangs there forever after
                              # the folding voted the phantom walls down.
                              stale=None)
        # A coherent alternative placement of the *whole* sweep is what separates a carry from
        # a person walking in front: a person replaces a third of the returns and matches again
        # nowhere, so they can trip `lost` but never `moved`.
        if self.reloc["trial"] and time.monotonic() - self.reloc["trial"][4] > RELOC_TTL:
            self._reloc_settle(None)              # no confirming sweep ever arrived
            return True
        moved = (r["best"] - r["zero"] >= WANT_MARGIN and r["best"] >= BEST_FLOOR
                 and math.hypot(r["dx"], r["dy"]) > 0.25)
        lost = r["zero"] < LOST_FLOOR and r["best"] < BEST_FLOOR
        if self.reloc["trial"]:
            return self._reloc_settle(r)
        if moved:
            # Before blaming somebody's hands: does that placement actually explain the sweep, or
            # does it only score better? See nav.through_wall and _veto_check.
            t0, t1 = self._through_pair(pose, pairs, r)
            check = self._veto_check(t0, t1)
            if check == "junk":
                # The veto loses its standing on a map that contradicts the sweep everywhere
                # the search can reach: "the grid drew a wall that is not there" is only a
                # cure worth folding more sweeps into if the grid can still be right about
                # something. Nothing is relocated and nothing is folded -- the pose is exactly
                # what nobody believes, and this is the same verdict the rolling detector
                # reaches for the same sweep one branch over.
                self._stale_map(r, t0, t1, hold=True)
                self._freeze_carry(r, True, ask=True, because=(
                    "位姿与地图对不上，但这块图不足以判断原因：当前位姿 %.0f%%、匹配器首选 %.0f%%"
                    "的回波都穿过了图上声称的墙，机器既不敢说是搬动也不敢说是图画错"
                    % (100 * t0, 100 * t1)))
                return True
            if check == "veto":
                with self.lock:
                    # Folding is the agreed cure here, so the rolling hold has to stand down;
                    # leaving it set would make _fold skip the very sweeps this line asks for.
                    self.motion["suspect"] = False
                return self._stale_map(r, t0, t1)
            if (r["best"] >= RELOC_FLOOR and r["best"] - r["rival"] >= RELOC_AMBIG
                    and self.reloc_tries < RELOC_TRIES):
                self._start_trial(r)              # confident and unambiguous: fix it, don't stall
                return True
            # Refused, so hand the decision to somebody with eyes. `ask` is reported rather than
            # recomputed by the page: a second copy of the threshold there is one edit away from
            # disagreeing with the machine that actually stopped the car -- and it already did.
            self._freeze_carry(r, True, ask=True)
            return True
        if not lost:
            self.watch_bad = 0
            # This sweep, judged from a standing pose, says the claim explains the world: that is
            # the honest frame the fold-hold waits for. Without it the flag would only ever be
            # cleared by driving, and a car parked in the corner it was carried into would stop
            # building the map it needs to get out.
            with self.lock:
                self.motion["suspect"] = False
            return False
        self.watch_bad += 1
        if self.watch_bad >= SUSPECT_TICKS:
            self._freeze_carry(r, False)
            return True
        return self._gate("雷达与地图对不上，第 %d/%d 次" % (self.watch_bad, SUSPECT_TICKS))

    def _gate(self, s):
        self._why(s)
        return False

    def _start_trial(self, r):
        """Move the car onto the offset the map proposed, then make the next sweep prove it.

        The sweep that produced the proposal is not folded: it was measured under the old
        registration, and folding it now would spend the very evidence being tested.
        """
        bx, by = self.car.bias
        trial = (bx, by, bx + r["dx"], by + r["dy"], time.monotonic())
        with self.lock:
            self.reloc["trial"] = trial
        self.car.set_bias(trial[2], trial[3])
        self.watch_hist.clear()
        with self.lock:
            self.reloc.update(bias=[trial[2], trial[3]])
        self.push("试重定位：把车挪到 (%+.2f, %+.2f) m 后重算位姿（原 (%+.2f, %+.2f)），"
                  "下一帧雷达验证。" % (trial[2], trial[3], bx, by), "map")

    def _reloc_settle(self, r):
        """Keep or revoke a trial. Confirmed = the sweep now lands where the map says it does."""
        trial = self.reloc["trial"]
        with self.lock:
            self.reloc["trial"] = None
        self.watch_hist.clear()
        if r is None or r["zero"] < RELOC_VERIFY:
            self.car.set_bias(trial[0], trial[1])
            self.reloc_tries += 1
            why = ("验证帧没来" if r is None
                   else "挪过去只有 %.0f%% 对得上，不到 %.0f%%" % (100 * r["zero"], 100 * RELOC_VERIFY))
            self.push("重定位失败（第 %d/%d 次）：%s。已退回原位并停住 —— "
                      "这个偏移不可信，别再凭它建图。" % (self.reloc_tries, RELOC_TRIES, why), "error")
            if r is not None:
                self._freeze_carry(r, True, quiet=True, ask=True)
            else:
                self.car.abort_reason = "重定位验证帧一直没来（车还在动？），已撤回修正并停车"
                self.stop_all()
                with self.lock:
                    self.mapping = False
                    self.reloc.update(frozen=True)
            # Those scores were measured at the pose we just threw away. Leaving them on screen
            # would show a confident 96% next to a car that is now, again, a metre off its map.
            with self.lock:
                self.reloc.update(agree=None, best=None, dx=0.0, dy=0.0,
                                  why="重定位已撤回，建图停在此处")
            return True
        with self.lock:
            self.reloc.update(bias=[trial[2], trial[3]])
        self.reloc_tries = 0
        # A committed relocation moves the pose by metres the suppressor did not spend, so its
        # budget restarts from the pose it was handed.
        self.drift_bias0 = None
        self._save_bias()
        with self.car.lock:
            cur, raw = self.car.pose, self.car.pose_raw
        self.push("重定位成功：车此刻在 (%.2f, %.2f) m = 里程计 (%.2f, %.2f) + 修正 (%+.2f, %+.2f)，"
                  "雷达与地图对上 %.0f%%，建图继续。"
                  % (cur[0], cur[1], raw[0], raw[1], trial[2], trial[3], 100 * r["zero"]), "map")
        return False

    def apply_offset(self, dx, dy):
        """Commit an offset a human chose (usually the runner-up the machine refused to guess)."""
        bx, by = self.car.bias
        self.car.set_bias(bx + float(dx), by + float(dy))
        with self.lock:
            self.reloc.update(bias=list(self.car.bias), trial=None, frozen=False,
                              agree=None, best=None)
        self.watch_hist.clear()
        self.watch_bad = 0
        self.reloc_tries = 0
        self._save_bias()
        self.resume_mapping()
        self.push("已按人工选择平移 (%+.2f, %+.2f) m，累计修正 (%+.2f, %+.2f) m"
                  % (float(dx), float(dy), self.car.bias[0], self.car.bias[1]), "map")

    def _freeze_carry(self, r, moved, quiet=False, ask=False, because=None):
        self.watch_bad = 0
        # Whoever is driving has to unwind with *this* verdict, not with a bare "收到停止指令":
        # a pose the map cannot explain reads exactly like somebody pressed the button otherwise,
        # and then the operator chases a person who was never at the console.
        # `because` replaces the wording when the trigger is not a carry at all -- the drift
        # suppressor stopping because it spent its budget is its own event, and logging it as
        # "疑似被搬动" is what made people look for hands that had never touched the car.
        why = because or ("机器判定位姿与地图对不上（吻合 %.0f%%，最好偏移 %.2f m 处 %.0f%%），"
                          "已停车并暂停建图"
                          % (100 * r["zero"], math.hypot(r["dx"], r["dy"]), 100 * r["best"]))
        self.car.abort_reason = why
        self.stop_all()
        cands = sorted([[r["dx"], r["dy"], r["best"]], [r["rdx"], r["rdy"], r["rival"]]],
                       key=lambda c: -c[2])
        with self.lock:
            self.mapping = False
            self.reloc.update(frozen=True, agree=r["zero"], best=r["best"],
                              dx=r["dx"], dy=r["dy"], rival=r["rival"],
                              rdx=r["rdx"], rdy=r["rdy"], cand=cands, trial=None, ask=ask)
        if quiet:
            return
        if because:
            self.push(because + "。已急停并暂停建图，别让错的折叠焊进地图。"
                              "确认位置后点\"恢复建图\"或选一个偏移。", "error")
            return
        self.push("疑似被搬动：里程计说车是静止的，但只有 %.0f%% 的回波能在地图上找到墙；"
                  "把雷达平移 (%+.2f, %+.2f) m 后能对上 %.0f%%，"
                  "另一个可能 (%+.2f, %+.2f) m 也有 %.0f%%。%s"
                  "已急停并暂停建图，别让错的折叠焊进地图。确认位置后点\"恢复建图\"或选一个偏移。"
                  % (100 * r["zero"], r["dx"], r["dy"], 100 * r["best"],
                     r["rdx"], r["rdy"], 100 * r["rival"],
                     "" if moved else "怎么挪都对不上，可能是环境变了或地图过期了。"), "error")

    def _stale_map(self, r, t0, t1, hold=False):
        """The sweep contradicts the grid and no other placement contradicts it less.

        So this is not "somebody moved the car" -- it is "the grid has walls in places the lidar
        is looking straight through", which a tour that ground itself against furniture draws
        often enough. The answer is the opposite of freezing: folding these very sweeps votes
        free space along each ray and knocks the phantom walls back down, while a freeze protects
        the wrong map from the only thing that can cure it. Returns False so the sweep is folded.

        `hold` is the other half of `_veto_check`: the same contradiction, but everywhere the
        matcher can reach, so the grid has no standing left to explain anything. Then folding the
        sweep would be writing at a pose nobody believes, and this returns True -- not "the map is
        wrong, keep going" but "the map is the only witness and it cannot answer".
        """
        now = time.monotonic()
        why = ("图里有看得穿的墙（当前位姿 %.0f%%、匹配器首选 %.0f%%），"
               "谁都解释不了这帧雷达，判不了；位姿没改，这帧也不折进图" % (100 * t0, 100 * t1))
        say_txt = ("地图这块画错了，而且它现在谁都证明不了：当前位置 %.0f%%、匹配器首选 %.0f%%"
                   "的回波都穿过了图上声称的墙。没有改位姿，这一帧也没折进图，"
                   "交回人工确认位置或重画地图。" % (100 * t0, 100 * t1))
        if not hold:
            why = ("图里有看得穿的墙（当前位姿 %.0f%%、匹配器首选 %.0f%%），"
                   "按地图错了处理，继续建图投票掉它" % (100 * t0, 100 * t1))
            say_txt = ("地图这块画错了，先不按搬动处理：当前位置有 %.0f%% 的回波穿过了图上声称的墙，"
                       "而匹配器想要的偏移 (%+.2f, %+.2f) m 更糟（%.0f%%）。没有改位姿也没有停图，"
                       "继续折叠会把这些假墙投票掉。"
                       % (100 * t0, r["dx"], r["dy"], 100 * t1))
        with self.lock:
            self.reloc.update(ask=False, stale=[round(t0, 3), round(t1, 3)], why=why)
            say = now - self.stale_t > 20.0
            if say:
                self.stale_t = now
        if say:
            self.push(say_txt, "warn")
        return hold

    def _drift_reset(self):
        """Throw away the suppressor's evidence and budget. Called whenever a human vouches for
        where the car is (恢复建图 / 选偏移) and whenever the map starts over: after either, the
        running estimate and the spent error budget describe a situation that no longer exists,
        and keeping them would let a stale estimate pay out against a map nobody trusts."""
        with self.lock:
            self.drift_est = [0.0, 0.0]
            self.drift_est_at = 0.0
            self.drift_roll = 0.0
            self.drift_total = 0.0
            self.drift_n = 0
            self.drift_bias0 = None
            self.drift_hist.clear()
            self.drift.update(res=None, agree=None, last=None, est=[0.0, 0.0],
                              roll=0.0, total=0.0, n=0,
                              why="重新计时")

    def resume_mapping(self):
        with self.lock:
            self.mapping = True
            self.reloc.update(frozen=False, agree=None, best=None, trial=None, cand=[], ask=False)
        self.watch_hist.clear()
        self.watch_bad = 0
        self.reloc_tries = 0
        self._drift_reset()
        self.push("已恢复建图，搬动检测重新武装", "map")

    def reset_map(self):
        with self.lock:
            self.world = nav.World()
            self.trail.clear()
            # Clearing is only ever done to start a new map, and the blank grid is anchored on
            # the raw odometry it is about to be folded with -- so it must also switch folding
            # back on. It used not to: after the machine froze mapping for a bad relocation,
            # 清空地图 wiped the good grid and then quietly drew nothing, while the log line
            # below promised "重新开始建图".
            self.mapping = True
            self.reloc.update(frozen=False, agree=None, best=None, cand=[], trial=None)
        self.watch_hist.clear()
        self.watch_bad = 0
        self.reloc_tries = 0
        self.map_cache = {}                # (half, lidar) -> (png, monotonic)
        self._drift_reset()
        # A blank grid is anchored on raw odometry again, so a bias left over from a saved map
        # would draw the new room one metre to the side of the car that is building it.
        if self.car.bias != (0.0, 0.0):
            self.car.set_bias(0.0, 0.0)
            self.reloc["bias"] = [0.0, 0.0]
        for f in (self.cache_file, self.bias_file, self.places_file):
            try:
                os.remove(f)
            except OSError:
                pass
        # A new grid is a new frame, so words pinned to the old one are not "stale", they are
        # coordinates belonging to a room that no longer exists here.
        with self.lock:
            had = len(self.places)
            self.places = []
            self.map_cache = {}
        self.push("地图已清空，重新开始建图（重定位修正一并归零%s）"
                  % ("，并忘掉 %d 个地点" % had if had else ""), "map")

    def _load_grid(self):
        """Re-seed the map from the last session: restarting the bridge used to throw away
        every fold, which is what made coordinate checks impossible after a fix."""
        try:
            lo = np.load(self.cache_file)
        except Exception as exc:               # a corrupt cache must never take down the bridge
            if os.path.exists(self.cache_file):
                self.push("上次地图缓存不可用（%s），从空白开始" % type(exc).__name__, "error")
            return
        if lo.shape == self.world.lo.shape and lo.dtype == self.world.lo.dtype:
            self.world.lo = lo
            self.push("已恢复上次地图 %s（占据 %d 格）"
                      % (os.path.basename(self.cache_file), int((lo >= 0.65).sum())), "map")
        # The origin is a point *in* that saved odom frame, so the two must come back together
        # or a bridge restart would silently move the printed zero out from under the map.
        try:
            with open(self.origin_file, encoding="utf-8") as fh:
                x, y = json.load(fh)
            self.origin = (float(x), float(y))
        except Exception:
            pass
        # ... and so must the relocalisation bias: it says which odom reading the saved grid was
        # drawn around. Restoring the map without it would put the car back inside a wall.
        try:
            with open(self.bias_file, encoding="utf-8") as fh:
                bx, by = json.load(fh)
            if (round(bx, 3), round(by, 3)) != tuple(self.car.bias):
                self.car.set_bias(bx, by)
                self.reloc["bias"] = [float(bx), float(by)]
                self.push("已恢复上次重定位修正 (%+.2f, %+.2f) m" % (bx, by), "map")
        except Exception:
            pass

    def _save_bias(self):
        try:
            with open(self.bias_file, "w", encoding="utf-8") as fh:
                json.dump([round(float(b), 4) for b in self.car.bias], fh)
        except OSError:
            pass

    def _save_origin(self):
        try:
            with open(self.origin_file, "w", encoding="utf-8") as fh:
                json.dump(list(self.origin), fh)
        except OSError:
            pass

    # ------------------------------------------------------- places (语义记忆) ----
    def _load_places(self):
        try:
            with open(self.places_file, encoding="utf-8") as fh:
                rows = json.load(fh)
            self.places = [{"label": str(r["label"])[:PLACE_LABEL_MAX],
                            "x": float(r["x"]), "y": float(r["y"]),
                            "yaw": float(r.get("yaw", 0.0))}
                           for r in rows if isinstance(r, dict) and r.get("label")]
        except Exception:
            self.places = []

    def _save_places(self):
        try:
            with open(self.places_file, "w", encoding="utf-8") as fh:
                json.dump(self.places, fh, ensure_ascii=False)
        except OSError:
            pass

    def add_place(self, label):
        """Anchor a word to where the car stands now, in the grid's own frame."""
        label = (label or "").strip()[:PLACE_LABEL_MAX]
        if not label:
            return "空的标签，没记。"
        with self.car.lock:
            pose = self.car.pose
        if pose is None:
            return "位姿还没有（%s 从未到过），不能把一个词钉到地图上。" % ODOM_TOPIC
        x, y, yaw = pose[0], pose[1], pose[2]
        with self.lock:
            old = [p for p in self.places if p["label"] == label]
            self.places = [p for p in self.places if p["label"] != label]
            if len(self.places) >= PLACE_MAX:
                return "已经记了 %d 个地点，先 Forget 掉没用的再记新的。" % PLACE_MAX
            self.places.append({"label": label, "x": round(x, 3), "y": round(y, 3),
                                "yaw": round(yaw, 3)})
            self._save_places()
            self.map_cache = {}             # the picture now has one more mark on it
        self.push("记住「%s」在地图 (%+.2f, %+.2f) m%s" % (
            label, x, y, "（覆盖旧的）" if old else ""), "map")
        return "已记「%s」= 地图 (%+.2f, %+.2f) m。它会和这张图一起存在，清空地图时一并删除。" % (
            label, x, y)

    def del_place(self, label):
        label = (label or "").strip()
        with self.lock:
            hit = [p for p in self.places if p["label"] == label] or (
                [p for p in self.places if label and label in p["label"]] if len(label) >= 2 else [])
            if not hit:
                return "没找到「%s」。现在记着：%s" % (
                    label, "、".join(p["label"] for p in self.places) or "一个都没有")
            self.places = [p for p in self.places if p not in hit]
            self._save_places()
            self.map_cache = {}
        self.push("已忘记 %d 个地点：%s" % (len(hit), "、".join(p["label"] for p in hit)), "map")
        return "已忘记「%s」（剩下 %d 个）" % ("」「".join(p["label"] for p in hit), len(self.places))

    def places_near(self, pose, limit=4):
        """Remembered places ordered by distance, with body-frame bearing (正=向左) -- the same
        convention Move speaks, so the model can act on the answer without converting."""
        if pose is None or not self.places:
            return []
        rows = []
        with self.lock:
            items = list(self.places)
        ox, oy = self.origin
        for p in items:
            dx, dy = p["x"] - pose[0], p["y"] - pose[1]
            rng = math.hypot(dx, dy)
            rel = math.degrees(math.atan2(dy, dx) - pose[2])
            rows.append({"label": p["label"], "dist": round(rng, 2),
                         "bearing": round((rel + 180.0) % 360.0 - 180.0, 0),
                         "map": [round(p["x"] - ox, 2), round(p["y"] - oy, 2)]})
        rows.sort(key=lambda r: r["dist"])
        return rows[:limit]

    def _save_grid(self):
        now = time.monotonic()
        if now - self.last_save < 5.0:
            return
        self.last_save = now
        lo = self.world.grid().copy()
        try:
            # Write-then-replace: a kill during a plain np.save leaves a truncated file behind,
            # and the next start then dies on it.
            tmp = self.cache_file + ".tmp"
            with open(tmp, "wb") as fh:        # a path would make numpy append ".npy" to it
                np.save(fh, lo)
            os.replace(tmp, self.cache_file)
        except OSError as exc:
            self.push("地图存盘失败: %s" % exc, "error")

    # -------------------------------------------------------------- views ----
    def map_view(self):
        """The global picture, in one text block: where the car sits inside the map it built,
        what is still unmeasured and in which direction, and whether the lidar still agrees with
        the claimed pose.

        This exists because every tool the local model could call was local -- `GetStatus`,
        `ScanRing`, `CheckAhead` answer "what is around me", and the map the Pilot plans against
        was only ever visible to a human on :8010. A planner that cannot ask "how much of the room
        is left, and which way" cannot schedule an exploration, only repeat one.
        Read-only, and safe to call while a tour runs.
        """
        with self.car.lock:
            pose = self.car.pose
            bias = list(self.car.bias)
            volts = self.car.volts
        if pose is None:
            return "底盘离线：从未收到 %s，没有位姿也就没有地图。" % ODOM_TOPIC
        lo = self.world.grid()
        res = self.world.res
        x, y, yaw = pose[0], pose[1], pose[2]
        ox, oy = self.origin
        occ = int((lo >= 0.65).sum())
        free = int((lo <= -2.0).sum())
        known = occ + free
        walls = int((lo >= nav.MATCH_WALL).sum())
        with self.lock:
            rl = dict(self.reloc)
            dd = dict(self.drift)
            mt = {k: v for k, v in self.motion.items() if k != "last"}
            busy, mapping, touring = self.busy, self.mapping, bool(self.tour)
        out = ["地图：%d 格已知（障碍 %d + 空地 %d，约 %.1f m²），折叠 %d 帧，分辨率 %.2f m，建图%s"
               % (known, occ, free, known * res * res, self.world.frames, res,
                  "开" if mapping else "关（等人确认位置）")]
        out.append("我在地图系 (%.2f, %.2f) m，朝向 %.0f°（里程计原始 (%.2f, %.2f)，"
                   "重定位修正 (%+.2f, %+.2f)）；位姿 %.1f 秒前，电池 %s V"
                   % (x - ox, y - oy, math.degrees(yaw), x, y, bias[0], bias[1],
                      time.monotonic() - pose[3], "%.2f" % volts if volts else "?"))
        # The three numbers that say whether the pose may be trusted *right now*, straight from
        # the detector that would stop the car: agreement at the claimed pose, the best any
        # offset achieves, and how much of the sweep falls on ground nobody has measured.
        if rl.get("agree") is not None:
            out.append("雷达↔地图：吻合 %.0f%%，范围内最好 %.0f%% 在 (%+.1f, %+.1f) m，"
                       "支撑 %d 点，图上墙 %d 格，看穿假墙的回波占 %s%s"
                       % (100 * rl["agree"], 100 * (rl["best"] or 0), rl["dx"] or 0,
                          rl["dy"] or 0, rl.get("support") or 0, walls,
                          "-" if rl.get("stale") is None else
                          ("当前 %.0f%%、匹配器首选 %.0f%%" % (100 * rl["stale"][0],
                                                          100 * rl["stale"][1])),
                          "，已冻结待人工确认" if rl["frozen"] else ""))
            if rl.get("why"):
                out.append("  判定：%s" % rl["why"])
        elif rl.get("why"):
            out.append("雷达↔地图：%s" % rl["why"])
        out.append("漂移抑制：已补 %d 次共 %.2f m，此刻估计仍差 (%+.2f, %+.2f) m"
                   % (dd["n"], dd["total"], dd["est"][0], dd["est"][1]))
        if mt["at"] is not None:
            out.append("运动中定位检查：上次 %.1f 秒前，未测回波占 %.0f%%，连续可疑 %d 次，"
                       "已中止 %d 次（%s）"
                       % (time.monotonic() - mt["at"], 100 * (mt["outside"] or 0), mt["bad"],
                          mt["n"], mt["why"] or "正常"))
        f = nav.frontier_view(self.world, pose)
        if f is None:
            out.append("前沿：车在地图边界外，无法判断还剩什么没看。")
        elif f["in_wall"]:
            out.append("前沿：车的当前位置在图上读作墙里 —— 位姿或地图有一个是错的，"
                       "先停下来重新定位，别再往前走。")
        elif not f["frontier"]:
            out.append("前沿：可达范围内已经没有未测格子，这块地图能看的都看完了；"
                       "要继续扩图得换一个起点重新建。")
        else:
            # The 12-sector histogram is the part that turns "there is unknown stuff" into an
            # instruction. Bearings are body-frame and positive = left, which is the one thing
            # the Move tool already speaks; the histogram starts at 正后方 and runs
            # counter-clockwise, so index len//2 is dead ahead.
            hist = f["sectors"]
            span = 360.0 / len(hist)
            top = max(range(len(hist)), key=lambda i: hist[i])
            centre = -180.0 + span * (top + 0.5)
            out.append("前沿：还有 %d 格没测到，最近一处在 %.1f m、%+.0f°（正=向左）；"
                       "未测最集中的方位 %+.0f°（该扇区 %d 格）。"
                       "每 30° 一格、从正后开始逆时针：%s（第 %d 格是正前）"
                       % (f["frontier"], f["nearest"], f["bearing"], centre, hist[top],
                          " ".join(str(h) for h in hist), len(hist) // 2))
        near = self.places_near(pose)
        if near:
            out.append("记住的地点（共 %d 个，按远近）：%s；问「XX 在哪」就答这个，"
                       "要说新地点用 Remember，说不要了用 Forget"
                       % (len(self.places), "，".join(
                           "「%s」%.1f m %+.0f°（显示系 %.2f, %.2f）"
                           % (r["label"], r["dist"], r["bearing"], r["map"][0], r["map"][1])
                           for r in near)))
        elif self.places:
            out.append("记住的地点：%d 个，但都不在当前位姿的可见范围内。" % len(self.places))
        out.append("电机：%s" % (("被 %s 占用" % busy) if busy else
                                ("巡游中" if touring else "空闲")))
        return "\n".join(out)

    def state(self):
        with self.car.lock:
            p, raw, ptz = self.car.pose, self.car.pose_raw, list(self.car.ptz)
            volts, bias = self.car.volts, list(self.car.bias)
        with self.lock:
            v, w, deadline, busy = self.teleop[0], self.teleop[1], self.teleop[2], self.busy
            rl = dict(self.reloc)
            dd = {k: v for k, v in self.drift.items() if k != "last"}
            mt = {k: v for k, v in self.motion.items() if k != "last"}
        lo = self.world.grid()
        ox, oy = self.origin
        age = self.car.scan_age()
        rl["bias"] = [round(bias[0], 3), round(bias[1], 3)]
        # Read from the car rather than from self.drift: a human choosing a relocation candidate
        # or clearing the map moves the bias without the suppressor being involved at all, and a
        # stale number here would report corrections that no longer exist.
        dd["bias"] = rl["bias"]
        with self.lock:
            spots = list(self.places)
        # Numbered in exactly the order render_map stamps onto the picture, so the list beside
        # the map and the magenta dots in it are the same objects, not two guesses.
        places = [{"n": i + 1, "label": sp["label"],
                   "map": [round(sp["x"] - ox, 2), round(sp["y"] - oy, 2)],
                   "dist": None if p is None else round(math.hypot(sp["x"] - p[0], sp["y"] - p[1]), 2)}
                  for i, sp in enumerate(spots)]
        return {
            "pose": None if p is None else [round(p[0] - ox, 3), round(p[1] - oy, 3),
                                            round(math.degrees(p[2]), 1)],
            # raw is what the board's wheel odometry says, with the correction left out: the gap
            # between the two *is* the carry, and hiding it would make the fix unauditable.
            "pose_raw": None if raw is None else [round(raw[0], 3), round(raw[1], 3),
                                                  round(math.degrees(raw[2]), 1)],
            "origin": [round(ox, 3), round(oy, 3)],   # odom point the printed (0,0) sits on
            "reloc": rl,
            "drift": dd,
            "motion": mt,
            "odom_age": None if p is None else round(time.monotonic() - p[3], 2),
            "scan_age": None if age is None else round(age, 2),
            "volts": None if volts is None else round(volts, 2),
            "ptz": [round(ptz[0], 1), round(ptz[1], 1)],
            "imu": (None if self.car.imu is None else
                    [round(self.car.imu[0], 3), round(time.monotonic() - self.car.imu[1], 2)]),
            # What the bridge's two heading sensors disagree by. Non-zero and recent means the
            # pose this frame is drawn at is partly invented, and the sweep is being held out.
            "odom_lie": [round(self.car.yaw_lie, 1),
                         round(max(0.0, time.monotonic() - self.car.yaw_lie_t), 1)],
            "odom_slip": [round(self.car.slip_m, 2),
                          round(max(0.0, time.monotonic() - self.car.slip_t), 1)],
            "ring": self.car.sector_ring(24),
            "teleop": {"v": round(v, 3), "w": round(w, 3),
                       "active": time.monotonic() < deadline},
            "busy": busy, "mapping": self.mapping, "frames": self.world.frames,
            "occ": int((lo >= 0.65).sum()), "free": int((lo <= -2.0).sum()),
            "samples": len(self.samples), "gains": gains(),
            # Derived, not fitted: the band the wheels ignore (nav.ACT_W_CUTON, measured) and the
            # smallest turn command any loop will bother sending. Kept out of gains() so
            # nav_gains.json only ever stores things a human might re-fit.
            "chassis": {"cuton": nav.ACT_W_CUTON, "turn_floor": round(nav.turn_floor(), 3),
                        "dead_eff": round(max(nav.ACT_W_DEAD, nav.ACT_W_CUTON), 3),
                        "dead_eff_neg": round(max(nav.ACT_W_DEAD_NEG, nav.ACT_W_CUTON), 3)},
            "caps": {"v": CAP_V, "w": CAP_W}, "tour": bool(self.tour),
            "places": places,
            "cam": ("http://%s:%d/stream" % (self.args.camera_ip, self.args.camera_port)
                    if self.args.camera_ip else ""),
            "log": list(self.log)[:40], "uptime": round(time.monotonic() - self.started, 1),
            "geom": self.map_geom,
        }

    def scan_json(self):
        with self.car.lock:
            pairs = list(self.car.pairs)
        return {"pairs": [[round(a, 1), round(d, 3)] for a, d in pairs],
                "age": self.car.scan_age()}

    def set_ptz(self, yaw, pitch, raw=None, absolute=False):
        if raw is not None:
            # Raw servo degrees bypass the user-facing limits: the pitch reference on this
            # board is not the value the docs claim, so the only way to find where the camera
            # actually looks is to drive the servo directly and take a frame at each angle.
            sx = max(20.0, min(150.0, float(raw[0])))
            sy = None if raw[1] is None else max(-135.0, min(135.0, float(raw[1])))
            np_ = PTZ_SERVO_PITCH_LEVEL - sx
            ny = (PTZ_SERVO_YAW_CENTRE - sy) if sy is not None else self.car.ptz[0]
            np_ = max(-70.0, min(70.0, np_))
        else:
            with self.car.lock:
                cur = list(self.car.ptz)
            ny, np_ = ((float(yaw), float(pitch)) if absolute
                       else (cur[0] + float(yaw), cur[1] + float(pitch)))
            ny = max(PTZ_YAW_LIMIT[0], min(PTZ_YAW_LIMIT[1], ny))
            np_ = max(PTZ_PITCH_LIMIT[0], min(PTZ_PITCH_LIMIT[1], np_))
        with self.car.lock:
            self.car.ptz = [ny, np_]
        msg = Vector3()
        msg.x = PTZ_SERVO_PITCH_LEVEL - np_
        msg.y = PTZ_SERVO_YAW_CENTRE - ny
        msg.z = 0.0
        for _ in range(5):
            self.car.ptz_pub.publish(msg)
            time.sleep(0.03)
        return "云台 servo=(%.0f, %.0f) yaw=%.1f pitch=%.1f" % (msg.x, msg.y, ny, np_)

    def beep(self, ms):
        n = UInt16()
        n.data = int(max(20, min(3000, int(ms))))
        for _ in range(3):
            self.car.beep_pub.publish(n)
            time.sleep(0.02)
        return "蜂鸣 %d ms" % n.data

    def set_origin(self, reset=False):
        """Re-anchor the printed coordinate frame without touching the map or the odometry.

        Translation only, on purpose: the grid was built in the odom frame, and rotating the
        numbers without resampling the picture would put the map's north and the printed
        heading out of step -- the exact kind of disagreement that makes a coordinate readout
        worthless. So 朝向 always stays the raw odom heading.
        """
        with self.car.lock:
            p = self.car.pose
        if reset:
            self.origin = (0.0, 0.0)
            self.map_cache = {}
            self._save_origin()
            self.push("已还原到里程计原点，即这次上电时车所在的位置", "origin")
            return "ok"
        if p is None:
            return "失败: 没有里程计"
        self.origin = (p[0], p[1])
        self.map_cache = {}
        self._save_origin()
        self.push("已把当前位置设为原点：地图内容不变，只有刻度数字跟着平移；"
                  "里程计真实原点在显示坐标 (%.2f, %.2f)" % (-p[0], -p[1]), "origin")
        return "ok"

    def start_tour(self, b):
        if not self.claim("tour"):
            return "失败: 电机正被 %s 占用" % self.busy
        secs = float(b.get("seconds", 120))
        out_dir = str(b.get("out_dir", ""))

        def run():
            try:
                self.push("Tour 开始（预算 %.0f 秒）" % secs, "tour")
                txt = self.car.drive_tour(secs, out_dir, self.world, self.trail)
                self.push("Tour 结束: " + txt, "tour")
            except Exception as exc:
                self.push("Tour 异常: %s: %s" % (type(exc).__name__, exc), "error")
            finally:
                self.last_save = 0.0
                self._save_grid()
                self.tour = None
                self.release("tour")

        self.car.nav_stop = False
        self.tour = {"seconds": secs}
        threading.Thread(target=run, daemon=True).start()
        return "Tour 已后台启动，地图实时刷新，急停可中断"

    # ------------------------------------------------------------ render ----
    def render_map(self, half_m, with_lidar, center="car", axes=True):
        world = self.world
        lo = world.grid()
        n, res, ox, oy = world.n, world.res, world.ox, world.oy
        with self.car.lock:
            pose = self.car.pose
            pairs = list(self.car.pairs) if with_lidar else []
        if pose is None:
            pose = (0.0, 0.0, 0.0)
        odx, ody = self.origin
        dpose = (pose[0] - odx, pose[1] - ody, pose[2])
        # Which world point sits in the middle of the picture. "car" is what you want while
        # driving; "odom" puts the printed origin in the middle and grows the window just enough
        # to keep the car inside it, which is the only view where printed coordinates can be
        # checked against the room. Square window in metres either way: clamping each axis
        # separately makes the picture rectangular, which reads as a different y scale.
        if center == "odom":
            mx, my = odx, ody
            half_m = max(half_m, math.hypot(dpose[0], dpose[1]) + 0.6)
        else:
            mx, my = pose[0], pose[1]
        cx, cy = int((mx - ox) / res), int((my - oy) / res)
        k = max(8, int(half_m / res))

        def win(centre, k):
            """Window of 2k+1 cells around `centre`, slid back inside the grid instead of clipped."""
            a, b = centre - k, centre + k + 1
            if a < 0:
                a, b = 0, min(n, b - a)
            if b > n:
                a, b = max(0, a - (b - n)), n
            return a, b

        c0, c1 = win(cx, k)
        r0, r1 = win(cy, k)
        sub = lo[r0:r1, c0:c1]
        img = np.zeros(sub.shape + (3,), np.uint8)
        img[:] = (52, 56, 62)                       # never measured
        img[sub <= 0.4] = (232, 234, 236)           # free
        img[sub >= 0.65] = (18, 18, 20)             # occupied
        img[(sub > 0.4) & (sub < 0.65)] = (150, 128, 70)
        img = np.flipud(img)

        def put(px, py, col, rad=0):
            c = int((px - ox) / res) - c0
            r = (r1 - r0 - 1) - (int((py - oy) / res) - r0)
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    if 0 <= r + dr < img.shape[0] and 0 <= c + dc < img.shape[1]:
                        img[r + dr, c + dc] = col

        for tx, ty, _ in list(self.trail)[-1200:]:
            put(tx, ty, (60, 190, 120))
        for a, d in pairs:
            ang = pose[2] + math.radians(a)
            put(pose[0] + d * math.cos(ang), pose[1] + d * math.sin(ang), (60, 200, 255))
        t = res
        while t < max(0.5, half_m * 0.12):
            hx, hy = pose[0] + t * math.cos(pose[2]), pose[1] + t * math.sin(pose[2])
            c = int((hx - ox) / res) - c0
            r = (r1 - r0 - 1) - (int((hy - oy) / res) - r0)
            if 0 <= r < img.shape[0] and 0 <= c < img.shape[1]:
                img[r, c] = (255, 210, 0)
            t += res / 4.0
        with self.lock:
            spots = list(self.places)
        # Places go under the car marker on purpose: a note pinned to the spot the car is
        # standing on is the common case, and it must not erase "where am I".
        for p in spots:
            put(p["x"], p["y"], PLACE_COL, 2)
        put(pose[0], pose[1], (235, 60, 60), 3)
        up = np.repeat(np.repeat(img, 3, axis=0), 3, axis=1)
        if axes:
            draw_axes(up, ox, oy, res, c0, r1, dpose, (odx, ody))
        # Number each dot the same way state()["places"] and map_view enumerate them, so the
        # list beside the picture, the sentence the model reads out, and the pixels agree.
        mpp3 = res / 3.0
        x_min3, y_top3 = ox + c0 * res - odx, oy + r1 * res - ody
        for i, p in enumerate(spots):
            px = int(round((p["x"] - odx - x_min3) / mpp3))
            py = int(round((y_top3 - (p["y"] - ody)) / mpp3))
            if 0 <= px < up.shape[1] and 0 <= py < up.shape[0]:
                ax_text(up, min(max(0, px - 6), up.shape[1] - 8),
                        min(max(0, py - 14), up.shape[0] - 10),
                        str(i + 1), PLACE_COL, 2)
        # Everything a viewer needs to place its own marks is published in the printed frame,
        # so an overlay and the picture it sits on can never disagree about where zero is.
        self.map_geom = {"x_min": ox + c0 * res - odx, "y_max": oy + r1 * res - ody,
                         "mpp": res / 3.0,
                         "w": int(img.shape[1] * 3), "h": int(img.shape[0] * 3),
                         "pose": [round(dpose[0], 3), round(dpose[1], 3),
                                  round(math.degrees(dpose[2]), 1)],
                         "home": [round(odx, 3), round(ody, 3)],
                         "grid": [ox - odx, oy - ody, n * res]}
        return png_bytes(up)

    def map_png(self, half_m, with_lidar, center="car", axes=True):
        now = time.monotonic()
        # Cache per window, not per instant: with a time-only cache a request for the wide view
        # got handed the narrow frame rendered 200 ms earlier, so 全图 looked like it did nothing.
        key = (round(half_m, 2), bool(with_lidar), center, bool(axes))
        cached = self.map_cache.get(key)
        if cached is not None and now - cached[1] < 0.45:
            return cached[0]
        body = self.render_map(half_m, with_lidar, center, axes)
        if len(self.map_cache) > 8:
            self.map_cache.clear()
        self.map_cache[key] = (body, now)
        return body

    # ------------------------------------------------------- calibration ----
    def fit(self, apply=False):
        """Least squares on the recorded samples: achieved = gain * commanded (+ stiction).

        Only bursts long enough to have left the ramp count. The board holds ~0.2-0.3 s of
        queued twist and accelerates into it, so a 1 s teleop burst averages a motion that never
        reached speed: a pool of 19 such samples fit to gain_v 0.283 / gain_w 0.015 against the
        0.82 / 0.857 the 6-8 s sweeps give. The short ones are not bad data, they measure a
        different quantity.
        """
        s = list(self.samples)
        keep = [c for c in s if c[4] >= FIT_MIN_DT]
        lin = [(c[0], c[2]) for c in keep if abs(c[1]) < 0.02 and abs(c[0]) > 0.03]
        ang = [(c[1], c[3]) for c in keep if abs(c[0]) < 0.02 and abs(c[1]) > 0.05]
        out = {"n_linear": len(lin), "n_angular": len(ang),
               "n_total": len(s), "n_short_dropped": len(s) - len(keep),
               "min_dt": FIT_MIN_DT}
        if lin:
            a = np.array(lin, float)
            out["gain_v"] = round(float((a[:, 0] * a[:, 1]).sum() / (a[:, 0] ** 2).sum()), 3)
            out["v_rmse"] = round(float(np.sqrt(((a[:, 0] * out["gain_v"] - a[:, 1]) ** 2)
                                                .mean())), 4)
        if ang:
            # Left and right are two motors behind two PWM paths, and the vendor's own sheet
            # calibrates them apart (left 1.0708 / right 1.0863). Fit them apart too, but only
            # believe a side once it has ANG_MIN_SIDE samples of its own -- below that a pooled
            # fit, or the other half mirrored, beats a two-point line through the noise.
            pos = [c for c in ang if c[0] >= 0.0]
            neg = [c for c in ang if c[0] < 0.0]
            ccw = ang_fit(pos) if len(pos) >= ANG_MIN_SIDE else None
            cw = ang_fit(neg, "_neg") if len(neg) >= ANG_MIN_SIDE else None
            split = ccw is not None and cw is not None
            if ccw is None and cw is not None:
                ccw = {k[:-4]: v for k, v in cw.items()}
            elif cw is None and ccw is not None:
                cw = {k + "_neg": v for k, v in ccw.items()}
            elif ccw is None:
                ccw = ang_fit(ang)
                cw = {k + "_neg": v for k, v in ccw.items()}
            out.update({"n_ccw": len(pos), "n_cw": len(neg), "split": split})
            out.update(ccw)
            out.update(cw)
        out["current"] = gains()
        # The voltage the samples were taken at, not the voltage right now: the pool can be
        # refilled from disk hours later (see _load_samples), and "now" would then anchor the
        # volt scale to a pack reading that has nothing to do with these rows.
        vols = [c[6] for c in keep if len(c) > 6 and c[6]]
        ref = round(sum(vols) / len(vols), 2) if vols else self.car.volts
        out["volt_ref_now"] = ref
        out["volt_n"] = len(vols)
        if apply:
            out["volt_ref"] = ref
            out["applied"] = apply_gains(out)
            self.push("已按 %d 条直线 + %d 条转向样本（CCW %d / CW %d，%s）套用拟合增益"
                      " v=%.3f 左 gain=%.3f 偏置=%.3f 死区=%.3f 右 gain=%.3f 偏置=%.3f"
                      " 死区=%.3f，基准电压 %s V"
                      % (len(lin), len(ang), out.get("n_ccw", 0), out.get("n_cw", 0),
                         "左右分别" if out.get("split") else "共用一组",
                         nav.ACT_GAIN_V, nav.ACT_GAIN_W, nav.ACT_W_BIAS, nav.ACT_W_DEAD,
                         nav.ACT_GAIN_W_NEG, nav.ACT_W_BIAS_NEG, nav.ACT_W_DEAD_NEG,
                         ("%.2f" % nav.ACT_VOLT_REF) if nav.ACT_VOLT_REF else "未知"),
                      "gain")
        return out

    def record_feedback(self, note, truth=None):
        e = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "note": str(note)}
        with self.car.lock:
            p = self.car.pose
        if p:
            e["odom"] = [round(p[0], 3), round(p[1], 3), round(math.degrees(p[2]), 1)]
        if truth:
            e["truth"] = truth
        with open(FEEDBACK_FILE, "a", encoding="utf-8") as f:
            f.write(json.dumps(e, ensure_ascii=False) + "\n")
        self.push("用户反馈已记录：%s" % note, "user")
        return e


def gains():
    return {"gain_v": nav.ACT_GAIN_V, "gain_w": nav.ACT_GAIN_W,
            "w_bias": nav.ACT_W_BIAS, "w_dead": nav.ACT_W_DEAD,
            "gain_w_neg": nav.ACT_GAIN_W_NEG, "w_bias_neg": nav.ACT_W_BIAS_NEG,
            "w_dead_neg": nav.ACT_W_DEAD_NEG, "volt_ref": nav.ACT_VOLT_REF,
            "max_v": nav.MAX_V, "max_w": nav.MAX_W}


def ang_fit(rows, suffix=""):
    """One sign of the turn sweep: |w_act| = gain*|w_cmd| - bias. Returns suffixed keys."""
    a = np.array([[abs(c[0]), -1.0] for c in rows], float)
    y = np.array([abs(c[1]) for c in rows], float)
    coef, *_ = np.linalg.lstsq(a, y, rcond=None)
    g, bias = float(coef[0]), float(coef[1])
    return {"gain_w" + suffix: round(g, 3),
            "w_bias" + suffix: round(bias, 3),
            "w_dead" + suffix: round(max(0.0, bias / g), 3) if g > 0 else None,
            "w_rmse" + suffix: round(float(np.sqrt(((a @ coef - y) ** 2).mean())), 4)}


# Turning a corner needs at least this many samples of that corner before the fit is a
# measurement rather than two points and a prayer.
ANG_MIN_SIDE = 3

# Shortest burst (s) that counts as a measurement of the steady response -- see Console.fit.
FIT_MIN_DT = 2.5

# Rows of calib_samples.jsonl to read back at startup (the deque holds the same amount).
SAMPLE_CAP = 4000


def apply_gains(vals):
    for key, attr in (("gain_v", "ACT_GAIN_V"), ("gain_w", "ACT_GAIN_W"),
                      ("w_bias", "ACT_W_BIAS"), ("w_dead", "ACT_W_DEAD"),
                      ("gain_w_neg", "ACT_GAIN_W_NEG"), ("w_bias_neg", "ACT_W_BIAS_NEG"),
                      ("w_dead_neg", "ACT_W_DEAD_NEG"), ("volt_ref", "ACT_VOLT_REF"),
                      ("max_v", "MAX_V"), ("max_w", "MAX_W")):
        if key in vals and vals[key] is not None:
            setattr(nav, attr, float(vals[key]))
    keep = {k: v for k, v in gains().items() if v is not None}
    with open(GAINS_FILE, "w", encoding="utf-8") as f:
        json.dump(keep, f, indent=1, sort_keys=True)
    return keep


def load_gains():
    """Restore nav_gains.json. Old files carry one angular gain for both directions; the
    clockwise half inherits it instead of being left at the factory constant."""
    if not os.path.exists(GAINS_FILE):
        return False
    try:
        vals = json.load(open(GAINS_FILE, encoding="utf-8"))
    except (ValueError, OSError):
        return False
    if "gain_w" in vals and "gain_w_neg" not in vals:
        for key in ("gain_w", "w_bias", "w_dead"):
            if key in vals:
                vals[key + "_neg"] = vals[key]
    try:
        apply_gains(vals)
        return True
    except (ValueError, OSError):
        return False


def png_bytes(rgb):
    import struct
    import zlib
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[r].tobytes() for r in range(h))

    def chunk(tag, data):
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 4)) + chunk(b"IEND", b""))


def serve(car, args, console=None):
    """Start the console on its own port inside this process; returns the Console."""
    con = console or Console(car, args)
    threading.Thread(target=con._stream, daemon=True).start()
    threading.Thread(target=con._fold, daemon=True).start()
    html_path = os.path.join(HERE, "car_web_ui.html")
    outer = con

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json", extra=None):
            if isinstance(body, str):
                body = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            if body:
                self.wfile.write(body)

        def _body(self):
            try:
                n = int(self.headers.get("Content-Length", 0))
                return json.loads(self.rfile.read(n).decode("utf-8")) if n else {}
            except (ValueError, UnicodeDecodeError):
                return {}

        # ---- GET ----------------------------------------------------------
        def do_GET(self):
            path, _, qs = self.path.partition("?")
            q = dict(kv.split("=", 1) for kv in qs.split("&") if "=" in kv)
            try:
                if path in ("/", "/index.html"):
                    self._send(200, open(html_path, "rb").read(), "text/html; charset=utf-8")
                elif path == "/api/state":
                    self._send(200, json.dumps(outer.state(), ensure_ascii=False))
                elif path == "/api/drift":
                    with outer.lock:
                        rows = list(outer.drift_hist)
                        now = dict(outer.drift)
                    self._send(200, json.dumps({"now": now, "hist": rows}))
                elif path == "/api/scan":
                    self._send(200, json.dumps(outer.scan_json()))
                elif path == "/api/map.png":
                    self._send(200, outer.map_png(float(q.get("half", 4.0)),
                                                  q.get("lidar", "1") != "0",
                                                  q.get("center", "car"),
                                                  q.get("axes", "1") != "0"), "image/png")
                elif path == "/api/shot":
                    try:
                        data = grab_jpeg(args.camera_ip, args.camera_port,
                                         min(4.0, args.camera_timeout))
                    except Exception as exc:
                        return self._send(502, "摄像头不可用: %s" % exc, "text/plain")
                    self._send(200, data or b"", "image/jpeg")
                elif path == "/api/log":
                    self._send(200, json.dumps({"log": list(outer.log)[:120],
                                                "samples": len(outer.samples)}))
                else:
                    self._send(404, "{}")
            except BrokenPipeError:
                pass
            except Exception as exc:
                outer.push("GET %s 失败: %s" % (path, exc), "error")
                self._send(500, '{"error":"%s"}' % type(exc).__name__)

        # ---- POST ---------------------------------------------------------
        def do_POST(self):
            b = self._body()
            path = self.path.partition("?")[0]
            try:
                if path == "/api/teleop":
                    ok = outer.set_teleop(b.get("v", 0), b.get("w", 0), b.get("hold", 0.3))
                    self._send(200, '{"ok":%s}' % ("true" if ok else "false"))
                elif path == "/api/stop":
                    outer.stop_all("human")
                    self._send(200, '{"ok":true}')
                elif path == "/api/ptz":
                    raw = None
                    if "x" in b:
                        raw = (b["x"], b.get("y"))
                    self._send(200, json.dumps(
                        {"text": outer.set_ptz(b.get("yaw", 0), b.get("pitch", 0), raw,
                                               bool(b.get("abs")))},
                        ensure_ascii=False))
                elif path == "/api/mapping":
                    if "on" in b:
                        if b["on"]:
                            outer.resume_mapping()    # also re-arms the carry detector
                        else:
                            outer.mapping = False
                    if b.get("reset"):
                        outer.reset_map()
                    self._send(200, json.dumps({"mapping": outer.mapping}))
                elif path == "/api/reloc":
                    if b.get("dx") is not None or b.get("dy") is not None:
                        outer.apply_offset(float(b.get("dx", 0.0)), float(b.get("dy", 0.0)))
                    self._send(200, json.dumps({"bias": [round(x, 3) for x in outer.car.bias],
                                                "mapping": outer.mapping}, ensure_ascii=False))
                elif path == "/api/origin":
                    txt = outer.set_origin(bool(b.get("reset")))
                    self._send(200, json.dumps({"ok": txt == "ok", "text": txt,
                                                "origin": [round(v, 3) for v in outer.origin]}))
                elif path == "/api/calib":
                    if not outer.claim("calib"):
                        return self._send(409, '{"error":"电机被占用"}')
                    try:
                        txt = outer.car.rate_test(float(b.get("vx", 0)), float(b.get("wz", 0)),
                                                  float(b.get("seconds", 4)))
                        outer.add_calib_sample(getattr(outer.car, "last_calib", None))
                    finally:
                        outer.release("calib")
                    outer.push(txt, "meas")
                    self._send(200, json.dumps({"text": txt}, ensure_ascii=False))
                elif path == "/api/tour":
                    self._send(200, json.dumps({"text": outer.start_tour(b)},
                                               ensure_ascii=False))
                elif path == "/api/gains":
                    self._send(200, json.dumps(apply_gains(b)))
                elif path == "/api/fit":
                    self._send(200, json.dumps(outer.fit(bool(b.get("apply")))))
                elif path == "/api/calib_clear":
                    self._send(200, json.dumps(outer.clear_samples()))
                elif path == "/api/feedback":
                    outer.record_feedback(b.get("note", ""), b.get("truth"))
                    self._send(200, '{"ok":true}')
                elif path == "/api/beep":
                    self._send(200, json.dumps({"text": outer.beep(b.get("ms", 200))},
                                               ensure_ascii=False))
                elif path == "/api/places":
                    # The same two tools the model calls, on a button: a human standing in the
                    # room is better at naming things than any of the sensors.
                    if b.get("forget") is not None:
                        txt = outer.del_place(str(b.get("forget")))
                    else:
                        txt = outer.add_place(str(b.get("label", "")))
                    self._send(200, json.dumps({"text": txt}, ensure_ascii=False))
                else:
                    self._send(404, "{}")
            except BrokenPipeError:
                pass
            except Exception as exc:
                outer.push("POST %s 失败: %s" % (path, exc), "error")
                self._send(500, '{"error":"%s"}' % type(exc).__name__)

    httpd = ThreadingHTTPServer(("0.0.0.0", args.web_port), Handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print("控制台: http://127.0.0.1:%d/  (MCP 仍在 /:%d)" % (args.web_port, args.port))
    # The MCP MapInfo tool reads the growing grid through this back-reference: the map lives in
    # the Console, but the tools are built around the Car, and build_server runs before serve().
    car.console = con
    return con
