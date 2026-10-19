# -*- coding: utf-8 -*-
"""Offline acceptance test for car_web._motion_watch -- step 4: localisation checked *while rolling*.

Nothing here needs the car, a bridge or a lidar. The method under test is reached the same way
_drift_test.py reaches the controller: bind the real `Console._motion_watch` to a stub that has
exactly the attributes it reads. That matters more than usual here, because the interesting part
of this detector is not the maths (nav.scan_offset, already tested in _reloc_test.py) but *when it
is allowed to pull the motors* -- and every one of those permissions is a branch.

The branches, in the order they can bite:
  1  carried while a command owns the motors  -> stop, and say why to the command
  2  driving normally on its own map          -> stay silent
  3  driving into ground nobody has measured  -> stay silent (this is exploration working)
  4  a human has the stick                    -> warn, never take the wheel away from them
  5  phantom walls make a candidate *score* better and this map can still explain the sweep
     somewhere -> the veto excuses the drive. This is the alarm the veto was built for.
  5b the same drive with the see-through answer removed -> stop, proving 5 was saved by the veto
  6  the same *real* carry on a map that contradicts the sweep at every placement it can reach
     -> the veto has no standing left. Keep the pose, stop the drive, and say "can't tell".
  6b/6c the same carry across maps smeared by 8 degrees and 20 degrees of the chassis' own
     uncommanded yaw: mild ghosts still let the contrast through (normal abort), heavy ones
     land in case 6. This is the 2026-10-18 live failure -- a real +0.80 m jump reported as
     "地图画错了" -- reproduced end to end, and the reason THROUGH_JUNK exists.
  7  a *real* carry on a map two folds old: the peak is broad because hardly anything is claimed,
     so the gate that refuses to relocate onto a vague answer silently refused to stop. The grid's
     own question (0.32 -> 0.00 see-through) is what stands in for the missing sharpness -- and
     7b is the same young map driven honestly, which must stay silent.
  8  the veto fired on a map that was hardly contradicted (0.05 -> 0.06): "no better" about nothing
     is no argument, so THROUGH_ACCUSE makes the gate earn its veto.
  9  the *timing* failure, replayed: a carry four frames deep has already drawn a fiction room
     around itself, the score's margin has decayed below WANT_MARGIN and `fits` closes. The grid's
     own accusation (see-through 0.46 -> 0.21 at a placement beyond odometry noise) has to be able
     to stop the car by itself; 9b is that map with the grid silenced, which must drive on.

Run: C:\\microros-agent\\pc_side\\_rosrun.bat _motion_test.py
"""
import math
import threading

import numpy as np

import car_web
import nav
from _reloc_test import sweep


class FakeTime:
    """_motion_watch gates samples by wall clock (0.2 s) and by distance (0.15 m). The distance
    gate is what a scripted drive can satisfy; the clock one only makes the test sleep."""

    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        return self.t

    def sleep(self, s):
        pass


TIME = FakeTime()


class FakeCar:
    def __init__(self):
        self.acting = False
        self.abort_reason = ""
        self.stop_src = ""
        self.yaw_lie_t = 0.0      # last time the bridge caught odom inventing heading
        self.slip_t = 0.0         # ... and inventing distance (same hold, other witness)


class Stub:
    """Just the attributes _motion_watch reads and writes."""

    # The veto helper belongs to the real Console and only ever touches self.world, so bind it
    # rather than duplicating it -- a copy here could disagree with the thing under test.
    _through_pair = car_web.Console._through_pair
    _veto_check = car_web.Console._veto_check
    _stale_map = car_web.Console._stale_map
    _machine_stop = car_web.Console._machine_stop
    _hold_vote = car_web.Console._hold_vote

    def __init__(self, world, busy="move"):
        self.lock = threading.Lock()
        self.car = FakeCar()
        self.world = world
        self.busy = busy
        self.logs = []
        self.stops = 0
        self.seen = []
        self.holds = []
        self.stale_t = 0.0
        self.reloc = {"frozen": False, "trial": None}
        self.motion = {"why": None, "zero": None, "best": None, "outside": None, "suspect": False,
                       "dx": None, "dy": None, "bad": 0, "at": None, "last": None, "n": 0}

    def push(self, text, kind="info"):
        self.logs.append((kind, text))

    def stop_all(self, who="machine"):
        self.stops += 1
        self.car.stop_src = who    # mirrors the real signature: the channel split is what runs

    def no_veto(self):
        """Erase the see-through answer, which is what `moved` falls back to when the sweep is too
        thin to ask. Used to prove that case 5 is stopped by the veto and not by luck."""
        self._through_pair = lambda pose, pairs, r: (None, None)
        return self

    def coherent_veto(self):
        """Pin the see-through pair to a *contradicted but not hopeless* map: the claimed pose has
        30% of its rays passing through walls the grid swears are there, the candidate removes only
        10 points of that, and 20% is still below THROUGH_JUNK -- so the grid can explain the sweep
        somewhere and "this patch is drawn wrong" is a claim it is entitled to make. Exactly the
        state in which the veto is allowed to say "not a carry", and the one case 6 is not: there
        every reachable placement contradicts the sweep, and only stopping is safe."""
        self._through_pair = lambda pose, pairs, r: (0.30, 0.20)
        return self

    def vacuous_veto(self):
        """Pin the pair to a map that is barely contradicting the sweep at all (5% -> 6%): no
        improvement, but nothing to improve either. Below THROUGH_ACCUSE the veto has no subject,
        so a real carry has to reach the motors -- this is case 5 told apart by THROUGH_ACCUSE and
        nothing else."""
        self._through_pair = lambda pose, pairs, r: (0.05, 0.06)
        return self

    def drive(self, poses, true_at, n=667):
        """Feed one sweep per claimed pose; `true_at(pose)` says where the car really is.

        After each frame, ask the question `_fold` asks before it lets a sweep reach the grid, and
        keep the answer. The hold is part of this detector's job now: an abort needs two samples,
        and if the sweep between them is folded at the lying pose the map grows a room around the
        lie and the second sample measures nothing. Watching `stops` alone would miss that.
        """
        for i, p in enumerate(poses):
            real = true_at(p)
            car_web.Console._motion_watch(
                self, (p[0], p[1], math.radians(p[2])), sweep(real[0], real[1], math.radians(p[2]), n=n))
            self.seen.append(self.motion["why"])
            self.holds.append(car_web.Console._hold_vote(self))
            TIME.t += 0.5
        return self


def room():
    w = nav.World(extent=24.0, res=0.05)
    for py in (-1.0, 0.0, 1.0):
        for px in (-1.5, 0.0, 1.5):
            w.integrate((px, py, 0.0), sweep(px, py, 0.0), every=3, samples=12)
    return w


def phantom_room(centres, rad=0.6):
    """The room, plus a tight box of invented walls around every place the car has been told to
    go. Nothing in the real world is at those ranges, so every ray that leaves the car passes
    through one of them -- which is the signature `nav.through_wall` reads and the shape a tour
    grinds into furniture draws. The score cannot see it: an endpoint near *any* mapped cell
    scores, so the junk raises the peak as happily as it raises the noise floor. Measured here:
    zero 0.03, peak 0.999 at the true offset, i.e. the detector says "carried" with conviction."""
    w = room()
    lo = w.lo.copy()
    for (cx, cy) in centres:
        for i in range(-int(rad / w.res), int(rad / w.res) + 1):
            s = i * w.res
            for (dx, dy) in ((rad, s), (-rad, s), (s, rad), (s, -rad)):
                lo[int((cy + dy - w.oy) / w.res), int((cx + dx - w.ox) / w.res)] = 2.5
    w.lo = lo
    return w


def ghost_room(angle_deg=20.0):
    """The room plus a rotated copy of its own walls -- the shape a map takes when every frame was
    folded at a yaw the odometry invented (this chassis turns 34-43° during a plain `Move forward`
    with no steering commanded). Occupancy sticks while free space gets re-marked, so the ghosts
    pile into the middle of the room: the matcher then scores *anything* -- an endpoint is never
    far from a cell -- while `through_wall` finds a claimed wall in front of nearly every ray.
    That is the state in which the veto loses the right to say "not a carry": measured in
    _ghost_probe.py, one real 0.8 m carry reads 0.375% -> 0.330% see-through at 20° of ghosting
    (no contrast left, and before THROUGH_JUNK the car drove on) against 0.375% -> 0.116% at 8°
    (the carry still shows, and the abort still happens)."""
    w = room()
    th = math.radians(angle_deg)
    lo = w.lo
    occ = lo >= nav.MATCH_WALL
    ys, xs = np.nonzero(occ)
    wx = xs * w.res + w.ox
    wy = ys * w.res + w.oy
    rx = wx * math.cos(th) - wy * math.sin(th)
    ry = wx * math.sin(th) + wy * math.cos(th)
    out = lo.copy()
    out[((ry - w.oy) / w.res).astype(np.int64), ((rx - w.ox) / w.res).astype(np.int64)] = 2.5
    w.lo = out
    return w


def straight(x0, x1, step=0.2, yaw=0.0):
    out, x = [], x0
    while x <= x1 + 1e-9:
        out.append((x, 0.0, yaw))
        x += step
    return out


def thin_room(folds=2):
    """The room after only `folds` sweeps have been folded into it -- the map the car actually has
    during the first metre of its first tour. `task 32` asked for this case by name (稀疏图 + 真搬动
    = 必须停), and _thin_probe.py says why it is the hard one: 吻合 0.03, 峰值 0.97, 次优 0.94. The
    peak is not sharp because almost nothing is claimed, so the ambiguity gate that protects a
    *relocation* silently protects a lie from being noticed. Only the see-through pair separates it
    from honest exploration here (0.32 -> 0.00 for a carry, against case 3's unmapped front)."""
    w = nav.World(extent=24.0, res=0.05)
    cells = [(px, py) for py in (-1.0, 0.0, 1.0) for px in (-1.5, 0.0, 1.5)]
    for (px, py) in cells[:folds]:
        w.integrate((px, py, 0.0), sweep(px, py, 0.0), every=3, samples=12)
    return w


def half_healed(folds=3, carry=(0.8, -0.5), step=0.16, x0=-1.0):
    """An honest room with the first `folds` frames of a carry folded into it at the lying pose,
    which is the exact state the live car reached in one second: the fiction room is half drawn, so
    the score has started to believe the lie while the grid still contradicts it. _healed_probe.py
    replays it frame by frame -- 间距 0.96, 0.36, 0.32, 0.23, 0.11, 0.06 -- so from the fourth frame
    on `fits` is closed, while the see-through pair still points at the truth (0.464 -> 0.205).
    Returns (world, x of the next un-folded frame).
    """
    w = room()
    for i in range(folds):
        claim = (x0 + i * step, 0.0)
        real = (claim[0] + carry[0], claim[1] + carry[1])
        w.integrate((claim[0], claim[1], 0.0), sweep(real[0], real[1], 0.0), every=3, samples=12)
    return w, x0 + folds * step


def why_of(stub):
    """The last verdict the detector reached; "" means it stayed silent the whole way."""
    return next((w for w in reversed(stub.seen) if w), "")


def case(stub, name, expect_stop, expect_warn_only=False, expect_veto=False,
         expect_judge_cannot=False, expect_hold=None):
    ok = (stub.stops > 0) == expect_stop
    why = why_of(stub)
    print("%-46s 停车 %d 次  中止 %d 次  扣票 %d/%d  %s"
          % (name, stub.stops, stub.motion["n"], sum(stub.holds), len(stub.holds),
             "OK" if ok else "错"))
    print("     %s" % (why[:170] or "（全程无判定）"))
    if expect_stop:
        assert stub.car.abort_reason, "中止必须把原因写给正在跑的命令"
        assert not why.startswith("注意"), "机器在跑时必须真停，不能只提醒"
        # 机器自己按的停必须带着它的判词。写成"收到停止指令"就等于把地图问题记成"有人按了按钮"，
        # 然后值班的人去找一只手。
        assert "停止指令" not in stub.car.abort_reason, \
            "机器中止报成了人工急停：" + stub.car.abort_reason
        assert stub.car.stop_src == "machine", "机器停车没登记来源，正在跑的命令无法分辨谁按的"
    else:
        assert stub.car.abort_reason == "", "不该停的时候停了"
    if expect_warn_only:
        assert any("点动中" in t for _k, t in stub.logs), "点动必须只提醒，且提醒要看得见"
    if expect_veto:
        # A case that passes because the detector never reached the branch proves nothing, so
        # insist the see-through question was actually asked and actually answered here.
        assert any("看得穿" in (w or "") for w in stub.seen), "否决门根本没跑到，这个用例没有测到它"
        # ...and that the verdict reached the human, not just the internal field: the banner and
        # MapInfo both read `reloc.stale`, which is what the shared _stale_map writes.
        assert stub.reloc.get("stale"), "否决了搬动却没写下地图错了，页面上就看不见"
        assert any("画错了" in t for _k, t in stub.logs), "这条判定得进日志"
        assert "不算搬动" in why, "图能自洽时否决门就该明说不算搬动"
    if expect_judge_cannot:
        # The veto fired but this map cannot answer the question, so the machine must stop *and*
        # must not claim either cause. "不算搬动" here would be the 2026-10-18 lie.
        assert "不足以判断" in why, "图不足以判断时不能报成搬动，也不能报成没搬动：" + why
        assert "没能确认" in stub.car.abort_reason, "中止判词要写清是哪一种不确定"
        assert stub.reloc.get("stale"), "地图自己矛盾这条得进共享字段，横幅和 MapInfo 都要看得见"
        assert any("画错了" in t for _k, t in stub.logs), "图的问题要有给人类读的一条日志"
    # The abort waits for two counted samples, and the fold loop runs five times a second in
    # between. If that middle frame votes, the grid grows a room around the lie and the second
    # sample measures the room, not the carry -- the 0.71 -> 0.95 healing measured on the car.
    if expect_stop:
        assert any(stub.holds[:-1]), "可疑帧没被扣在图外：谎言折进图里，第二次判定就等不到了"
    if expect_hold is False:
        assert not any(stub.holds), "干净行驶也把票扣住了，建图会被一路拖慢"
    assert ok
    return ok


def main():
    real_time = car_web.time
    car_web.time = TIME          # the detector's clock gate is not what is under test
    try:
        w = room()
        carry = lambda p: (p[0] + 0.8, p[1] - 0.5)

        # 1 -- carried while a command owns the motors
        st = Stub(w).drive(straight(-1.0, 1.0), carry)
        case(st, "1 行进中被搬走 (0.8,-0.5)", expect_stop=True)

        # 2 -- driving normally: the sweep is the map's own
        st = Stub(w).drive(straight(-1.0, 1.0), lambda p: p)
        case(st, "2 干净行驶（雷达就是这张图）", expect_stop=False)

        # 3 -- driving into ground nobody measured: `zero` collapses and *nothing* fits, which is
        # exactly what a carry looks like to a score that only knows walls. `outside` is the only
        # thing that tells them apart, and exploration is worthless if it aborts here.
        far = nav.World(extent=24.0, res=0.05)
        far.integrate((-1.0, 0.0, 0.0), sweep(-1.0, 0.0, 0.0), every=3, samples=12)
        st = Stub(far).drive(straight(-1.0, 2.0), lambda p: p)
        case(st, "3 开进从未测过的地面（不是搬动）", expect_stop=False)

        # 4 -- a human has the stick: warn, never take control away from somebody who can see
        st = Stub(w, busy="").drive(straight(-1.0, 1.0), carry)
        case(st, "4 点动中被搬走：只提醒不代管", expect_stop=False, expect_warn_only=True)

        # 5 -- the veto is still a veto when the map can answer: the see-through shares stay low
        # and unimproved, so "the candidate explains nothing better" is a claim worth believing.
        path = [(p[0] + 0.8, -0.5) for p in straight(-1.0, 1.0)]
        st = Stub(phantom_room(path)).coherent_veto().drive(straight(-1.0, 1.0), carry)
        case(st, "5 假墙但图仍能自洽（否决门放行）", expect_stop=False, expect_veto=True)

        # 5b -- the identical run with the see-through answer taken away must stop, or case 5 only
        # proves that something else happened to save it.
        st = Stub(phantom_room(path)).no_veto().drive(straight(-1.0, 1.0), carry)
        case(st, "5b 同一条路，但没有否决门（必须停）", expect_stop=True)

        # 6 -- the live 2026-10-18 failure: a *real* carry on a map whose every reachable placement
        # contradicts the sweep. The veto fires (no contrast left) and that is exactly where it
        # stops being evidence -- the car must not be told "不算搬动" and keep rolling.
        st = Stub(phantom_room(path)).drive(straight(-1.0, 1.0), carry)
        case(st, "6 假墙糊满图 + 真搬动（判不了 → 停）", expect_stop=True,
             expect_judge_cannot=True)

        # 6b/6c -- the same carry across two ghosted maps: heavy ghosting (20°) lands in case 6,
        # light ghosting (8°) still shows the contrast and aborts the ordinary way.
        st = Stub(ghost_room(20.0)).drive(straight(-1.0, 1.0), carry)
        case(st, "6b 重影 20° 的图 + 真搬动（判不了 → 停）", expect_stop=True,
             expect_judge_cannot=True)
        st = Stub(ghost_room(8.0)).drive(straight(-1.0, 1.0), carry)
        case(st, "6c 重影 8° 的图 + 真搬动（照常中止）", expect_stop=True)

        # 7 -- the case task 32 asked for by name: 稀疏图 + 真搬动 = 必须停. On a map only two folds
        # old the peak is broad *because* so little is claimed (0.97 against a 0.94 runner-up), so
        # the gate that stops a relocation sliding onto a vague answer used to excuse a real carry
        # silently. _thin_probe.py: 吻合 0.03, and the see-through pair 0.32 -> 0.00 was the only
        # thing in the whole measurement that knew the car had moved.
        st = Stub(thin_room(2)).drive(straight(-1.0, 1.0), carry)
        case(st, "7 稀疏图两次折叠 + 真搬动（必须停）", expect_stop=True)

        # 7b -- the same young map with nothing moved: the relaxation must not turn into a false
        # alarm at every tour's first metre, so the honest drive over the same folds is the A/B.
        st = Stub(thin_room(2)).drive([(p[0], -1.0, 0.0) for p in straight(-1.5, 1.5)], lambda p: p)
        case(st, "7b 同一张稀疏图，正常沿已测地面开（不许停）", expect_stop=False,
             expect_hold=False)

        # 8 -- the veto's other hole, and it is the thin map's: with the claimed pose hardly
        # contradicted at all, "the candidate looks through these walls no less" is a statement
        # about nothing, and a gate that fires on it would excuse any carry on a young map. The
        # pair is pinned rather than built because the point is the *shares*, not the geometry.
        st = Stub(phantom_room(path)).vacuous_veto().drive(straight(-1.0, 1.0), carry)
        case(st, "8 图几乎没矛盾时，否决门不得赦免真搬动", expect_stop=True)

        # 9 -- the live 2026-10-18 numbers, replayed: the car was already 0.92 m into a drive whose
        # first three frames had been folded at the lying pose, so the map was half fiction and the
        # score's margin had decayed to 0.226, just under WANT_MARGIN 0.25 (the live reading was
        # 吻合 0.771 / 间距 0.219). `fits` never opens, so the veto work in task 32 is never even
        # reached -- the branch that has to catch this is the grid's own accusation
        # (0.464 -> 0.205 see-through at a placement 0.94 m off) and it has to be a way in by itself.
        w9, x9 = half_healed()
        st = Stub(w9).drive(straight(x9, x9 + 0.64, step=0.16), carry)
        case(st, "9 半糊的图：打分瞎了但网格看得见（必须停）", expect_stop=True)
        assert "打分已经看不出" in why_of(st), \
            "停是停了，但判词没说是哪个感觉发现的：" + why_of(st)

        # 9b -- the same map with the grid's answer taken away. Silence here is what makes case 9 a
        # test of the new route rather than of the old one still working by luck.
        st = Stub(half_healed()[0]).no_veto().drive(straight(x9, x9 + 0.64, step=0.16), carry)
        case(st, "9b 同一张半糊的图，没有网格指认（老代码就是这样走过去的）",
             expect_stop=False, expect_hold=False)

        # 10 -- the fold loop's *second* witness, and it is not the score's. On 2026-10-19 a book
        # at the drive wheel made odom report 40° of rotation the gyro never saw (the car did not
        # turn; the operator's eyes settled that). The relocation peak stayed high, because a ring
        # of walls rotated along with the car still matches itself, so `suspect` never set and
        # every one of those frames voted into the grid -- 914 new cells and 930 erased, the exact
        # ghost this file's own ghost_room() models. The bridge now stamps `yaw_lie_t`, and a
        # stamped frame is held out even when the score has no complaint at all.
        st = Stub(room()).drive(straight(-1.0, 1.0), lambda p: p)
        assert not any(st.holds), "对照组先要干净：没有哪条判据在扣票"
        st.car.yaw_lie_t = TIME.t
        assert all(car_web.Console._hold_vote(st) for _ in range(5)), \
            "里程计朝向被抓到撒谎时，折叠必须扣在图外"
        st.car.yaw_lie_t = TIME.t - car_web.MOTION_HOLD - 0.1
        assert not car_web.Console._hold_vote(st), "过期的一瞥不该把建图永久扣死"
        print("%-46s 抓到撒谎即扣票、过期即放行  OK" % "10 另一条判据：odom 朝向与陀螺对不上")

        # 10b -- the same hold opened from the other lie. A straight push against a book invents
        # *distance*, not heading, so the gyro agrees with odom and case 10's witness stays quiet
        # while the grid grows a wall ahead of a car that never moved. The ring behind the car is
        # what catches it, and the frame has to be held out on that evidence alone.
        st = Stub(room()).drive(straight(-1.0, 1.0), lambda p: p)
        assert not any(st.holds), "对照组同样要干净：位移判据不能凭空扣票"
        st.car.slip_t = TIME.t
        assert all(car_web.Console._hold_vote(st) for _ in range(5)), \
            "雷达不认里程计的位移时，折叠必须扣在图外"
        st.car.slip_t = TIME.t - car_web.MOTION_HOLD - 0.1
        assert not car_web.Console._hold_vote(st), "位移的旧账同样不该把建图永久扣死"
        print("%-46s 身后的雷达不认账即扣票、过期即放行  OK" % "10b 第三条判据：odom 位移被雷达驳回")
    finally:
        car_web.time = real_time
    print("\n全部通过：该停的停了，不该停的一次都没停，判不了的没有假装判得出来。")


if __name__ == "__main__":
    main()
