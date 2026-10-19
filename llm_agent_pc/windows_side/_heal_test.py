# -*- coding: utf-8 -*-
"""谎言是怎么被自己折进图里的：同一场搬动，放行折叠 vs 扣住投票，四种结局。

真车 2026-10-18 实测：一条 Move 跑到 0.12 m 时把位姿平移 +0.80 m（等价于被搬走，只是不用手）。
行进中检测器第一帧就判对了 —— 吻合 0.71、峰值 0.97 正好落在 (-0.80, 0.00)，也就是真位姿。
然后什么也没发生：`连续 1/2 次`，命令跑完，车带着谎走完全程。第二帧的吻合度已经爬到 0.95。
中间那 0.2 秒里，折叠循环把这帧按**假位姿**投进了图，图长出了一间容得下谎的屋子。

这里把同一个循环搬回桌面：同一条路、同一个 0.8 m 谎、同一套判据，两处变量：
`_fold` 里那句 `if self._hold_vote(): continue` 在不在，以及中止要连判几帧（`MOTION_BAD`）。

C 组是车上原来那一遍的复现：连判窗口一长，放行折叠就把证据自己吃掉了 —— 吻合度爬回 0.9 以上，
`bad` 一路退回 0，永远攒不齐。D 组是扣票之后：同一套判据，第 6 帧照样停。
A/B 是当前出厂配置（MOTION_BAD=2）：稀疏图那条新通道（网格与打分一致就扣电机）把第 3 帧就凑齐的
判定接上了执行链，两种都停 —— 停得住之后，扣不扣票的差别就只剩"图上有没有多出一间假屋子"。

hold 读的是 `motion["suspect"]`（sticky 的"这一帧的位姿解释不了这帧雷达"），不是 `bad`：
`bad` 要连判两次才到 2，而第一帧之后那次投票就已经把证据糊掉了 —— 用 `bad>0` 当门等于没有门。

Run: C:\\microros-agent\\pc_side\\_rosrun.bat _heal_test.py
"""
import math

import car_web
import nav
from _motion_test import TIME, Stub, room, straight
from _reloc_test import sweep

CARRY = (0.8, -0.5)


def drive(hold, need=car_web.MOTION_BAD, metres=1.3, step=0.16):
    """一帧一帧地走：先问检测器，再按 `_fold` 的规则决定这帧能不能投票。

    step 必须大于 MOTION_ROLL(0.15 m)：检测器自己按 0.15 m 采一帧，走 0.1 m 的步长只会让它隔帧点头。
    `need` 临时改的是 car_web.MOTION_BAD —— 检测器读的是模块全局，所以这就是它自己要读的那个数。
    """
    w = room()
    st = Stub(w)
    trace = []
    real_bad = car_web.MOTION_BAD
    car_web.MOTION_BAD = need
    try:
        for p in straight(-1.0, -1.0 + metres, step=step):
            real = (p[0] + CARRY[0], p[1] + CARRY[1])
            claim = (p[0], p[1], 0.0)
            pairs = sweep(real[0], real[1], 0.0, n=667)
            car_web.Console._motion_watch(st, claim, pairs)
            # 再问一遍检测器内部的那些量（投票**之前**问，这才是它这一帧看见的图），
            # 好看清是哪一道门把 bad 退回 0 的
            r = nav.scan_offset(w, claim, pairs, reach=car_web.MOTION_REACH,
                                step=car_web.MOTION_STEP, every=car_web.MOTION_EVERY)
            t = st._through_pair(claim, pairs, r) if r else (None, None)
            held = hold and car_web.Console._hold_vote(st)
            if not held:
                w.integrate(claim, pairs)
            with st.lock:
                trace.append((round(math.hypot(p[0] + 1.0, p[1]), 2), st.motion["bad"],
                              r and round(r["zero"], 3), r and round(r["best"], 3),
                              r and (r["dx"], r["dy"]), r and round(r["rival"], 3), t,
                              bool(st.motion["suspect"]), held))
            TIME.t += 0.5
            if st.stops:
                break
    finally:
        car_web.MOTION_BAD = real_bad
    return st, trace


VETO_WORDS = {"veto": "赦", "junk": "判不了", "ok": "不赦", "none": "没答案"}


def show(tag, st, trace):
    print("%s  停车 %d 次  判定 %s" % (tag, st.stops, (st.motion["why"] or "")[:120]))
    print("     走了 m  bad 可疑扣票  吻合   峰值   次优  峰值偏移        穿墙(声称→首选)  否决")
    for row in trace:
        t = row[6]
        print("     %5.2f   %d    %s %s   %-6s %-6s %-6s (%+.2f,%+.2f)  %-18s %s"
              % (row[0], row[1], "是" if row[7] else "否", "扣" if row[8] else "投",
                 row[2], row[3], row[5], row[4][0] or 0, row[4][1] or 0,
                 "-" if t[0] is None else "%.2f->%.2f" % t,
                 "-" if t[0] is None else VETO_WORDS[car_web.Console._veto_check(None, t[0], t[1])]))


def best_zero(trace):
    v = [r[2] for r in trace if r[2] is not None]
    return max(v) if v else 0.0


def main():
    real_time = car_web.time
    car_web.time = TIME
    runs = {}
    try:
        for name, need, hold in [("A 出厂连判 2 帧，放行折叠", car_web.MOTION_BAD, False),
                                 ("B 出厂连判 2 帧，可疑帧扣票", car_web.MOTION_BAD, True),
                                 ("C 连判 6 帧，放行折叠（车上那一遍）", 6, False),
                                 ("D 连判 6 帧，可疑帧扣票", 6, True)]:
            st, tr = drive(hold, need=need)
            runs[name] = (st, tr)
            show(name, st, tr)
    finally:
        car_web.time = real_time

    st_a, tr_a = runs["A 出厂连判 2 帧，放行折叠"]
    st_b, tr_b = runs["B 出厂连判 2 帧，可疑帧扣票"]
    st_c, tr_c = runs["C 连判 6 帧，放行折叠（车上那一遍）"]
    st_d, tr_d = runs["D 连判 6 帧，可疑帧扣票"]

    assert st_c.stops == 0, "C 组没复现出车上那个失败（它停了），用例得重做"
    assert best_zero(tr_c) > 0.90, "C 组没自愈：吻合度没爬回 0.9 以上，谎没被折进图"
    assert not any(r[8] for r in tr_c), "C 组模拟的是车上原来那一遍，不该扣票"
    assert st_d.stops, "D 组：扣住可疑帧的票之后仍然没停车，第 4 步还是没接到执行链"

    assert st_a.stops and st_b.stops, "出厂配置下两种都该停：稀疏图那条新通道接上了判定"
    assert any(r[7] and r[8] for r in tr_b), "可疑帧没扣住投票：suspect 和 _hold_vote 没接上"
    assert int((runs["B 出厂连判 2 帧，可疑帧扣票"][0].world.lo >= nav.MATCH_WALL).sum()) \
        <= int((runs["A 出厂连判 2 帧，放行折叠"][0].world.lo >= nav.MATCH_WALL).sum()), \
        "扣票的那一遍图反而更多 —— 假屋子还是被折进去了"

    print("\n结论：连判窗口一长，放行折叠让吻合度从 %.2f 自己爬回 %.2f，车带着 %.2f m 的谎跑完 %d 帧"
          "也没停；把可疑帧扣在图外，同一套判据第 %d 帧就带着自己的判词停车。\n"
          "出厂配置（连判 %d 帧）下两种都停在第 %d 帧，差别只剩扣票那一遍的墙格少 %d 个 —— 谎没画进图。"
          % (tr_c[0][2], best_zero(tr_c), CARRY[0], len(tr_c), len(tr_d), car_web.MOTION_BAD,
             len(tr_a),
             int((runs["A 出厂连判 2 帧，放行折叠"][0].world.lo >= nav.MATCH_WALL).sum())
             - int((runs["B 出厂连判 2 帧，可疑帧扣票"][0].world.lo >= nav.MATCH_WALL).sum())))


main()
