# -*- coding: utf-8 -*-
"""car_cron.py —— 外部调度：到点把一句话发给 :8020 的会话，剩下交给它和大模型。

为什么不在 bridge 里再造一个 scheduler：所有 ROS 订阅必须待在 bridge 一个进程里（硬约束），
而"该做什么"本来就该由发消息那一方决定。发出去的那一轮和人说的一模一样地进**同一条会话**，
所以结果在 `:8020` 页面里看（模型每一步、工具自报、并排的实测位移、为什么中止），
不用为一件新事再造第二个日志、第二套确认、第二个急停。

跑法（都在 C:\\microros-agent）：
  python -X utf8 car_cron.py --list              每条任务今天还会不会发（闹钟设对没有）
  python -X utf8 car_cron.py --once              到点就发、不到点就退出 —— 任务计划程序每分钟调它
  python -X utf8 car_cron.py --daemon            自己按 --poll 秒循环，不想装计划任务时用它
  python -X utf8 car_cron.py --task 名字         手动点火（现在就想要它跑一次）
  python -X utf8 car_cron.py --task 名字 --dry-run   只跑体检、只打印要发的话，一个字都不发
  python -X utf8 car_cron.py --self-test         离线判据：假时钟 + 假服务，不碰任何真东西

任务表 `cron_tasks.json`（没有文件时 --list 把模板打出来）。`enabled` 默认 false：
新加的任务不显式打开就永远不跑 —— 这条是给"手滑写错时间"兜底的。

    {"tasks": [
      {"name": "晚间断头巡游", "at": "22:30", "days": [0, 1, 2, 3, 4],
       "prompt": "先看 MapInfo，把剩下的未测格用 Tour 走掉；每次 Move 不要超过 1.0 米，走三轮就停下汇报",
       "enabled": false, "min_volts": 11.5, "max_minutes": 10,
       "grace_min": 120, "require_map": true, "min_agree": 0.80, "max_odom_age": 2.0}]}

急停有两个，都不用碰这个脚本：`cron.pause` 文件一存在就**一班都不发**（连手动点火也挡着，
恢复就是删掉它）；页面上那个"急停"仍能在动作中途把车掐停。另外每条任务有 `max_minutes`，
到点由调度侧发 `POST :8020/stop` —— 上限是分钟数，不是模型自报的"我做完了"。
"""
import argparse
import datetime as dt
import json
import os
import sys
import time
import urllib.error
import urllib.request

CHAT = "http://127.0.0.1:8020"
CONSOLE = "http://127.0.0.1:8010"
HERE = os.path.dirname(os.path.abspath(__file__))

# 默认值一律取保守的一侧：宁可少跑一班，也不要在车不在线、地图是空的、或者它自己都在怀疑位置的时候跑。
DEFAULTS = {"min_volts": 11.5, "max_minutes": 10.0, "grace_min": 120,
            "require_map": True, "min_agree": 0.80, "max_odom_age": 2.0,
            "days": None, "enabled": False}
TEMPLATE = {"tasks": [{"name": "晚间断头巡游", "at": "22:30", "days": [0, 1, 2, 3, 4],
                       "prompt": "先看 MapInfo，把剩下的未测格用 Tour 走掉；每次 Move 不要超过 1.0 米",
                       "enabled": False}]}

PATHS = {"dir": HERE}                      # --self-test 会把它挪进临时目录，别污染真状态


def log(msg, now=None):
    line = "%s  %s" % ((now or dt.datetime.now()).strftime("%Y-%m-%d %H:%M:%S"), msg)
    try:
        with open(os.path.join(PATHS["dir"], "_cron.log"), "a", encoding="utf-8") as fh:
            fh.write(line + "\n")
    except OSError:
        pass
    print(line)


def load_tasks(path=None):
    """读任务表并补默认值。读坏了要当场说清是哪一条，别静默变成"今天没有任务"。"""
    path = path or os.path.join(PATHS["dir"], "cron_tasks.json")
    if not os.path.exists(path):
        return None, None
    try:
        with open(path, encoding="utf-8") as fh:
            raw = json.load(fh)
    except (ValueError, OSError) as exc:
        return None, "读不到 %s：%s" % (os.path.basename(path), exc)
    out, names = [], set()
    for i, item in enumerate(raw.get("tasks") or []):
        t = dict(DEFAULTS)
        t.update(item or {})
        t["grace_min"] = float(t["grace_min"])
        t["max_minutes"] = float(t["max_minutes"])
        if not t.get("name") or not t.get("prompt"):
            return out, "第 %d 条缺 name 或 prompt" % (i + 1)
        if t["name"] in names:
            return out, "任务重名：%s（去重按名字记，重名会互相顶掉）" % t["name"]
        names.add(t["name"])
        try:
            parse_at(t["at"])
        except ValueError as exc:
            return out, "任务 %s 的 at 不是 HH:MM：%s" % (t["name"], exc)
        out.append(t)
    return out, None


def parse_at(text):
    h, m = str(text).split(":")
    h, m = int(h), int(m)
    if not (0 <= h < 24 and 0 <= m < 60):
        raise ValueError(text)
    return h, m


def load_fired():
    try:
        with open(os.path.join(PATHS["dir"], "_cron_fired.json"), encoding="utf-8") as fh:
            return json.load(fh)
    except (ValueError, OSError):
        return {}


def save_fired(rec):
    try:
        with open(os.path.join(PATHS["dir"], "_cron_fired.json"), "w", encoding="utf-8") as fh:
            json.dump(rec, fh, ensure_ascii=False, indent=1)
    except OSError:
        pass


def slot(task, now):
    """最近一个**已经到点**的班：返回 (班点时刻, 这一班最晚还能发的时刻)。

    宽限不跨午夜：22:30 的任务配 120 分钟宽限，如果让宽限跨过午夜，等于"合上盖子到第二天
    00:29 醒来时，半夜三更让车自己动起来"。跨午夜那一段一律算错过。
    """
    h, m = parse_at(task["at"])
    for back in (0, 1):
        d = now.date() - dt.timedelta(days=back)
        fire_at = dt.datetime.combine(d, dt.time(h, m))
        if fire_at <= now:
            day_end = dt.datetime.combine(d, dt.time(23, 59, 59))
            return fire_at, min(fire_at + dt.timedelta(minutes=task["grace_min"]), day_end)
    return None, None


def due(task, now, rec):
    """这一班该不该发。返回 (发, 要记下的状态, 给人看的说明或 None)。

    同一班只发一次：计划程序每分钟调一遍 = 一天 1440 次，没这层去重就是每天发 1440 遍。
    睡过头在宽限内补跑（合上盖子最容易撞上），过了宽限记成"错过"并按**那一班的日期**收口 ——
    否则一条"错过"会在日志里从半夜刷到午夜。
    """
    if not task["enabled"]:
        return False, None, None
    fire_at, last = slot(task, now)
    if fire_at is None:
        return False, None, None
    day = fire_at.strftime("%Y-%m-%d")
    if rec.get("date") == day and rec.get("state") in ("fired", "missed"):
        return False, None, None
    if task["days"] is not None and fire_at.weekday() not in task["days"]:
        return False, None, None                    # 那一天的班不归它管，别在日志里刷屏
    if now > last:
        return False, {"date": day, "state": "missed"}, "%s %s 那班错过（宽限 %g 分钟，不跨午夜）" % (
            day, task["at"], task["grace_min"])
    return True, {"date": day, "state": "fired"}, None


def preflight(task, chat, car, occ):
    """体检。字段名全部照 :8020/state 与 :8010/api/state 实测的那一套，不写代码里没读过的键。"""
    if chat is None:
        return ["聊天页 :8020 没起（话发不出去，车什么都听不到）"]
    if chat.get("busy"):
        return ["会话正忙 —— 有人正在对车说话或动作还没完，这一班不插队"]
    if car is None:
        return ["车侧读数拿不到（bridge :8010 没起？）"]
    why = []
    if not car.get("online"):
        why.append("车不在线（里程计没在报）")
    if car.get("busy"):
        why.append("电机被「%s」占用" % car.get("busy"))
    volts = car.get("volts")
    if isinstance(volts, (int, float)) and volts < task["min_volts"]:
        why.append("电池 %.2f V 低于任务要求的 %.2f V" % (volts, task["min_volts"]))
    age = car.get("odom_age")
    if isinstance(age, (int, float)) and age > task["max_odom_age"]:
        why.append("里程计已经 %.1f s 没更新（阈值 %.1f s）" % (age, task["max_odom_age"]))
    if car.get("frozen"):
        why.append("重定位正被冻住（此刻它自己都在怀疑位置），不动")
    agree = car.get("agree")
    if isinstance(agree, (int, float)) and agree < task["min_agree"]:
        why.append("雷达↔地图只吻合 %.0f%%，低于任务要求的 %.0f%%" % (100 * agree, 100 * task["min_agree"]))
    if task["require_map"]:
        if occ is None:
            why.append("读不到地图占据格数，无法确认图不是空的")
        elif occ <= 0:
            why.append("地图是空的（清空过还没重画），先画再巡游")
    return why


def run_once(tasks, api, now, fired, dry=False, force=None):
    """跑一班，返回这班对每条任务做了什么（--self-test 就断言这个）。"""
    did = []
    day = now.strftime("%Y-%m-%d")
    if api.pause_exists():
        # 计划程序每分钟调一遍，所以这句一天只说一次 —— 否则日志被 pause 那几天刷满 1440 行。
        if (fired.get("__pause__") or {}).get("date") != day:
            log("cron.pause 存在：一班都不发（连手动点火也挡着，要恢复就删掉这个文件）", now)
            fired["__pause__"] = {"date": day, "state": "paused"}
            save_fired(fired)
        return did
    for task in tasks:
        name = task["name"]
        rec = fired.get(name) or {}
        if force is not None:
            if name != force:
                continue
            go, upd, note = True, {"date": now.strftime("%Y-%m-%d"), "state": "fired"}, None
        else:
            go, upd, note = due(task, now, rec)
        if note:
            log("%s %s" % (name, note), now)
        if not go:
            continue
        chat, car, occ = api.chat_state(), api.car_state(), api.occupied()
        why = preflight(task, chat, car, occ)
        if why:
            if rec.get("state") != "blocked" or sorted(rec.get("reasons") or []) != sorted(why):
                log("%s 跳过：%s" % (name, "；".join(why)), now)
            fired[name] = {"date": now.strftime("%Y-%m-%d"), "state": "blocked", "reasons": why}
            did.append((name, "blocked", why))
            continue
        if dry:
            log("%s [dry-run] 体检全过，本该发这句：%s" % (name, task["prompt"]), now)
            did.append((name, "dry", []))
            continue
        turns0 = (chat or {}).get("turns", 0)
        log("%s 触发（体检全过）：%s" % (name, task["prompt"]), now)
        err = api.post_chat(task["prompt"])
        if err:
            log("%s 发不出去：%s" % (name, err), now)
            fired[name] = {"date": now.strftime("%Y-%m-%d"), "state": "blocked", "reasons": [err]}
            did.append((name, "post_failed", [err]))
            continue
        upd["reasons"] = []
        fired[name] = upd
        state, info = wait_done(api, turns0, 60.0 * task["max_minutes"])
        text = api.last_text()
        log("%s %s（%.0f s，轮次到 %s）%s" % (
            name, state, info.get("used_s", 0), info.get("turns"),
            "：「%s」" % text[:120] if text else ""), now)
        write_last({"name": name, "at": now.strftime("%Y-%m-%d %H:%M:%S"), "state": state,
                    "info": info, "said": text, "prompt": task["prompt"]})
        did.append((name, state, info))
    save_fired(fired)
    return did


def wait_done(api, turns0, limit_s, sleep=time.sleep, clock=time.monotonic, poll=3.0):
    """等这轮跑完： busy 落下来**且轮次涨了**才算完；超过 max_minutes 由这边发急停。"""
    t0 = clock()
    while True:
        st = api.chat_state() or {}
        if not st.get("busy") and st.get("turns", turns0) > turns0:
            return "跑完", {"turns": st.get("turns"), "used_s": round(clock() - t0, 1)}
        if clock() - t0 > limit_s:
            api.post_stop()
            return "超时已急停", {"turns": st.get("turns"), "used_s": round(clock() - t0, 1),
                                 "上限": "%g 分钟" % (limit_s / 60.0)}
        sleep(poll)


def write_last(obj):
    try:
        with open(os.path.join(PATHS["dir"], "_cron_last.json"), "w", encoding="utf-8") as fh:
            json.dump(obj, fh, ensure_ascii=False, indent=1)
    except OSError:
        pass


def next_note(task, now):
    """给人核对闹钟用的一行话，和 due() 用同一个 slot() 算 —— 免得说的和做的不是一套。"""
    if not task["enabled"]:
        return "未启用（enabled=false 就永远不发）"
    h, m = parse_at(task["at"])
    hh = "%02d:%02d" % (h, m)
    if task["days"] is not None and now.weekday() not in task["days"]:
        return "今天（周%s）不在它的星期里；它写的时间是 %s" % ("一二三四五六日"[now.weekday()], hh)
    fire_at, last = slot(task, now)
    if fire_at is None or fire_at.date() != now.date():
        return "今天 %s 还没到（过后 %g 分钟内算这一班；宽限不跨午夜）" % (hh, task["grace_min"])
    if now <= last:
        return "%s 已过点，宽限内还会试（最晚到 %s）" % (hh, last.strftime("%H:%M"))
    return "%s 那班已过期（宽限 %g 分钟、不跨午夜），下一班明天" % (hh, task["grace_min"])


# --------------------------------------------------------------------- HTTP --
class HttpApi(object):
    def __init__(self, chat=CHAT, console=CONSOLE, timeout=5.0):
        self.chat_url, self.console_url, self.timeout = chat, console, timeout

    def _get(self, url):
        try:
            with urllib.request.urlopen(url, timeout=self.timeout) as r:
                return json.loads(r.read().decode("utf-8", "replace"))
        except (urllib.error.URLError, OSError, ValueError):
            return None

    def _post(self, url, obj):
        req = urllib.request.Request(url, data=json.dumps(obj).encode("utf-8"),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=15.0) as r:
                r.read()
            return None
        except urllib.error.HTTPError as exc:
            return "HTTP %d：%s" % (exc.code, exc.read().decode("utf-8", "replace")[:120])
        except (urllib.error.URLError, OSError) as exc:
            return "%s: %s" % (type(exc).__name__, exc)

    def chat_state(self):
        st = self._get(self.chat_url + "/state")
        return None if st is None else dict(st.get("chat") or {})

    def car_state(self):
        st = self._get(self.chat_url + "/state")
        return None if st is None else dict(st.get("car") or {})

    def occupied(self):
        st = self._get(self.console_url + "/api/state")
        return None if st is None else st.get("occ")

    def last_text(self):
        st = self._get(self.chat_url + "/history")
        for ev in reversed((st or {}).get("events") or []):
            if ev.get("type") == "assistant":
                return ev.get("text") or ""
        return ""

    def post_chat(self, text):
        # confirm=False：无人值守不该留一张没人点的确认卡 —— 60 s 后它自己超时变拒绝，
        # 那一轮就只会拿着一串拒绝结束。硬上限换由 max_minutes + 急停 + 服务端 clamp 三处兜。
        return self._post(self.chat_url + "/chat", {"text": text, "confirm": False})

    def post_stop(self):
        return self._post(self.chat_url + "/stop", {})

    def pause_exists(self):
        return os.path.exists(os.path.join(PATHS["dir"], "cron.pause"))


# --------------------------------------------------------- 自测（不碰真东西）--
class FakeApi(object):
    """假服务：把要发的话记下来，别的一概不真做 —— 自测因此能在不接车的情况下量完全部判据。"""

    def __init__(self, chat=None, car=None, occ=2000, pause=False, busy_turns=0, text=None):
        base = {"busy": False, "turns": 3}
        self.chat = dict(base if chat is None else chat)
        self.car = ({"online": True, "busy": "", "volts": 12.4, "mapping": True,
                     "frozen": False, "agree": 0.99, "odom_age": 0.05} if car is None else dict(car))
        self.occ, self.pause, self.text = occ, pause, text
        # busy_turns：发出去之后还要"忙"几次读数才算跑完。只在发过话之后才倒计时 ——
        # 体检那次读数不许把忙状态吃掉（不然测不出"有人在开车时别插队"）。
        self.busy_left = busy_turns
        self.left = None
        self.sent, self.stops, self.gets = [], 0, 0

    def chat_state(self):
        self.gets += 1
        if self.chat.get("busy") and self.left is not None:
            if self.left <= 0:
                self.chat["busy"] = False
                self.chat["turns"] = self.chat.get("turns", 0) + 1
            self.left -= 1
        return dict(self.chat)

    def car_state(self):
        return dict(self.car)

    def occupied(self):
        return self.occ

    def last_text(self):
        return self.text or ""

    def post_chat(self, text):
        self.sent.append(text)
        self.chat["busy"] = True
        self.left = self.busy_left
        return None

    def post_stop(self):
        self.stops += 1
        self.chat["busy"] = False
        return None

    def pause_exists(self):
        return self.pause


def self_test():
    import tempfile
    PATHS["dir"] = tempfile.mkdtemp()          # 日志/去重表都写进临时目录，真状态一个字不动
    fails = []

    def ck(label, got, want):
        ok = got == want
        print(("OK  " if ok else "FAIL ") + label)
        if not ok:
            print("      got=%r want=%r" % (got, want))
            fails.append(label)

    def task(**over):
        t = dict(DEFAULTS)
        t.update({"name": "T", "at": "22:30", "prompt": "去巡游", "enabled": True})
        t.update(over)
        return t

    T = dt.datetime(2026, 10, 20, 22, 30)      # 2026-10-20 是星期二（weekday()=1）
    good = {"busy": False, "turns": 3}
    YDAY = {"date": "2026-10-19", "state": "fired"}

    # 1 到期判定：全部假时钟
    ck("到点当分钟就发", due(task(), T, {})[0], True)
    ck("未到点不发", due(task(), T - dt.timedelta(minutes=1), YDAY)[0], False)
    ck("同一天已发过就不再发（每分钟调一次 = 一天 1440 次）",
       due(task(), T + dt.timedelta(minutes=5), {"date": "2026-10-20", "state": "fired"})[0], False)
    ck("睡过头在宽限内补跑（22:30 的班，23:00 才有电）", due(task(), T + dt.timedelta(minutes=30), {})[0], True)
    ck("  宽限一直算到 23:59 收口（22:30 + 120 分钟本来到 00:30）",
       due(task(), T + dt.timedelta(minutes=79), {})[0], True)
    go, upd, note = due(task(), T + dt.timedelta(minutes=121), {})     # 00:31，已跨午夜
    ck("宽限不跨午夜：00:31 醒来说「昨天那班错过」，而不是半夜把车发动",
       [go, upd["state"], upd["date"], "2026-10-20 22:30" in note], [False, "missed", "2026-10-20", True])
    ck("  错过之后按那一班的日期收口（第二天的班照旧）",
       [due(task(), T + dt.timedelta(hours=2), upd)[0],
        due(task(), T + dt.timedelta(days=1, hours=1), {})[0]], [False, True])
    ck("未启用的任务永远不发", due(task(enabled=False), T, {})[0], False)
    ck("days 不含那一班的那天不发", due(task(days=[0, 2, 4]), T, {})[0], False)
    ck("days 含那天才发", due(task(days=[1]), T, {})[0], True)

    # 2 体检：每条都要给出人话理由，而且不许发消息
    ck("电机被别人占用", preflight(task(), good, {"online": True, "busy": "点动", "volts": 12.4,
                                             "frozen": False, "agree": 0.99, "odom_age": 0.05}, 2000),
       ["电机被「点动」占用"])
    ck("车不在线 + 里程计过期 = 两条理由",
       len(preflight(task(), good, {"online": False, "busy": "", "volts": 12.4, "frozen": False,
                                    "agree": 0.99, "odom_age": 9.0}, 2000)), 2)
    ck("电压低于任务要求", len(preflight(task(), good, {"online": True, "busy": "", "volts": 10.9,
                                                   "frozen": False, "agree": 0.99, "odom_age": 0.05}, 2000)), 1)
    ck("重定位冻住 + 吻合 55%", len(preflight(task(), good, {"online": True, "busy": "", "volts": 12.4,
                                                         "frozen": True, "agree": 0.55, "odom_age": 0.05}, 2000)), 2)
    ck("地图是空的", len(preflight(task(), good, FakeApi().car_state(), 0)), 1)
    api = FakeApi()
    ck("  占据格数读不到也不发（宁少跑一班）", len(preflight(task(), good, api.car_state(), None)), 1)
    ck("会话正忙不插队", preflight(task(), {"busy": True, "turns": 1}, api.car_state(), 2000),
       ["会话正忙 —— 有人正在对车说话或动作还没完，这一班不插队"])
    ck("聊天页没起：只此一条理由", len(preflight(task(), None, api.car_state(), 2000)), 1)
    ck("任务可以自己收紧阈值（min_volts=12.6 时 12.4 V 不发）",
       len(preflight(task(min_volts=12.6), good, api.car_state(), 2000)), 1)

    # 3 一班真的跑起来
    api, fired = FakeApi(chat={"busy": False, "turns": 7}, text="未测还剩 482 格，我走了一圈"), {}
    did = run_once([task()], api, T, fired)
    ck("把话发给了会话", api.sent, ["去巡游"])
    ck("跑完记 fired + 状态", [fired["T"]["state"], did[0][1]], ["fired", "跑完"])
    ck("助手最后一句话进 _cron_last.json（人不用开着页面也能看到结果）",
       json.load(open(os.path.join(PATHS["dir"], "_cron_last.json"), encoding="utf-8"))["said"],
       "未测还剩 482 格，我走了一圈")
    lines = open(os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read().splitlines()
    ck("日志一行说清发了什么、跑了多久", ("触发" in lines[0]) and ("跑完" in lines[1]), True)

    # 4 体检不过：一个字都不发，且同一条理由不重复骂
    bad_car = {"online": False, "busy": "", "volts": 9.9, "frozen": True, "agree": 0.4, "odom_age": 9.9}
    api, fired = FakeApi(car=bad_car, chat={"busy": False, "turns": 0}), {}
    did = run_once([task()], api, T, fired)
    ck("体检不过：没发出去", [api.sent, did[0][1]], [[], "blocked"])
    ck("  理由逐条记下（写清是哪几条，不是一句「体检没过」）", sorted(fired["T"]["reasons"]),
       sorted(["车不在线（里程计没在报）", "电池 9.90 V 低于任务要求的 11.50 V",
               "里程计已经 9.9 s 没更新（阈值 2.0 s）", "重定位正被冻住（此刻它自己都在怀疑位置），不动",
               "雷达↔地图只吻合 40%，低于任务要求的 80%"]))
    before = len(open(os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read().splitlines())
    run_once([task()], api, T + dt.timedelta(minutes=1), fired)
    ck("  下一班同样的理由不再重复骂（每分钟一次会刷到爆）",
       len(open(os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read().splitlines()), before)
    ck("  但 blocked 不等于 fired：车回来当天仍允许补跑",
       due(task(), T + dt.timedelta(minutes=2), fired["T"])[0], True)

    # 5 上限是分钟数，不是模型自报的"我做完了"
    api = FakeApi(chat={"busy": False, "turns": 0}, busy_turns=10 ** 9)     # 发出去之后永远忙
    ticks = [0.0]
    state, info = wait_done(api, 0, 120.0, sleep=lambda s: ticks.__setitem__(0, ticks[0] + 60),
                            clock=lambda: ticks[0])
    ck("超过 max_minutes 就由这边发急停", [state, api.stops], ["超时已急停", 1])
    ck("  急停的理由写成分钟数，人一眼知道上限在哪", info["上限"], "2 分钟")

    # 6 暂停开关：连手动点火也挡着
    api = FakeApi(pause=True, chat={"busy": False, "turns": 0})
    ck("cron.pause 存在时一班都不发", [run_once([task()], api, T, {}), api.sent], [[], []])
    ck("  手动点火也照样挡（pause 说的就是「谁都别动车」）",
       [len(run_once([task()], api, T, {}, force="T")), api.sent][1], [])
    ck("  暂停时连 /state 都没去读（一次 GET 都不发）", api.gets, 0)
    n0 = len(open(os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read().splitlines())
    # 第二班是新进程（计划程序每分钟起一个新的），所以判据要从磁盘读回来才算数
    run_once([task()], api, T + dt.timedelta(minutes=1), load_fired())
    ck("  暂停那句一天只说一次（每分钟调一遍会刷满 1440 行）",
       len(open(os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read().splitlines()), n0)

    # 7 手动点火：不看时间与星期，但仍过体检
    api, fired = FakeApi(chat={"busy": False, "turns": 0}), {}
    did = run_once([task(at="03:00", days=[6], grace_min=1)], api, T, fired, force="T")
    ck("--task 手动点火：过点、宽限、星期全绕开", [len(api.sent), did[0][1]], [1, "跑完"])
    ck("  点完当天记 fired（免得夜里那班又自己跑一遍）", fired["T"]["state"], "fired")
    api = FakeApi(chat={"busy": True, "turns": 0})
    ck("  但手动点火也要过体检（有人正在开车就别插队）",
       [run_once([task()], api, T, {}, force="T")[0][1], api.sent], ["blocked", []])
    api = FakeApi(chat={"busy": False, "turns": 0})
    api.post_chat = lambda t: "HTTP 409：上一条指令还在执行"
    ck("发不出去要记 blocked 并说清原因", run_once([task()], api, T, {})[0][2],
       ["HTTP 409：上一条指令还在执行"])
    api = FakeApi(chat={"busy": False, "turns": 0})
    ck("enabled=false 也允许手动点火（它管的是排班，不是「这条任务存不存在」）",
       [len(run_once([task(enabled=False)], api, T, {}, force="T")), api.sent], [1, ["去巡游"]])

    # 8 任务表本身
    p = os.path.join(PATHS["dir"], "cron_tasks.json")
    with open(p, "w", encoding="utf-8") as fh:
        json.dump(TEMPLATE, fh, ensure_ascii=False)
    ts, err = load_tasks(p)
    ck("模板任务表能读，且默认未启用", [err, len(ts), ts[0]["enabled"]], [None, 1, False])
    ck("缺的字段补上保守默认", [ts[0]["min_volts"], ts[0]["grace_min"], ts[0]["require_map"]],
       [11.5, 120, True])
    for body, want in (('{"tasks":[{"name":"甲","at":"25:00","prompt":"x","enabled":true}]}', "任务 甲 的 at"),
                       ('{"tasks":[{"name":"甲","at":"07:00","prompt":"x"},{"name":"甲","at":"08:00","prompt":"y"}]}', "重名"),
                       ('{"tasks":[{"at":"07:00","prompt":"x"}]}', "第 1 条缺 name")):
        with open(p, "w", encoding="utf-8") as fh:
            fh.write(body)
        ck("坏任务表报出「%s」，不静默当没任务" % want, want in (load_tasks(p)[1] or ""), True)
    with open(p, "w", encoding="utf-8") as fh:
        fh.write("not json")
    ck("文件是垃圾时报错", load_tasks(p)[1] is not None, True)
    ck("文件不存在返回 (None, None)（--list 会把模板打给人看）",
       load_tasks(os.path.join(PATHS["dir"], "nope.json")), (None, None))

    # 9 --list 那行话得是真的，而且必须和 due() 用同一套算出来的数
    now = dt.datetime(2026, 10, 20, 9, 0)
    ck("未到点：说清今天几点和宽限", next_note(task(at="22:30", grace_min=120), now),
       "今天 22:30 还没到（过后 120 分钟内算这一班；宽限不跨午夜）")
    ck("未启用就说不启用", next_note(task(enabled=False), now), "未启用（enabled=false 就永远不发）")
    ck("星期不对要说今天是周几", next_note(task(days=[6]), now),
       "今天（周二）不在它的星期里；它写的时间是 22:30")
    ck("过了宽限要说今天没了", next_note(task(at="03:00", grace_min=120), now),
       "03:00 那班已过期（宽限 120 分钟、不跨午夜），下一班明天")
    ck("宽限内要说最晚到几点（23:59 收口，不是 00:30）",
       next_note(task(at="22:30", grace_min=120), dt.datetime(2026, 10, 20, 23, 0)),
       "22:30 已过点，宽限内还会试（最晚到 23:59）")
    ck("  这行话里的最晚时刻和 due() 的判定是同一个数",
       [due(task(at="22:30", grace_min=120), dt.datetime(2026, 10, 20, 23, 59, 30), {})[0],
        due(task(at="22:30", grace_min=120), dt.datetime(2026, 10, 21, 0, 0, 30), {})[0]], [True, False])

    # 10 安装脚本必须和这个脚本对同一件事：它注册的那条命令、它翻的那几个电源开关。
    #    （跟"prompt 里的工具名要对着 tools/list 核"是同一类检查 —— 两边各写各的，早晚会漂。）
    try:
        with open(os.path.join(HERE, "安装定时任务.bat"), encoding="utf-8") as fh:
            setup = fh.read()
        with open(os.path.join(HERE, "卸载定时任务.bat"), encoding="utf-8") as fh:
            teardown = fh.read()
    except OSError:
        setup = teardown = ""
    ck("安装脚本注册的就是 car_cron.py --once（不是 --daemon，也没漏 --once）",
       "car_cron.py --once" in setup, True)
    ck("它替笔记本翻了那两个电池开关、并允许补跑错过的那班（schtasks 命令行给不了这三个开关）",
       ["DisallowStartIfOnBatteries=$false" in setup, "StopIfGoingOnBatteries=$false" in setup,
        "StartWhenAvailable=$true" in setup], [True, True, True])
    ck("装和删用的是同一个任务名（删错了就等于没人卸载）",
       ["MicroROS-CarCron" in setup, "MicroROS-CarCron" in teardown], [True, True])
    ck("两个 .bat 都是纯 ASCII（cmd.exe 按字节偏移读文件，多字节会咬断一行当命令跑）",
       [max((ord(c) for c in setup), default=0) < 128,
        max((ord(c) for c in teardown), default=0) < 128], [True, True])

    print("\n%s" % ("SELF-TEST PASS" if not fails else "SELF-TEST FAIL：%s" % fails))
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser(
        description="到点把一句话发给 :8020 的会话（外部调度，不在 bridge 里造第二个 scheduler）")
    ap.add_argument("--once", action="store_true", help="跑一班就退出（任务计划程序用的就是它）")
    ap.add_argument("--daemon", action="store_true", help="自己循环")
    ap.add_argument("--poll", type=float, default=20.0, help="--daemon 的轮询秒数")
    ap.add_argument("--task", help="只跑这一条，不看时间（手动点火）")
    ap.add_argument("--dry-run", action="store_true", help="只体检、只打印，不发话")
    ap.add_argument("--list", action="store_true", help="列出任务与今天还会不会发")
    ap.add_argument("--self-test", action="store_true", help="离线判据，不碰任何服务")
    cfg = ap.parse_args()
    if cfg.self_test:
        return self_test()
    tasks, err = load_tasks()
    if err:
        print("任务表有问题：%s" % err)
        return 2
    if tasks is None:
        print("还没有 %s。模板长这样（enabled 先写 false，看清时间再打开）：\n%s"
              % (os.path.join(PATHS["dir"], "cron_tasks.json"),
                 json.dumps(TEMPLATE, ensure_ascii=False, indent=2)))
        return 0
    if cfg.list:
        now, fired = dt.datetime.now(), load_fired()
        day = now.strftime("%Y-%m-%d")
        for t in tasks:
            rec = fired.get(t["name"]) or {}
            mark = {"fired": "今天已发", "missed": "今天已错过", "blocked": "今天被拦"}.get(
                rec.get("state"), "") if rec.get("date") == day else ""
            print("%-14s %-6s %-42s %s" % (t["name"], t["at"], next_note(t, now), mark))
        return 0
    api, fired = HttpApi(), load_fired()
    while True:
        run_once(tasks, api, dt.datetime.now(), fired, dry=cfg.dry_run, force=cfg.task)
        if cfg.task or not cfg.daemon:
            return 0
        time.sleep(cfg.poll)


if __name__ == "__main__":
    sys.exit(main())
