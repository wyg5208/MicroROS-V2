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
       "grace_min": 120, "require_map": true, "min_agree": 0.80, "max_odom_age": 2.0,
       "max_distance": 0.8, "max_angle": 90},
      {"name": "朝目标续跑", "at": "21:00", "every_min": 20, "use_goal": true,
       "enabled": false, "max_minutes": 6, "max_distance": 0.8, "max_angle": 45}]}

急停有两个，都不用碰这个脚本：`cron.pause` 文件一存在就**一班都不发**（连手动点火也挡着，
恢复就是删掉它）；页面上那个"急停"仍能在动作中途把车掐停。另外每条任务有 `max_minutes`，
到点由调度侧发 `POST :8020/stop` —— 上限是分钟数，不是模型自报的"我做完了"。

最后那对 `max_distance` / `max_angle` 是**这一班的单次动作上限**：不写就沿用聊天页启动时的
全局上限（`--max-distance 1.5 --max-angle 90`），写了只能往更小压，且发出去后要核对 :8020
的回显 —— 回显对不上（多半是聊天页还在跑不认识这两个字段的旧版）就当场急停，不拿"我以为压住了"当证据。

`allow_tools` 是**这一班的工具白名单**（工具名列成数组，`["MapInfo", "GetStatus"]`）：不在名单里的
工具调用会在服务端被拦下，**根本不发给车**。上限管的是"这一步别走太远"，白名单管的才是"这一班根本
不该动车"，两者互补 —— 只读汇报那种任务要的是后者。`Stop` 不受名单限制（任何时候都能停），
人那一侧的急停更不经过这里。同样要核对回显，回显里没有这个字段就当场急停。

`use_goal: true` 把这班交给**长程目标**：那句话不由任务表写，发的是服务端 `chat.goal_nudge` 那一句，
和页面上「按目标再跑一轮」按出去的**一字不差** —— 自动班和人点的那班因此是同一句话、同一份账。
它只在挂着 active 目标时才发：没目标、目标已经 done/expired、轮数到头、聊天页是不认识 goal_nudge 的
旧版，四种一律跳过并把人话理由写进日志。再配 `every_min: 20` 就成循环：当天从 `at` 起每这么多分钟
发一班，窗口仍是 `grace_min`（到 23:59 收口，不跨午夜）。循环有两道刹车，都收进 `_cron_fired.json`
的 `halted`：上一班撞到 `max_minutes` 被这边掐停了、或者闸没被回显接住 —— 之后当天不再发下一班。
无人值守的循环最怕的不是"跑歪一步"，是"歪了还自己接下一班"。
"""
import argparse
import datetime as dt
import json
import os
import re
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
            "days": None, "enabled": False, "max_distance": None, "max_angle": None,
            "allow_tools": None, "use_goal": False, "every_min": None}
TEMPLATE = {"tasks": [{"name": "晚间断头巡游", "at": "22:30", "days": [0, 1, 2, 3, 4],
                       "prompt": "先看 MapInfo，把剩下的未测格用 Tour 走掉；每次 Move 不要超过 1.0 米",
                       "max_distance": 1.0, "enabled": False},
                      {"name": "朝目标续跑", "at": "21:00", "every_min": 20, "use_goal": True,
                       "max_minutes": 6, "max_distance": 0.8, "max_angle": 45, "enabled": False}]}

PATHS = {"dir": HERE}                      # --self-test 会把它挪进临时目录，别污染真状态

# "闸没被回显接住"这句话里的记号。循环刹车靠它分辨两种失败：话根本没进场（HTTP 失败，下一班照发）
# 和话已经进场了、正拿着全局上限跑（那边已经发过急停）。判据会核对真消息里带着这个记号。
STOPPED_MARK = "已发急停"


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
        if not t.get("name"):
            return out, "第 %d 条缺 name" % (i + 1)
        if not t.get("prompt") and not t.get("use_goal"):
            return out, "第 %d 条（%s）缺 prompt：要么自己写一句话，要么写 use_goal 让它发服务端那句目标催促" % (i + 1, t["name"])
        if t["name"] in names:
            return out, "任务重名：%s（去重按名字记，重名会互相顶掉）" % t["name"]
        names.add(t["name"])
        if "use_goal" in (item or {}) and not isinstance(t["use_goal"], bool):
            # 和 allow_tools 同一个道理：写错了不许当"没写"。当没写 = 那班发的是任务表里那句
            # 自己写的话（或者干脆没有话），而人以为它发的是目标那句。
            return out, "任务 %s 的 use_goal 不是 true/false：%r" % (t["name"], item.get("use_goal"))
        if t["use_goal"] and t.get("prompt"):
            return out, ("任务 %s 同时写了 prompt 和 use_goal：自动班必须发服务端 chat.goal_nudge 那一句，"
                         "自己写的那句请删掉（要发自己那句就把 use_goal 改成 false）" % t["name"])
        if t["every_min"] is not None:
            try:
                t["every_min"] = float(t["every_min"])
            except (TypeError, ValueError):
                return out, "任务 %s 的 every_min 不是分钟数：%r（不要循环就整条别写）" % (t["name"], item.get("every_min"))
            if t["every_min"] < 1:
                return out, "任务 %s 的 every_min=%g 太小（至少 1 分钟：跑完之前那一班自己会跳过，别拿更密的轮询当推进）" % (
                    t["name"], t["every_min"])
            if not t["use_goal"]:
                # 循环只给目标班开：目标那边有 turns_left 计数、有暗号、有 200 字/30 轮上限，
                # 一轮跑歪了会自己把额度烧完。自己写的一句话没有这套账，每 20 分钟重发一遍
                # 就是一个没人看着的循环。
                return out, ("任务 %s 写了 every_min 但没写 use_goal：自己写的那句话每隔几分钟重发一遍，"
                             "这边没有能兜住它的额度（要续跑目标就写 use_goal）" % t["name"])
        for key in ("max_distance", "max_angle"):
            if t[key] is None:
                continue
            try:
                t[key] = max(0.0, float(t[key]))
            except (TypeError, ValueError):
                # 写了就必须是数：把它当没写，发出去的那句话就拿全局上限跑，
                # 而日志里还写着"这班压到 0.8 米"—— 那比不压更糟。
                return out, "任务 %s 的 %s 不是数：%r（不写就留空，别写错的字）" % (t["name"], key, item.get(key))
        if t["allow_tools"] is not None:
            v = t["allow_tools"]
            # 和那对数同一个道理：写错了不能当"没写"。当没写=这一班拿全套 15 个工具跑，
            # 而日志里还写着"只许 MapInfo"，那正是本任务最怕的那件事。
            if not isinstance(v, list) or any(not isinstance(x, str) or not x.strip() for x in v):
                return out, "任务 %s 的 allow_tools 不是工具名的列表：%r（不限制就整条别写）" % (t["name"], v)
            keep = []
            for x in v:
                if x.strip() not in keep:
                    keep.append(x.strip())
            t["allow_tools"] = keep
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


def loop_step(task):
    """循环班隔多少分钟发一班；不是循环（含只发一班的 use_goal 班）返回 None。"""
    if task.get("use_goal") and task.get("every_min"):
        return float(task["every_min"])
    return None


def slot_index(task, fire_at, now):
    """现在落在这天第几个班点上（第 0 班 = `at` 那一刻）。

    不是循环班就一律 0 —— 去重记法和以前"一天一班"一模一样，老任务表一个字不改行为不变。
    """
    step = loop_step(task)
    if step is None:
        return 0
    return max(0, int((now - fire_at).total_seconds() // (60.0 * step)))


def due(task, now, rec):
    """这一班该不该发。返回 (发, 要记下的状态, 给人看的说明或 None)。

    同一班只发一次：计划程序每分钟调一遍 = 一天 1440 次，没这层去重就是每天发 1440 遍。
    循环班这句改成**同一个班点只发一次**（第几班见 slot_index）—— 否则每 20 分钟那一班会在
    22:30~22:49 之间发 20 遍。
    睡过头在宽限内补跑（合上盖子最容易撞上），过了宽限记成"错过"并按**那一班的日期**收口 ——
    否则一条"错过"会在日志里从半夜刷到午夜。
    halted 是循环的刹车（上一班被急停掐停、或闸没接住），它把这一天剩下的班全部收掉。
    """
    if not task["enabled"]:
        return False, None, None
    fire_at, last = slot(task, now)
    if fire_at is None:
        return False, None, None
    day = fire_at.strftime("%Y-%m-%d")
    idx = slot_index(task, fire_at, now)
    if rec.get("date") == day and rec.get("state") in ("halted", "missed"):
        return False, None, None
    if rec.get("date") == day and rec.get("state") == "fired" and int(rec.get("slot") or 0) >= idx:
        return False, None, None
    if task["days"] is not None and fire_at.weekday() not in task["days"]:
        return False, None, None                    # 那一天的班不归它管，别在日志里刷屏
    if now > last:
        if rec.get("date") == day and rec.get("state") == "fired":
            return False, None, None                # 循环窗口自己走完了，那班发过，别喊"错过"
        return False, {"date": day, "state": "missed", "slot": idx}, "%s %s 那班错过（宽限 %g 分钟，不跨午夜）" % (
            day, task["at"], task["grace_min"])
    return True, {"date": day, "state": "fired", "slot": idx}, None


def caps_note(task):
    """日志里那句"这班把单次动作上限压到多少"；两个都没写就是空串（沿用服务端全局上限）。"""
    p = []
    if task["max_distance"] is not None:
        p.append("%.2f 米" % task["max_distance"])
    if task["max_angle"] is not None:
        p.append("%.0f 度" % task["max_angle"])
    return "、".join(p)


def caps_applied(echo, max_distance, max_angle):
    """服务器回显的那对数**不高于**请求值，才算这个上限真被接住了。

    不采信"我发出去了"，只采信对面报回来的生效值 —— XML 那回就是脚本自认为自己改好了、
    schtasks 一个字没应用（离线判据 7 条全过）。字段整个没有 = 聊天页跑的是旧版 car_chat.py，
    那句话正拿全局上限跑着，一样算没生效。没请求上限时回显无所谓。
    """
    if not isinstance(echo, dict):
        return False
    for key, want in (("max_distance", max_distance), ("max_angle", max_angle)):
        if want is None:
            continue
        got = echo.get(key)
        if not isinstance(got, (int, float)) or isinstance(got, bool) or got > want + 1e-9:
            return False
    return True


def gate_note(task):
    """日志里那句"这班带了哪些闸"：上限 + 白名单，都没写就是空串。"""
    bits = []
    note = caps_note(task)
    if note:
        bits.append("单次上限 " + note)
    allow = task.get("allow_tools")
    if allow:
        bits.append("只许 " + "、".join(allow))
    elif allow is not None:
        bits.append("禁止调用任何工具")
    return "；".join(bits)


def tools_applied(echo, allow_tools):
    """回显的白名单**是请求值的子集**才算这层闸真接住了（服务端只会更严，不会更松）。

    和 `caps_applied` 同一条理由：话已经发出去了，字段被旧版聊天页安静地忽略，
    那一刻"只读班"正拿着动作工具在跑。没请求白名单时回显无所谓。
    """
    if allow_tools is None:
        return True
    if not isinstance(echo, dict):
        return False
    got = echo.get("allow_tools")
    if not isinstance(got, list):
        return False
    return set(str(x) for x in got) <= set(allow_tools)


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


def goal_note(chat):
    """这一班该发哪句话、或者为什么不发。返回 (话或 None, 人话理由或 None)。

    话不是这里写的：取服务端 `chat.goal_nudge`，和页面上「按目标再跑一轮」按出去的那句**一字不差**。
    四种跳过各说各的原因 —— 无人值守最容易出的事，是"没人在场，它拿一句没有目标可推进的话在跑"。
    """
    nudge = chat.get("goal_nudge")
    if not isinstance(nudge, str) or not nudge.strip():
        return None, ("聊天页 :8020 没下发那句目标催促：它多半还在跑不认识 goal_nudge 的旧版 car_chat.py，重启它")
    g = chat.get("goal")
    if not g:
        return None, "现在没挂着长程目标：在页面上先立一个，或者把这条班的 use_goal 关掉"
    if g.get("status") != "active":
        return None, "目标已经是「%s」状态，不再自动续跑（要接着做就重新立一个）" % (g.get("status") or "?")
    left = g.get("turns_left")
    if not isinstance(left, int) or isinstance(left, bool) or left < 1:
        return None, "目标的轮数到头了（还剩 %s 轮）：自动班不替它续额度" % left
    return nudge, None


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
        # 话由谁定：普通班发任务表里那句，目标班发服务端那句催促（这里不抄，只取回）。
        text = task.get("prompt")
        if task.get("use_goal"):
            text, skip = goal_note(chat)
            if skip:
                if rec.get("reasons") != [skip]:        # 同一个理由一天只骂一次
                    log("%s 跳过：%s" % (name, skip), now)
                fired[name] = {"date": day, "state": "blocked", "reasons": [skip]}
                did.append((name, "blocked", [skip]))
                continue
        if dry:
            note = gate_note(task)
            log("%s [dry-run] 体检全过，本该发这句%s：%s" % (
                name, "（" + note + "）" if note else "", text), now)
            did.append((name, "dry", []))
            continue
        turns0 = (chat or {}).get("turns", 0)
        note = gate_note(task)
        log("%s 触发（体检全过%s）：%s" % (
            name, "，" + note if note else "", text), now)
        err = api.post_chat(text, task["max_distance"], task["max_angle"],
                            task["allow_tools"])
        if err:
            log("%s 发不出去：%s" % (name, err), now)
            # 发不出去有两种：这班本来就发不动（blocked，下一分钟/下一班还可以再试），
            # 和**戴着闸的话被掐停了**（闸没接住 = 那句话此刻正拿全局上限、全套 15 个工具跑着）。
            # 后者对循环班必须当天收口：不然后一班每 20 分钟把同一句没闸的话再放出去一次。
            # 分辨这两种只看消息里那个 STOPPED_MARK 记号（发不出去=话没进场，没记号；闸没接住=话已经在
            # 里面跑了，post_chat 补过急停、必带记号）。判据会核对真消息确实带着它。
            halt = loop_step(task) is not None and STOPPED_MARK in err
            fired[name] = {"date": day, "state": "halted" if halt else "blocked", "reasons": [err]}
            if halt:
                log("%s 这个循环今天到此为止：闸没被回显接住，不要下一班" % name, now)
            did.append((name, "post_failed", [err]))
            continue
        upd["reasons"] = []
        fired[name] = upd
        state, info = wait_done(api, turns0, 60.0 * task["max_minutes"])
        said = api.last_text()
        log("%s %s（%.0f s，轮次到 %s）%s" % (
            name, state, info.get("used_s", 0), info.get("turns"),
            "：「%s」" % said[:120] if said else ""), now)
        write_last({"name": name, "at": now.strftime("%Y-%m-%d %H:%M:%S"), "state": state,
                    "gates": note, "info": info, "said": said, "prompt": text})
        # 循环的第一道刹车：上一班是撞到 max_minutes 被这边掐停的。
        # 那一班为什么停还没人知道（模型卡住？车卡住？网络断？），这种情况下"过 20 分钟再发一句"
        # 是最不该自动做的事 —— 记 halted，当天剩下的班全部收掉。
        if state == "超时已急停" and loop_step(task) is not None:
            upd["state"] = "halted"
            log("%s 这个循环今天到此为止：上一班是 %g 分钟上限掐停的，没人看着就别让它自己接下一班" % (
                name, task["max_minutes"]), now)
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
    step = "" if loop_step(task) is None else "每 %g 分钟一班、" % loop_step(task)
    if task["days"] is not None and now.weekday() not in task["days"]:
        return "今天（周%s）不在它的星期里；它写的时间是 %s" % ("一二三四五六日"[now.weekday()], hh)
    fire_at, last = slot(task, now)
    if fire_at is None or fire_at.date() != now.date():
        return "今天 %s 还没到（%s过后 %g 分钟内算这一班；宽限不跨午夜）" % (hh, step, task["grace_min"])
    if now <= last:
        return "%s 已过点，宽限内还会试（%s最晚到 %s）" % (hh, step, last.strftime("%H:%M"))
    return "%s 那班已过期（%s宽限 %g 分钟、不跨午夜），下一班明天" % (hh, step, task["grace_min"])


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
        """返回 (错误串或 None, 响应 JSON)。响应体以前是丢掉的，现在要拿它核对回显。"""
        req = urllib.request.Request(url, data=json.dumps(obj).encode("utf-8"),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=15.0) as r:
                raw = r.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as exc:
            return "HTTP %d：%s" % (exc.code, exc.read().decode("utf-8", "replace")[:120]), None
        except (urllib.error.URLError, OSError) as exc:
            return "%s: %s" % (type(exc).__name__, exc), None
        try:
            return None, json.loads(raw) if raw.strip() else {}
        except ValueError:
            return None, {}

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

    def post_chat(self, text, max_distance=None, max_angle=None, allow_tools=None):
        # confirm=False：无人值守不该留一张没人点的确认卡 —— 60 s 后它自己超时变拒绝，
        # 那一轮就只会拿着一串拒绝结束。硬上限换由 max_minutes + 急停 + 服务端 clamp 三处兜。
        # 这两个可选数就是第三处的旋钮：任务自己把单次位移压小，但只在回显对上时才算数。
        body = {"text": text, "confirm": False}
        if max_distance is not None:
            body["max_distance"] = max_distance
        if max_angle is not None:
            body["max_angle"] = max_angle
        if allow_tools is not None:
            body["allow_tools"] = allow_tools
        err, echo = self._post(self.chat_url + "/chat", body)
        if err:
            return err
        bad = [n for n, ok in (("单次上限", caps_applied(echo, max_distance, max_angle)),
                               ("工具白名单", tools_applied(echo, allow_tools))) if not ok]
        if bad:
            # 话已经发出去了，正拿全局上限、全套 15 个工具跑着 —— 先掐停再解释，别等它跑完。
            self.post_stop()
            want = gate_note({"max_distance": max_distance, "max_angle": max_angle,
                              "allow_tools": allow_tools}) or "没要求"
            return ("%s没生效（要 %s，:8020 回显 %s）：" + STOPPED_MARK + "。"
                    "多半是聊天页还在跑不认识这些字段的旧版 car_chat.py，重启它") % (
                        "、".join(bad), want, json.dumps(echo, ensure_ascii=False)[:120])
        return None

    def post_stop(self):
        return self._post(self.chat_url + "/stop", {})[0]

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
        self.sent_caps = []              # 每次 post_chat 带的那对逐轮上限，供判据核对
        self.sent_tools = []             # 每次 post_chat 带的那份工具白名单

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

    def post_chat(self, text, max_distance=None, max_angle=None, allow_tools=None):
        self.sent.append(text)
        self.sent_caps.append((max_distance, max_angle, allow_tools))
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
    api.post_chat = lambda *a: "HTTP 409：上一条指令还在执行"
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
    ck("模板任务表能读，且两条都默认未启用", [err, len(ts), ts[0]["enabled"], ts[1]["enabled"]],
       [None, 2, False, False])
    ck("  模板里那条目标班：话由服务端给，循环 20 分钟一班",
       [ts[1]["use_goal"], ts[1].get("prompt"), ts[1]["every_min"]], [True, None, 20.0])
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

    # 11 逐轮动作上限：任务表里那个数得一路带到 :8020，而且**回显对上**才算生效。
    ck("任务表没写就是 None（沿用服务端的天花板，不在这里另立一个默认数）",
       [task()["max_distance"], task()["max_angle"]], [None, None])
    with open(p, "w", encoding="utf-8") as fh:
        json.dump({"tasks": [{"name": "甲", "at": "07:00", "prompt": "x",
                              "max_distance": "一点点"}]}, fh, ensure_ascii=False)
    ck("写错成字直接报错，不许当没写（当没写 = 那句话拿全局上限跑，日志却写着压小过）",
       "不是数" in (load_tasks(p)[1] or ""), True)
    ck("caps_note 只把写了的那半句说出来",
       [caps_note(task()), caps_note(task(max_distance=0.8)),
        caps_note(task(max_distance=0.8, max_angle=45))], ["", "0.80 米", "0.80 米、45 度"])
    for echo, d, a, want in (({"max_distance": 0.8, "max_angle": 45.0}, 0.8, 45, True),
                             ({"max_distance": 0.5}, 0.8, None, True),
                             ({}, 0.8, None, False),
                             ({"max_distance": 1.5}, 0.8, None, False),
                             ({"max_distance": None}, 0.8, None, False),
                             (None, 0.8, None, False),
                             ({}, None, None, True)):
        ck("回显 %r 对要求 %s/%s -> %s" % (echo, d, a, want), caps_applied(echo, d, a), want)
    api, fired = FakeApi(chat={"busy": False, "turns": 0}), {}
    run_once([task(max_distance=0.8, max_angle=45)], api, T, fired, force="T")
    ck("任务表里的数真的带上了这次 POST", api.sent_caps, [(0.8, 45, None)])
    txt = open(os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read()
    ck("  触发那行日志带上上限（不然事后只看日志，不知道这班是压着跑的）", "0.80 米、45 度" in txt, True)
    ck("  _cron_last.json 也留一笔 gates",
       json.load(open(os.path.join(PATHS["dir"], "_cron_last.json"), encoding="utf-8"))["gates"],
       "单次上限 0.80 米、45 度")
    api = FakeApi(chat={"busy": False, "turns": 0})
    api.post_chat = lambda *a: "单次上限没生效（要 0.80 米，:8020 回显 {}）：已发急停"
    ck("  上限没生效 = 这班算发失败，记 blocked、不进等跑完的循环",
       [run_once([task(max_distance=0.8)], api, T, {}, force="T")[0][1], api.stops],
       ["post_failed", 0])
    with open(os.path.join(HERE, "car_chat.py"), encoding="utf-8") as fh:
        chat_src = fh.read()
    ck("服务端回显的字段名和这边读的是同一对（两边各写各的，早晚会漂）",
       ['"max_distance": dist' in chat_src, '"max_angle": ang' in chat_src,
        '"allow_tools": tools' in chat_src], [True, True, True])

    # 12 "回显对不上就当场急停"只能在真 HTTP 上证：假 api 里这条分支根本不会被走到。
    import http.server
    import threading
    hits = {"stop": 0, "body": None}

    class MockChat(http.server.BaseHTTPRequestHandler):
        echo = {}

        def log_message(self, *a):
            pass

        def do_POST(self):
            raw = self.rfile.read(int(self.headers.get("Content-Length", 0)))
            if self.path == "/chat":
                hits["body"] = json.loads(raw.decode("utf-8"))
            if self.path == "/stop":
                hits["stop"] += 1
            out = dict(self.echo) if self.path == "/chat" else {"ok": True}
            data = json.dumps(out).encode()
            self.send_response(202 if self.path == "/chat" else 200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    srv = http.server.HTTPServer(("127.0.0.1", 0), MockChat)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = "http://127.0.0.1:%d" % srv.server_address[1]
    try:
        api = HttpApi(chat=url, console=url)
        MockChat.echo = {"ok": True, "max_distance": 0.8, "max_angle": 45.0}
        ck("真发一次：回显对得上就正常返回，一发急停都不发",
           [api.post_chat("去巡游", 0.8, 45), hits["stop"]], [None, 0])
        ck("  上限走字段、原话一个字不混进 text（页面看到的就是任务表里那句）",
           hits["body"], {"text": "去巡游", "confirm": False, "max_distance": 0.8, "max_angle": 45})
        MockChat.echo = {"ok": True}          # 旧版 car_chat.py：不认识这两个字段
        err = api.post_chat("去巡游", 0.8)
        ck("回显没这对字段 = 没接住：话虽然发出去了，立刻补一发急停",
           [hits["stop"], "没生效" in err, "重启它" in err, STOPPED_MARK in err], [1, True, True, True])
        MockChat.echo = {"max_distance": 1.5}
        ck("  回显比请求大（服务端压根没压）也一样判没生效",
           [api.post_chat("去巡游", 0.8) is not None, hits["stop"]], [True, 2])
        MockChat.echo = {}
        ck("没要求上限时不看回显（老任务表一个字都不必改，行为和以前一样）",
           [api.post_chat("去巡游"), hits["body"]],
           [None, {"text": "去巡游", "confirm": False}])
        # 白名单走同一条回显规矩：旧版聊天页同样是安静地忽略，而那班车什么都调得动。
        MockChat.echo = {"ok": True, "max_distance": 1.5, "max_angle": 90.0,
                         "allow_tools": ["MapInfo", "GetStatus"]}
        ck("白名单对上回显：正常返回，一发急停都不补",
           [api.post_chat("只读汇报", None, None, ["MapInfo", "GetStatus"]), hits["stop"]], [None, 2])
        ck("  白名单也是走字段，不进那句话", hits["body"],
           {"text": "只读汇报", "confirm": False, "allow_tools": ["MapInfo", "GetStatus"]})
        MockChat.echo = {"ok": True}          # 旧版：不认识 allow_tools
        err = api.post_chat("只读汇报", None, None, ["MapInfo"])
        ck("回显没有 allow_tools = 这班其实什么都调得动：立刻急停",
           [hits["stop"], "工具白名单没生效" in err], [3, True])
        MockChat.echo = {"ok": True, "allow_tools": ["MapInfo", "Tour"]}
        ck("  回显比请求宽（服务端还多放了一个 Tour）同样判没生效",
           [api.post_chat("只读汇报", None, None, ["MapInfo"]) is not None, hits["stop"]], [True, 4])
    finally:
        srv.shutdown()

    # 13 工具白名单这层闸本身：上限管的是"这一步别走太远"，白名单管的才是"这班根本不该动车"。
    ck("任务表没写 allow_tools 就是不限制（老表一个字不改，行为和以前一字不差）",
       task()["allow_tools"], None)
    for bad_v, tag in (("MapInfo", "写成字符串"), ({"a": 1}, "写成对象"), ([1, 2], "列表里不是名字")):
        with open(p, "w", encoding="utf-8") as fh:
            json.dump({"tasks": [{"name": "甲", "at": "07:00", "prompt": "x",
                                  "allow_tools": bad_v}]}, fh, ensure_ascii=False)
        ck("  %s：当场报错，不许当没写" % tag, "allow_tools" in (load_tasks(p)[1] or ""), True)
    with open(p, "w", encoding="utf-8") as fh:
        json.dump({"tasks": [{"name": "甲", "at": "07:00", "prompt": "x",
                              "allow_tools": ["MapInfo", " MapInfo ", "GetStatus"]}]}, fh, ensure_ascii=False)
    ck("  重复与空格在加载时就收干净（回显比对用的是同一份规范化结果）",
       load_tasks(p)[0][0]["allow_tools"], ["MapInfo", "GetStatus"])
    ck("gate_note 把两样闸拼成人话，没写的部分不多嘴",
       [gate_note(task()), gate_note(task(max_distance=0.8)),
        gate_note(task(allow_tools=["MapInfo", "GetStatus"])),
        gate_note(task(max_distance=0.8, allow_tools=[]))],
       ["", "单次上限 0.80 米", "只许 MapInfo、GetStatus", "单次上限 0.80 米；禁止调用任何工具"])
    for echo, want_list, want in (({"allow_tools": ["MapInfo"]}, ["MapInfo"], True),
                                  ({"allow_tools": []}, ["MapInfo"], True),
                                  ({"allow_tools": ["MapInfo", "Tour"]}, ["MapInfo"], False),
                                  ({"allow_tools": []}, [], True),
                                  ({}, ["MapInfo"], False),
                                  ({"allow_tools": "MapInfo"}, ["MapInfo"], False),
                                  (None, ["MapInfo"], False),
                                  ({}, None, True)):
        ck("白名单回显 %r 对要求 %s -> %s" % (echo, want_list, want),
           tools_applied(echo, want_list), want)
    api = FakeApi(chat={"busy": False, "turns": 0})
    run_once([task(allow_tools=["MapInfo"])], api, T, {}, force="T")
    ck("任务表里的名单真的带上了这次 POST", api.sent_caps, [(None, None, ["MapInfo"])])
    txt = open(os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read()
    ck("  日志与 _cron_last.json 都写得出这班戴着哪层闸",
       ["只许 MapInfo" in txt,
        json.load(open(os.path.join(PATHS["dir"], "_cron_last.json"), encoding="utf-8"))["gates"]],
       [True, "只许 MapInfo"])
    api = FakeApi(chat={"busy": False, "turns": 0})
    api.post_chat = lambda *a: "工具白名单没生效（要 只许 MapInfo，:8020 回显 {}）：已发急停"
    ck("  白名单没生效 = 这班算发失败，记 blocked、不进等跑完的循环",
       run_once([task(allow_tools=["MapInfo"])], api, T, {}, force="T")[0][1], "post_failed")

    # 14 自动续跑：目标班发的是**服务端那一句**，不是任务表里写的；跳过要说清是哪一种；循环要有刹车。
    #    S 是假字样 —— 判据只问"发出去的是不是这一句"，cron 不许知道真句子长什么样（它也不该有）。
    S = "【服务端下发的目标催促】"

    def gchat(**over):
        c = {"busy": False, "turns": 3, "goal_nudge": S,
             "goal": {"text": "把北墙那一片测完", "turns_left": 5, "status": "active", "why": ""}}
        c.update(over)
        return c

    def gtask(**over):
        t = task(prompt=None, use_goal=True)
        t.update(over)
        return t

    ck("目标班要发的就是服务端那一句", goal_note(gchat()), (S, None))
    ck("  没挂目标：理由直接说清该怎么办", goal_note(gchat(goal=None))[1],
       "现在没挂着长程目标：在页面上先立一个，或者把这条班的 use_goal 关掉")
    ck("  目标已宣称完成：不再续跑",
       "已经是「done」" in goal_note(gchat(goal={"status": "done", "turns_left": 0}))[1], True)
    ck("  轮数到头：不替它续额度",
       "轮数到头" in goal_note(gchat(goal={"status": "active", "turns_left": 0}))[1], True)
    ck("  旧版聊天页没下发那句：也算跳过", "旧版 car_chat.py" in goal_note({"busy": False, "turns": 3})[1], True)
    api, fired = FakeApi(chat=gchat(turns=0), text="北墙还剩 12 格"), {}
    ck("一班真的把那句发了出去", [run_once([gtask()], api, T, fired, force="T")[0][1], api.sent],
       ["跑完", [S]])
    ck("  _cron_last.json 留的是这一句（事后只看它就知道 cron 发了什么）",
       json.load(open(os.path.join(PATHS["dir"], "_cron_last.json"), encoding="utf-8"))["prompt"], S)
    api = FakeApi(chat=gchat(goal=None, turns=0))
    ck("  没目标：一个字都不发", [run_once([gtask()], api, T, {}, force="T")[0][1], api.sent], ["blocked", []])
    api = FakeApi(chat={"busy": False, "turns": 0})
    ck("  旧版聊天页：也不退回任务表那句（那种班根本没有自己那句），并说重启它",
       [run_once([gtask()], api, T, {})[0][2][0].startswith("聊天页"), api.sent], [True, []])
    api = FakeApi(chat=gchat(turns=0))
    run_once([gtask()], api, T, {}, dry=True)
    ck("  dry-run 也先查目标，且打印的就是那一句", S in open(
        os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read().splitlines()[-1], True)

    L = gtask(every_min=20)
    REC3 = {"date": "2026-10-20", "state": "fired", "slot": 3}
    ck("循环班：到点算第 0 班", due(L, T, {})[1], {"date": "2026-10-20", "state": "fired", "slot": 0})
    ck("  同一班里每分钟调一遍只发一次（22:30~22:49 是同一班）",
       due(L, T + dt.timedelta(minutes=19), {"date": "2026-10-20", "state": "fired", "slot": 0})[0], False)
    ck("  过 20 分钟才换第 1 班",
       due(L, T + dt.timedelta(minutes=20), {"date": "2026-10-20", "state": "fired", "slot": 0})[1]["slot"], 1)
    ck("  循环窗口仍是 grace_min 那个，23:59 收口", due(L, dt.datetime(2026, 10, 20, 23, 59), REC3)[0], True)
    ck("  跨午夜后不再发、也不喊错过（那班本来就发过了）",
       [due(L, dt.datetime(2026, 10, 21, 0, 5), REC3)[0], due(L, dt.datetime(2026, 10, 21, 0, 5), REC3)[1]],
       [False, None])
    ck("  没写 every_min 的目标班还是「一天一班」：slot 恒 0",
       [slot_index(gtask(), T, T + dt.timedelta(minutes=95)),
        due(gtask(), T + dt.timedelta(minutes=95), REC3)[0]], [0, False])
    ck("  every_min 只有配上 use_goal 才算循环班",
       [loop_step(task()), loop_step(task(every_min=5)), loop_step(gtask(every_min=5))], [None, None, 5.0])

    wd, globals()["wait_done"] = wait_done, (lambda a, t0, lim, **kw: ("超时已急停", {"turns": 0, "used_s": 1.0}))
    try:
        api, fired = FakeApi(chat=gchat(turns=0)), {}
        run_once([L], api, T, fired, force="T")
        ck("刹车一：上一班是被 max_minutes 掐停的 → 这个循环当天收口（fired 改 halted）",
           [fired["T"]["state"], api.sent], ["halted", [S]])
        ck("  halted 把当天剩下的班全部收掉", due(L, T + dt.timedelta(minutes=20), fired["T"])[0], False)
        ck("  日志最后一行写着为什么停（人只看日志就知道循环没人接了）",
           "到此为止" in open(os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read().splitlines()[-1], True)
        api, fired = FakeApi(chat=gchat(turns=0)), {}
        run_once([gtask()], api, T, fired, force="T")
        ck("  只发一班的目标班超时不记 halted（本来就没有下一班要刹）", fired["T"]["state"], "fired")
    finally:
        globals()["wait_done"] = wd

    api = FakeApi(chat=gchat(turns=0))
    api.post_chat = lambda *a: "单次上限没生效（要 0.80 米，:8020 回显 {}）：" + STOPPED_MARK
    fired = {}
    ck("刹车二：闸没被回显接住 = 那句正拿全局上限跑 → 循环当天收口",
       [run_once([L], api, T, fired)[0][1], fired["T"]["state"],
        due(L, T + dt.timedelta(minutes=20), fired["T"])[0]], ["post_failed", "halted", False])
    api = FakeApi(chat=gchat(turns=0))
    api.post_chat = lambda *a: "HTTP 500：聊天页正在重启"
    fired = {}
    ck("  话根本没进场（HTTP 失败）不算刹车：记 blocked，下一班照发",
       [run_once([L], api, T, fired)[0][1], fired["T"]["state"],
        due(L, T + dt.timedelta(minutes=20), fired["T"])[0]], ["post_failed", "blocked", True])
    api, fired = FakeApi(chat=gchat(goal=None, turns=0)), {}
    run_once([L], api, T, fired)
    n0 = len(open(os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read().splitlines())
    run_once([L], api, T + dt.timedelta(minutes=1), fired)
    run_once([L], api, T + dt.timedelta(minutes=20), fired)
    ck("  没目标这一条：下一分钟不重发、下一班重试，但同一个理由不刷第二遍",
       [api.sent, len(open(os.path.join(PATHS["dir"], "_cron.log"), encoding="utf-8").read().splitlines())],
       [[], n0])

    p = os.path.join(PATHS["dir"], "cron_tasks.json")
    def load(body):
        with open(p, "w", encoding="utf-8") as fh:
            json.dump({"tasks": [body]}, fh, ensure_ascii=False)
        return load_tasks(p)
    ck("use_goal 班可以没有 prompt（话由服务端给）",
       [load({"name": "甲", "at": "07:00", "use_goal": True, "every_min": 20})[1],
        load({"name": "甲", "at": "07:00", "use_goal": True, "every_min": 20})[0][0]["every_min"]],
       [None, 20.0])
    ck("老表没写这两个字段：use_goal=False、every_min=None（行为和以前一字不差）",
       [load({"name": "甲", "at": "07:00", "prompt": "x"})[0][0]["use_goal"],
        load({"name": "甲", "at": "07:00", "prompt": "x"})[0][0]["every_min"]], [False, None])
    for body, want in (({"name": "甲", "at": "07:00", "prompt": "x", "use_goal": True}, "同时写了 prompt"),
                       ({"name": "甲", "at": "07:00", "prompt": "x", "every_min": 20}, "写了 every_min 但没写 use_goal"),
                       ({"name": "甲", "at": "07:00", "use_goal": True, "every_min": "半小时"}, "不是分钟数"),
                       ({"name": "甲", "at": "07:00", "use_goal": True, "every_min": 0}, "太小"),
                       ({"name": "甲", "at": "07:00", "use_goal": "true"}, "不是 true/false"),
                       ({"name": "甲", "at": "07:00"}, "缺 prompt")):
        ck("  %s：当场报错，不许当没写" % want, want in (load(body)[1] or ""), True)

    with open(os.path.join(HERE, "car_cron.py"), encoding="utf-8") as fh:
        cron_src = fh.read()
    with open(os.path.join(HERE, "car_chat_ui.html"), encoding="utf-8") as fh:
        ui_src = fh.read()
    m = re.search(r'GOAL_NUDGE = \("([^"]+)"', chat_src)
    ck("能从 car_chat.py 抄出那句的头几个字（它换了写法判据就红，不许静默跳过）", m is not None, True)
    head = (m.group(1)[:8]) if m else "GOAL-NOT-FOUND"
    ck("那句催促全仓只有 car_chat.py 一份：页面和 cron 都不抄",
       [head in chat_src, head in ui_src, head in cron_src], [True, False, False])
    ck("  三方读的是同一个字段名",
       ['"goal_nudge": GOAL_NUDGE' in chat_src, "chat.goal_nudge" in ui_src, "goal_nudge" in cron_src],
       [True, True, True])
    ck("  页面那个按钮没拿到那句就不许点（发空话比不发更糟）", "!nudge" in ui_src, True)

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
            mark = {"fired": "今天已发", "missed": "今天已错过", "blocked": "今天被拦",
                    "halted": "今天已停（循环刹车）"}.get(
                rec.get("state"), "") if rec.get("date") == day else ""
            if mark == "今天已发" and loop_step(t) is not None:
                mark += "第 %s 班" % (rec.get("slot") or 0)
            sends = ("发服务端那句目标催促" if t["use_goal"] else "发：%s" % str(t.get("prompt"))[:40])
            if loop_step(t) is not None:
                sends += " · 循环刹车：超时或闸没接住就当天收口"
            print("%-14s %-6s %-42s %s\n               %s" % (t["name"], t["at"], next_note(t, now), mark, sends))
        return 0
    api, fired = HttpApi(), load_fired()
    while True:
        run_once(tasks, api, dt.datetime.now(), fired, dry=cfg.dry_run, force=cfg.task)
        if cfg.task or not cfg.daemon:
            return 0
        time.sleep(cfg.poll)


if __name__ == "__main__":
    sys.exit(main())
