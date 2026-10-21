# -*- coding: utf-8 -*-
"""`drives_gate` 的判据：会动车的脚本，不给 `drives` 就一个字都不许发出去。

全程零电机命令：子进程里先把 socket / subprocess 绑住，再 runpy 真跑那些脚本，
所以"它没动车"不是我看它退出码 2 猜的，而是**它想发也发不出去**。
反向对照（C 段）证明这把锁真的会响：不锁的话那条连接就该发出去。

跑法（不需要 ROS，也不需要车在线）：
    C:/Users/Leo/AppData/Local/Programs/Python/Python310/python.exe -X utf8 _drives_gate_test.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from drives_gate import DRIVES_TOKEN, require_drives  # noqa: E402

SCRIPTS = ["_abort_test.py", "_cruise_w.py", "_estop_move_test.py", "_honest_run.py",
           "_machine_stop_test.py", "_turn_cold.py", "_yaw_cause.py"]
PORTS = (8000, 8010, 8020)

n_pass = 0
n_fail = 0


def ck(name, got, want):
    global n_pass, n_fail
    if got == want:
        n_pass += 1
        print("OK   %s" % name)
    else:
        n_fail += 1
        print("FAIL %s\n       got  %r\n       want %r" % (name, got, want))


def close(name, got, want, tol=1e-9):
    global n_pass, n_fail
    ok = abs(got - want) <= tol
    if ok:
        n_pass += 1
        print("OK   %s" % name)
    else:
        n_fail += 1
        print("FAIL %s  got %r want %r" % (name, got, want))


BOOT = r'''
import json, runpy, socket, subprocess, sys
BLOCKED = []
PORTS = %(ports)r
_orig_connect = socket.socket.connect
def _guard(self, addr, *a, **k):
    port = addr[1] if isinstance(addr, tuple) and len(addr) >= 2 else None
    if port in PORTS:
        BLOCKED.append(["connect", str(addr), port])
        raise RuntimeError("drives-gate-test: 拦下发往端口 %%s 的连接" %% port)
    return _orig_connect(self, addr, *a, **k)
socket.socket.connect = _guard
_orig_cc = socket.create_connection
def _guard_cc(address, *a, **k):
    port = address[1] if isinstance(address, tuple) and len(address) >= 2 else None
    if port in PORTS:
        BLOCKED.append(["create_connection", str(address), port])
        raise RuntimeError("drives-gate-test: 拦住 create_connection")
    return _orig_cc(address, *a, **k)
socket.create_connection = _guard_cc
def _guard_popen(self, *a, **k):
    BLOCKED.append(["popen", str(a[0] if a else "")[:90], 0])
    raise RuntimeError("drives-gate-test: 拦住子进程")
subprocess.Popen.__init__ = _guard_popen

target = sys.argv[1]
sys.argv = sys.argv[1:]
code = 0
try:
    runpy.run_path(target, run_name="__main__")
except SystemExit as exc:
    code = 0 if exc.code is None else (exc.code if isinstance(exc.code, int) else 1)
except Exception as exc:
    code = 90
    print("RUNERR %%s: %%s" %% (type(exc).__name__, exc))
print("GUARD %%s" %% json.dumps({"code": code, "blocked": BLOCKED}))
''' % {"ports": list(PORTS)}


def run_guarded(script, args, script_dir=None, cwd=None):
    """在锁住网络/子进程的解释器里真跑那个脚本，返回 (退出码, stdout, 被拦清单)。

    script_dir/cwd 可以让那个脚本跑在一份临时目录里：`_abort_test.py` 的 main() 第一行
    就把活地图目录里的快照 copy 一份，判据不该为此要求"此刻正好有 bias.json"，
    更不该往你正在建的图旁边丢垃圾。
    """
    base = script_dir or HERE
    env = dict(os.environ, PYTHONPATH=HERE)
    p = subprocess.run([sys.executable, "-X", "utf8", "-c", BOOT, os.path.join(base, script)]
                       + list(args), capture_output=True, text=True, encoding="utf-8",
                       errors="replace", timeout=120, cwd=cwd or HERE, env=env)
    out = (p.stdout or "") + (p.stderr or "")
    code, blocked = None, None
    for line in out.splitlines():
        if line.startswith("GUARD "):
            d = json.loads(line[6:])
            code, blocked = d["code"], d["blocked"]
    return code, out, blocked


# ---------------------------------------------------------------- 0. 车此刻的位姿（独立对账用）
def live_pose():
    import urllib.request
    try:
        d = json.loads(urllib.request.urlopen("http://127.0.0.1:8010/api/state",
                                              timeout=4).read().decode("utf-8"))
        return d["pose"], d.get("busy"), d.get("motion", {}).get("n")
    except Exception:
        return None, None, None


POSE0, BUSY0, LIE0 = live_pose()
print("== 0. 起点：车 %s  busy=%r  机器中止计数=%r" % (POSE0, BUSY0, LIE0))
if POSE0 is None:
    print("   （:8010 没起：这一段的对账判据会跳过，其余照跑 —— 判据不该要求车在线）")

print("\n== A. helper 的语义（真跑，不看文本）==")
_saved = sys.argv
try:
    sys.argv = ["_estop_move_test.py"]
    rc = None
    try:
        require_drives("测试用说明")
    except SystemExit as exc:
        rc = exc.code
    ck("A1 只有脚本名（= 直接双击/批量 glob 的形状）→ 退出码 2，不是 0", rc, 2)
    sys.argv = ["_abort_test.py", "forward", "1.10"]
    rc = None
    try:
        require_drives("测试用说明")
    except SystemExit as exc:
        rc = exc.code
    ck("A2 有位置参数但没有那个词 → 照样拒（旧命令行一字不改地跑，正是这次出事的样子）", rc, 2)
    sys.argv = ["_abort_test.py", "drives"]
    rc = "run"
    try:
        require_drives("测试用说明")
    except SystemExit as exc:
        rc = exc.code
    ck("A3 给了词 → 放行（返回 None，不抛）", rc, "run")
    ck("A4 放行时把词从 argv 摘掉：调用方的 argv[1] 还是它原来的第一个参数",
       sys.argv, ["_abort_test.py"])
    sys.argv = ["_abort_test.py", DRIVES_TOKEN, "backward", "0.9"]
    require_drives("测试用说明")
    ck("A5 摘掉之后 argv[1]/argv[2] 与加门之前指向同一个参数（七个脚本各自的解析不用改）",
       sys.argv, ["_abort_test.py", "backward", "0.9"])
    sys.argv = ["_yaw_cause.py", "0.10", "drives"]
    rc = None
    try:
        require_drives("测试用说明")
    except SystemExit as exc:
        rc = exc.code
    ck("A6 词不在第一位也算没给（否则某个距离参数恰好撞上这个词就自己把门开了）", rc, 2)
finally:
    sys.argv = _saved

_gate_src = open(os.path.join(HERE, "drives_gate.py"), encoding="utf-8").read()
ck("A7 那个词只有一份定义", _gate_src.count('DRIVES_TOKEN = '), 1)
dupes = [s for s in SCRIPTS
         if "会真的让轮子转" in open(os.path.join(HERE, s), encoding="utf-8").read()]
ck("A8 拒绝文案没有第二份（七个脚本里不许各自刻一句，改文案只改一处）", dupes, [])
missing = [s for s in SCRIPTS if "require_drives(" not in
           open(os.path.join(HERE, s), encoding="utf-8").read()]
ck("A9 七个真车脚本一个都没漏（漏一个就等于它仍然能被 glob 批量执行）", missing, [])
late = []
for s in SCRIPTS:
    lines = open(os.path.join(HERE, s), encoding="utf-8").read().splitlines()
    gate = next((i for i, l in enumerate(lines) if "require_drives(" in l), 10 ** 6)
    argv = next((i for i, l in enumerate(lines)
                 if "sys.argv" in l and not l.strip().startswith("#")), 10 ** 6)
    if gate > argv:
        late.append(s)
ck("A10 门排在脚本自己读 argv 之前（排在后面就等于参数已经被解析过了）", late, [])

print("\n== B. 真子进程：不给词，七个脚本都得原地退出 ==")
for s in SCRIPTS:
    code, out, blocked = run_guarded(s, [])
    ck("%s 退出码 2" % s, code, 2)
    ck("%s 打了拒绝文案（含那个词）" % s, DRIVES_TOKEN in out and "轮子" in out, True)
    ck("%s 一个连接/子进程都没发出去" % s, blocked, [])

print("\n== C. 反向对照：锁本身会响（不然 B 的\"没响\"毫无意义）==")
canary = os.path.join(HERE, "_gate_canary.py")
with open(canary, "w", encoding="utf-8") as f:
    f.write("# -*- coding: utf-8 -*-\n"
            "import json, urllib.request\n"
            "print(json.loads(urllib.request.urlopen("
            "'http://127.0.0.1:8010/api/state', timeout=4).read())['pose'])\n")
try:
    code, out, blocked = run_guarded("_gate_canary.py", [])
    ck("C1 一个真会连 :8010 的小脚本，在这套锁下被拦（证明锁是装的、不是没通电）",
       bool(blocked) and blocked[0][0] in ("connect", "create_connection"), True)
    ck("C2 拦的时候报的是锁的名字，不是超时（看得懂是谁拦的）",
       "drives-gate-test" in out, True)
    # 不带锁跑同一段：必须真能连上（否则 C1 是"网络本来就不通"假绿）
    p = subprocess.run([sys.executable, "-X", "utf8", canary], capture_output=True,
                       text=True, encoding="utf-8", errors="replace", timeout=30, cwd=HERE)
    unlocked_ok = p.returncode == 0 and POSE0 is not None
    if POSE0 is None:
        print("SKIP C3（:8010 没起，无法证明不带锁时连得通）")
    else:
        ck("C3 不带锁时它连得上（所以 B 段那些 blocked=[] 是门拦的，不是车离线）",
           unlocked_ok, True)
finally:
    try:
        os.remove(canary)
    except OSError:
        pass

print("\n== D. 给了词就真放行：脚本要动车时被锁摁住 ==")
try:
    import rclpy  # noqa: F401
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False

ARGS = {"_abort_test.py": ["forward", "1.10"], "_cruise_w.py": [],
        "_estop_move_test.py": ["stop", "0.5"], "_honest_run.py": ["0.6"],
        "_machine_stop_test.py": ["0.8", "1"], "_turn_cold.py": ["1", "30", "turn_right"],
        "_yaw_cause.py": ["0.10", "0.2"]}
CACHE_DIR = os.path.join(HERE, "cache_console")
for s in SCRIPTS:
    if s == "_yaw_cause.py":
        if HAS_RCLPY:
            # 它自己发 cmd_vel 走 DDS，不是 TCP：这把锁摁不住它，所以带词的那一遍根本不能跑。
            print("SKIP %s 带词那遍（rclpy 可用，而 DDS 不经过被锁的 socket —— 跑了就是真动车）" % s)
            continue
        code, out, blocked = run_guarded(s, [DRIVES_TOKEN] + ARGS[s])
        ck("%s 带词 → 不再退出码 2（它卡在 import rclpy，那已经在门后面了）" % s, code != 2, True)
        ck("%s 带词 → 拒绝文案一个字都不该出现" % s, "轮子" not in out, True)
        continue
    before = set(os.listdir(CACHE_DIR)) if os.path.isdir(CACHE_DIR) else set()
    if s == "_abort_test.py":
        # 它 main() 第一行就 copy 活地图目录里的两份快照。判据不为此要求"此刻正好有
        # bias.json"（刚清空地图时那两份是被删掉的），也不往你的图旁边丢垃圾：
        # 把它连同一个假 cache_console 搬到临时目录里跑，门后面第一步照样是发 HTTP。
        tmp = tempfile.mkdtemp(prefix="gate_abort_")
        os.makedirs(os.path.join(tmp, "cache_console"))
        for f, blob in (("grid.npy", b"\x00"), ("bias.json", b"[0.0, 0.0]")):
            with open(os.path.join(tmp, "cache_console", f), "wb") as fh:
                fh.write(blob)
        shutil.copy2(os.path.join(HERE, s), os.path.join(tmp, s))
        code, out, blocked = run_guarded(s, [DRIVES_TOKEN] + ARGS[s], script_dir=tmp, cwd=tmp)
        shutil.rmtree(tmp, ignore_errors=True)
    else:
        code, out, blocked = run_guarded(s, [DRIVES_TOKEN] + ARGS[s])
    after = set(os.listdir(CACHE_DIR)) if os.path.isdir(CACHE_DIR) else set()
    litter = sorted(x for x in after - before if ".pre_abort_" in x)
    for x in litter:                                  # 它第一行就备份快照，那不是我让它写的
        try:
            os.remove(os.path.join(CACHE_DIR, x))
        except OSError:
            pass
    ck("%s 带词 → 不再退出码 2" % s, code != 2, True)
    ck("%s 带词 → 拒绝文案不再出现（证明放行的是这个词，不是脚本自己碰巧失败）" % s,
       "轮子" not in out, True)
    ck("%s 带词 → 它确实想往外发（被锁摁住，所以车没动）" % s, bool(blocked), True)
    ck("%s 带词 → 位置参数没被摘词摘错位（没有 float/int 解析炸）" % s,
       "could not convert string to float" not in out, True)

print("\n== E. 车真的没动（独立来源，不看脚本自述）==")
POSE1, BUSY1, LIE1 = live_pose()
if POSE0 is None:
    print("SKIP E（:8010 没起）")
else:
    ck("E1 整轮判据跑完，位姿一字未变", POSE1, POSE0)
    ck("E2 没有动作在跑", BUSY1, "")
    ck("E3 机器中止计数没涨（涨了就是真发过命令）", LIE1, LIE0)

print("\nDRIVES-GATE-TEST %s  通过 %d / 失败 %d"
      % ("PASS" if n_fail == 0 else "FAIL", n_pass, n_fail))
sys.exit(0 if n_fail == 0 else 1)
