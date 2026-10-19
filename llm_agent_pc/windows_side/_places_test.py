# -*- coding: utf-8 -*-
"""语义记忆层的离线回归：一个词钉在**地图自己的框**里，不是钉在车此刻的谎上。

为什么要有这份文件：`Remember` 在真车上只能验一次「记下去了、读得出来」，验不了它最要紧的那条
性质 —— 之后车被搬动、被重定位、bias 被漂移抑制器一分一分地花掉，那个词还贴着原来那堵墙吗。
要拿真车验这条，就得先把图弄脏；所以搬到桌面上跑，用 `car_web` 里**同一批方法**（绑到 Stub 上，
不复制实现），只在最后一步把 `png_bytes`/`ax_text` 换成记录器，好读到它到底把数字落在哪个像素。

六组判据：
  1 词和墙同框：bias 一跳 0.8 m，词一动不动（它跟着墙走，不跟着车此刻的自信）
  2 重名覆盖而不是并排出两个；记满 PLACE_MAX 之后第 41 个被拒，且不许把已有的挤掉
  3 方位沿用 Move 的约定：正=向左，背后折进 ±180 而不是报 350°
  4 清空地图 = 换了一个框：词全部作废、places.json 删掉、建图重新打开
  5 图上那个数字就是列表里那一行：紫色点的连通块与 ax_text 记录的编号一一对上，
    而且关掉坐标轴也得有数字（曾经只在 axes 打开的那个分支里画）；车站在词上时红点盖住紫点
  6 中文标签存得回来（json 必须 ensure_ascii=False，否则文件里是 \\u5ba2\\u5385）
  7 `MapInfo` 那句地点话本身：只列最近 4 个时要把"只列"说出来、列的必须是最近的 4 个、
    ≤4 个时不许谎称还有、要说"全部列出"、坐标是减过显示原点的那一套

Run: C:\\microros-agent\\pc_side\\_rosrun.bat _places_test.py
"""
import json
import math
import os
import sys
import tempfile
import threading

import numpy as np

import car_web
import nav

fails = []


def check(name, got, want):
    if got == want:
        print("✓ %s" % name)
        return
    print("✗ %s\n     got=%r want=%r" % (name, got, want))
    fails.append(name)


class Car:
    """只长 pose/lock/pairs/bias 四样：pose = 里程计 + bias，和 bridge 里同一个式子。"""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.lock = threading.Lock()
        self.odom = (x, y)
        self.yaw = yaw
        self.bias = (0.0, 0.0)
        self.pairs = []
        self.volts = 12.0                       # map_view 会念电池，缺了它整句就拼不出来
        self.pose = (x, y, yaw, 0.0)

    def set_bias(self, bx, by):
        # car_mcp_server._bias_locked 干的就是这件事：已经握着的那条 pose 一起搬过去，不留一帧混叠
        self.bias = (float(bx), float(by))
        self.pose = (self.odom[0] + self.bias[0], self.odom[1] + self.bias[1],
                     self.yaw, self.pose[3])


class Stub:
    """绑真方法，不复制：复制一份出来，它和 car_web 里那个迟早会不一致。"""

    _save_places = car_web.Console._save_places
    _load_places = car_web.Console._load_places
    add_place = car_web.Console.add_place
    del_place = car_web.Console.del_place
    places_near = car_web.Console.places_near
    render_map = car_web.Console.render_map
    reset_map = car_web.Console.reset_map
    map_view = car_web.Console.map_view

    def __init__(self, pose=(0.0, 0.0, 0.0)):
        self.lock = threading.Lock()
        self.car = Car(*pose)
        self.world = nav.World()
        self.trail = []
        self.origin = (0.0, 0.0)
        self.places = []
        self.map_cache = {}
        self.logs = []
        self.mapping = False
        self.busy = ""
        self.tour = None
        self.drift = {"n": 0, "total": 0.0, "est": [0.0, 0.0]}
        self.motion = {"at": None, "outside": None, "bad": 0, "n": 0, "why": None}
        self.reloc = {"frozen": True, "agree": None, "best": None, "cand": [], "trial": None}
        self.watch_hist = []
        self.watch_bad = 3
        self.reloc_tries = 2
        d = tempfile.mkdtemp(prefix="_places_")
        self.cache_file = os.path.join(d, "grid.npy")
        self.bias_file = os.path.join(d, "bias.json")
        self.places_file = os.path.join(d, "places.json")
        self.origin_file = os.path.join(d, "origin.json")

    def push(self, text, kind="info"):
        self.logs.append((kind, text))

    def _drift_reset(self):
        pass


def cell_of(stub, x, y):
    """词落在第几格 —— 这才是"词和墙同框"的说法：格子编号不变，词就还贴着那面墙。"""
    return (int((x - stub.world.ox) / stub.world.res),
            int((y - stub.world.oy) / stub.world.res))


# --------------------------------------------------------------- 1 同框 ----
st = Stub()
st.add_place("充电器")
c0 = cell_of(st, st.places[0]["x"], st.places[0]["y"])
st.car.set_bias(0.8, 0.5)                       # 等价于人工「选偏移」把位姿平移 (+0.8, +0.5)
check("bias 跳了 0.8 m，词还钉在原来那一格上（记的时候车在 (0,0)）",
      [(p["label"], p["x"], p["y"]) for p in st.places], [("充电器", 0.0, 0.0)])
check("  词所在的格子编号一个都没变", cell_of(st, st.places[0]["x"], st.places[0]["y"]), c0)
check("  车此刻的自报位置倒是跟着 bias 跳走了（被修的从来是 pose，不是地图）",
      [round(v, 2) for v in st.car.pose[:2]], [0.8, 0.5])

# ------------------------------------------------ 2 重名覆盖 / 上限 ----
st2 = Stub()
st2.add_place("沙发")
st2.car.pose = (1.0, 0.0, 0.0, 0.0)
st2.add_place("沙发")
check("同名再记 = 覆盖，不是并排出两个", [(p["label"], p["x"]) for p in st2.places],
      [("沙发", 1.0)])
check("  覆盖要在日志里说出来（不然人以为记了两条）", any("覆盖" in t for _, t in st2.logs), True)
st3 = Stub()
for i in range(car_web.PLACE_MAX):
    st3.car.pose = (i * 0.01, 0.0, 0.0, 0.0)
    st3.add_place("p%d" % i)
r = st3.add_place("挤不进来的第 41 个")
check("记满 %d 个之后第 41 个被拒，且一条没掉" % car_web.PLACE_MAX,
      (len(st3.places), "先 Forget" in r), (car_web.PLACE_MAX, True))
r = st3.add_place("p0")
check("  到顶时重名覆盖仍然要成功（先删后判，不许把自己挤掉）",
      (len(st3.places), r.startswith("已记")), (car_web.PLACE_MAX, True))
check("空标签不记账", (st3.add_place("   "), len(st3.places)),
      ("空的标签，没记。", car_web.PLACE_MAX))
st4 = Stub()
st4.car.pose = None
check("没有位姿时不许钉词（否则会把词钉到某个假默认值上）",
      ("从未到过" in st4.add_place("哪里"), len(st4.places)), (True, 0))
st4b = Stub()
st4b.add_place("电视机旁")
check("Forget 允许只说一半（人记不住全名）", (st4b.del_place("电视"), len(st4b.places)),
      ("已忘记「电视机旁」（剩下 0 个）", 0))
check("  说没记过的词要报现在记着什么，别让人猜",
      st4b.del_place("阳台").startswith("没找到「阳台」"), True)

# ------------------------------------------------------- 3 方位约定 ----
st5 = Stub(pose=(0.0, 0.0, 0.0))                # 车头朝 +x，于是 +y 在左手边
for lab, (x, y) in {"左": (0.0, 1.0), "右": (0.0, -1.0), "前": (1.0, 0.0),
                    "后": (-1.0, 0.0)}.items():
    st5.places.append({"label": lab, "x": x, "y": y, "yaw": 0.0})
rows = {r["label"]: r["bearing"] for r in st5.places_near(st5.car.pose)}
check("方位沿用 Move 的约定：正=向左", [rows["左"], rows["右"]], [90.0, -90.0])
check("  正前方 0°，身后折进 ±180 而不是报 350°", [rows["前"], abs(rows["后"])], [0.0, 180.0])
check("  按远近排出来（四个都 1 m，只验长度与不炸）",
      [r["dist"] for r in st5.places_near(st5.car.pose, limit=4)], [1.0, 1.0, 1.0, 1.0])
st5.car.pose = (0.0, 0.0, math.radians(90), 0.0)      # 车头转朝 +y
rows = {r["label"]: r["bearing"] for r in st5.places_near(st5.car.pose)}
check("车头转 90° 之后，原来到手边的现在在正前方", rows["左"], 0.0)

# ---------------------------------------------------- 4 清空即换框 ----
st6 = Stub()
st6.add_place("要被忘掉的东西")
check("记下去的时候 places.json 真的落盘了", os.path.exists(st6.places_file), True)
st6.reset_map()
check("清空地图把词一起作废（新图 = 新框，旧坐标属于一间不存在的屋子）",
      (len(st6.places), os.path.exists(st6.places_file)), (0, False))
check("  清空之后建图是开着的（这条修的是「按钮说重新开始、其实什么都没画」）",
      st6.mapping, True)
check("  日志要报忘掉几个（人得知道手滑的代价）",
      any("忘掉 1 个地点" in t for _, t in st6.logs), True)

# ------------------------------------------------ 5 图上的数字 ----
def rendered(stub, axes):
    """跑一遍真渲染，只把最后两步换成记录器：读到它把数字落在哪个像素、车点盖没盖住地点点。"""
    calls, held = [], {}
    real_text, real_png = car_web.ax_text, car_web.png_bytes

    def spy_text(img, x, y, s, col, sc=2):
        calls.append((str(s), int(x), int(y), tuple(col), int(sc)))

    def spy_png(arr):
        held["up"] = np.array(arr)
        return b""

    car_web.ax_text, car_web.png_bytes = spy_text, spy_png
    try:
        stub.render_map(4.0, False, "car", axes)
    finally:
        car_web.ax_text, car_web.png_bytes = real_text, real_png
    return calls, held.get("up")


def blobs(up):
    """紫色（PLACE_COL）连通块的中心：一个点 + 压在它身上的编号，正好一块。

    按精确等值挑色，不用"接近"：这张图上的每个颜色都是直接写进数组的，没有抗锯齿也没有混合，
    而坐标轴那条粉线 (255,92,160) 和地点以前用的品红 (255,90,200) 只差在蓝通道上 —— 用阈值就会
    把轴线和地点搅成一坨，那正是这次改颜色的原因。
    """
    m = (up[:, :, 0] == PLACE[0]) & (up[:, :, 1] == PLACE[1]) & (up[:, :, 2] == PLACE[2])
    seen, out = set(), []
    for r0, c0 in np.argwhere(m):
        if (r0, c0) in seen:
            continue
        todo, blob = [(int(r0), int(c0))], []
        seen.add((int(r0), int(c0)))
        while todo:
            r, c = todo.pop()
            blob.append((r, c))
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                k = (r + dr, c + dc)
                if k not in seen and 0 <= k[0] < up.shape[0] and 0 <= k[1] < up.shape[1] \
                        and m[k[0], k[1]]:
                    seen.add(k)
                    todo.append(k)
        if len(blob) >= 40:                     # 一个点 15x15=225 像素，碎屑进不来
            out.append((sum(p[0] for p in blob) / len(blob),
                        sum(p[1] for p in blob) / len(blob), len(blob)))
    return out


st7 = Stub(pose=(0.0, 0.0, 0.0))
for lab, (x, y) in {"甲": (-1.2, 1.2), "乙": (1.2, 1.2), "丙": (0.0, -1.5)}.items():
    st7.places.append({"label": lab, "x": x, "y": y, "yaw": 0.0})
calls, up = rendered(st7, axes=False)
# 只认地点那一色：开着坐标轴时 draw_axes 也往同一张图上打黄色刻度数字，光看"是不是数字"分不开
PLACE = car_web.PLACE_COL
digits = [c for c in calls if c[0].isdigit() and c[3] == PLACE]
check("关掉坐标轴也有编号（以前只在 axes 那个分支里画，等于默认看不见）",
      sorted(c[0] for c in digits), ["1", "2", "3"])
bs = sorted(blobs(up), key=lambda b: b[1])       # 甲乙在上、丙在下
check("图上有三块紫点（三个点各一块，编号长在点身上）", len(bs), 3)


def owner(digit):
    """数字左上角 -> 它压着哪一块：ax_text 画在 (px-6, py-14)，字心约在 (+4, +5) 处。"""
    gx, gy = digit[1] + (4 * digit[4] - 1) / 2.0, digit[2] + 2 * digit[4] + 1
    i = min(range(len(bs)), key=lambda k: math.hypot(bs[k][1] - gx, bs[k][0] - gy))
    return i, math.hypot(bs[i][1] - gx, bs[i][0] - gy)


# 世界坐标从图上看不出来，但左右顺序看得出来：丙(x=0) 夹在甲(-1.2) 和乙(+1.2) 中间
owner_of = {int(d[0]): owner(d) for d in digits}
check("  每个数字都真的压在自己那块上（离块心 < 20 px）",
      all(dist < 20 for _, dist in owner_of.values()), True)
check("  1=甲(最左) 2=乙(最右) 3=丙(中间)：列表顺序就是图上编号",
      [owner_of[1][0], owner_of[2][0], owner_of[3][0]], [0, 2, 1])
calls_ax, _ = rendered(st7, axes=True)
check("  开着坐标轴时编号照旧在（黄色刻度数字不算数）",
      sorted(c[0] for c in calls_ax if c[0].isdigit() and c[3] == PLACE), ["1", "2", "3"])

st9 = Stub(pose=(0.0, 0.0, 0.0))
st9.places.append({"label": "脚下", "x": 0.0, "y": 0.0, "yaw": 0.0})
_, up9 = rendered(st9, axes=False)
cy, cx = up9.shape[0] // 2, up9.shape[1] // 2
crop = up9[cy - 24:cy + 24, cx - 24:cx + 24].reshape(-1, 3)
red = int(((crop[:, 0] > 200) & (crop[:, 1] < 110) & (crop[:, 2] < 110)).sum())
check("车站在词上时红点盖住紫点（人第一眼要看见的是「我在这儿」）", red >= 400, True)
check("  中心那一格不是地点色（没被地点吃掉）",
      tuple(up9[cy, cx]), (235, 60, 60))

# ---------------------------------------------------- 6 中文能存回来 ----
st8 = Stub(pose=(0.42, -0.31, 0.0))
st8.add_place("客厅中间")
raw = open(st8.places_file, encoding="utf-8").read()
check("落盘用的是真汉字，不是 \\u5ba2\\u5385（人也要能读这个文件）", "客厅中间" in raw, True)
back = Stub()
back.places_file = st8.places_file
back._load_places()
check("重启读回来一模一样", [(p["label"], p["x"], p["y"]) for p in back.places],
      [("客厅中间", 0.42, -0.31)])
bad = Stub()
with open(bad.places_file, "w", encoding="utf-8") as fh:
    fh.write('[{"label":"甲","x":1.0,"y":2.0},{"x":3,"y":4},{"label":"乙","x":0,"y":0,"yaw":0}]')
bad._load_places()
check("文件被手写坏过也不炸：缺字段的行丢掉，旁边的照旧读进来",
      [p["label"] for p in bad.places], ["甲", "乙"])
with open(bad.places_file, "w", encoding="utf-8") as fh:
    fh.write("not json at all")
bad._load_places()
check("  整个文件是垃圾时从空白开始，而不是把 bridge 拖崩", bad.places, [])

# ------------------------------------------------ 7 MapInfo 那句地点话 ----
# 手册把"一次只列最近 4 个"和"显示系减过原点"写成了承诺，那就得由回归钉住，不能靠我写文档时算对。
import re

st10 = Stub(pose=(0.0, 0.0, 0.0))
for i, (px_, py_) in enumerate([(0.5, 0.0), (1.0, 0.0), (1.5, 0.0),
                                (2.0, 0.0), (9.0, 0.0), (9.5, 0.0)]):
    st10.places.append({"label": "P%d" % i, "x": px_, "y": py_, "yaw": 0.0})
line = [l for l in st10.map_view().split("\n") if l.startswith("记住的地点")]
check("六个词只列得下四个，但句子要说清「共 6 个」和「按远近只列最近 4 个」",
      [len(line), bool(re.search(r"共 6 个，按远近", line[0] or "")),
       bool(re.search(r"按远近只列最近 4 个", line[0] or ""))] if line else [0, False, False],
      [1, True, True])
listed = re.findall(r"「(P\d)」", line[0]) if line else []
check("  列出来的是最近的四个（P0..P3），不是插入顺序的前四个凑数", listed, ["P0", "P1", "P2", "P3"])
st10.places = st10.places[:4]
line4 = [l for l in st10.map_view().split("\n") if l.startswith("记住的地点")]
check("  四个及以内不许出现「只列最近」这句（那就是全部，别让人以为还有）",
      bool(re.search(r"只列最近", line4[0])), False)
check("  但要说「全部列出」：列全了和截断了得能靠这句话区分开",
      bool(re.search(r"共 4 个，按远近全部列出", line4[0])), True)
check("  一个词都没记时整句不提地点（模型不会看到一句关于空气的话）",
      any(l.startswith("记住的地点") for l in Stub().map_view().split("\n")), False)
st10.origin = (1.0, 0.0)                          # 点过一次"把这里设为原点"
line_o = [l for l in st10.map_view().split("\n") if l.startswith("记住的地点")][0]
d0 = float(re.search(r"「P0」[\d.]+ m [+-]?\d+°（显示系 (-?[\d.]+), ", line_o).group(1))
check("  句子里那对坐标是**减过显示原点**的（P0 记在 0.5，原点挪到 1.0 之后该写 -0.50）",
      [d0, round(st10.places[0]["x"], 2)], [-0.5, 0.5])


print("\n%s" % ("全部通过" if not fails else "失败 %d 项: %s" % (len(fails), fails)))
sys.exit(1 if fails else 0)
