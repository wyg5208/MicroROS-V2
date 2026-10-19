# MicroROS-V2 本地大模型控制项目（Windows 原生，无 WSL / 无虚拟机）

亚博智能（Yahboom）MicroROS-V2 小车官方教程把 ROS2、micro-ROS Agent、MCP 服务、Dify/OpenClaw
后端全部放在一个 Ubuntu 虚拟机镜像里。本项目**不使用虚拟机、不使用 WSL、不添加车载电脑**，
把整套 ROS2 侧的运行时搬到笔记本 Windows 原生环境，并在 PC 侧自写了
建图 / 定位 / 规划 / 控制台 / MCP 桥接，让本机的 Ollama 大模型直接驱动车子。

- 适配硬件：MicroROS-V2 豪华版（ESP32-S3 控制板 + T-MINI PLUS 激光雷达 + ESP32-S3 图传云台相机）
- 适配固件：`MicroROS-V2-v1.1.0-complete.bin`（出厂闭源固件，未改一行）
- 代码位置：本项目自带的 `llm_agent_pc/`（PC 侧全部实现），仓库其余 24 个章节是厂家原始资料
- 实测状态：链路、建图、定位、规划、网页控制台、大模型工具调用全部在真车上跑通（详见《开发过程实录.md》）

---

## 1. 它是怎么连起来的

```
        ┌──────────────── 笔记本（Windows 11，一台机器，没有 Linux）─────────────────┐
        │                                                                          │
        │  Ollama qwen3.5:9b  (http://127.0.0.1:11434/v1)                          │
        │        │  OpenAI 兼容 + 工具调用                                          │
        │        ▼                                                                 │
        │  llm_agent_pc/agent.py            ← 纯标准库智能体（可换任何 OpenAI 兼容后端）│
        │        ▲  import run_turn / McpClient（同一条循环，两种外壳）                │
        │  llm_agent_pc/car_chat.py :8020   ← 网页聊天：输入框 + 每步回显 + 确认开关    │
        │        ▲  HTTP + SSE（浏览器打开 http://127.0.0.1:8020）                    │
        │        │  MCP streamable-http                                             │
        │        ▼                                                                 │
        │  car_mcp_server.py  ── :8000 ── MCP 工具服务端                              │
        │        │            └─ :8010 ── 网页控制台 car_web.py + car_web_ui.html    │
        │        │  rclpy（ROS2 Humble，Windows 原生）                                │
        │        ▼                                                                 │
        │  micro_ros_agent.exe  udp4/8090     ← 本项目自己编译的 Windows 版 Agent      │
        └────────┬─────────────────────────────────────────────────────────────────┘
                 │ WiFi UDP（XRCE-DDS）
            ┌────┴─────────────┐        ┌──────────────────────┐
            │ ESP32-S3 控制板   │  UART1 │ T-MINI PLUS 激光雷达  │
            │ micro-ROS 客户端  │◄───────┴──────────────────────┘
            │ 舵机 / 电机 / OLED │
            └──────────────────┘
            图传相机（独立 ESP32-S3 模组）── HTTP :81 /stream（MJPEG）→ PC 侧直接抓帧
```

关键点：**车上没有操作系统，只有固件**。ROS2 这一层整个跑在笔记本上，
所以"要不要虚拟机"这件事完全由 PC 侧决定，本项目的答案是不要。

## 2. 项目结构

```
Microros-V2/
├── README.md                    本文件
├── 开发过程实录.md               为什么原方案不可用 + 本次开发全过程与实测数据
├── 用户操作手册.md               下载 / 安装 / 部署 / UI 使用说明
│
├── llm_agent_pc/                ★ 本项目自研代码（与厂家源码完全分离）
│   ├── agent.py                 本地大模型智能体：MCP 客户端 + 工具调用循环（纯标准库）
│   ├── car_chat.py              网页聊天后端(:8020)：复用 agent.py 那条循环，SSE 回显每步
│   ├── car_chat_ui.html         聊天页面（单文件，无框架；每次请求从磁盘读，改完不用重启）
│   ├── _speak.ps1               朗读子进程：把文本用 `System.Speech` 念出来（或合成成 WAV 供测试）
│   ├── asr/                     本地语音输入：`_asr_setup.py` 取件脚本 + `bin/`whisper 预编译 + `models/`ggml 权重
│   ├── _voice_test.py           语音链路离线验收（转写真的听懂 + 朗读掐得掉 + 没出声能看出来 + 同音纠正表）
│   ├── _asr_audit.py            40 句语料回放正在服务的 /asr，量纠正表救回多少句（只读，不动车）
│   ├── 启动智能体.bat            起 agent.py（命令行外壳，指向本机 Ollama）
│   ├── 启动聊天页面.bat          起 car_chat.py 并自动开浏览器（日常用这个和大模型说话）
│   ├── car_cron.py              外部调度：到点把一句话发给 :8020 的会话（不碰 ROS，不自造 scheduler）
│   ├── cron_tasks.json          任务表（随包的两条都写 `enabled: false`，不显式打开就永远不发）
│   ├── 安装定时任务.bat          注册每分钟的 MicroROS-CarCron 并翻掉三个电源开关（不要管理员）
│   ├── 卸载定时任务.bat          只删那个计划任务，任务表与日志一律不碰
│   └── windows_side/            PC 侧运行时（真车跑的这一套，权威路径 C:\microros-agent\pc_side）
│       ├── car_mcp_server.py    ROS2 节点 + MCP(:8000) + 控制台宿主；唯一允许订阅的进程
│       ├── car_web.py           网页控制台后端(:8010)：点动、看门、标定、建图、搬动检测、重定位、漂移抑制
│       ├── car_web_ui.html      控制台前端（单文件，无框架；由 car_web.py 每次请求从磁盘读，改完不用重启）
│       ├── nav.py               地图+雷达融合的局部规划：Costmap / A* / 轨迹评分 / 前沿搜索
│       ├── mapper.py            独立占据栅格建图节点（写 cache/map.png；诊断与离图用）
│       ├── nav_gains.json       底盘真实响应系数（由控制台"从日志拟合"生成）
│       ├── selfcheck.py         链路自检：图传取帧 + 四个 ROS 话题是否在线
│       ├── mcp_probe.py         不用大模型逐个调 MCP 工具
│       ├── _mcp_call.py         单工具调用（`_mcp_call.py Move '{"direction":"forward","distance":0.3}'`）
│       ├── udp_sniff.py         Agent 未启动时，确认车的 XRCE-DDS 发现包有没有到达本机
│       ├── 启动Agent.bat         起 micro_ros_agent.exe udp4/8090
│       ├── 启动小车服务.bat       前台起 bridge（:8000 + :8010）
│       ├── 启动建图.bat          前台起独立 mapper.py
│       ├── 自检.bat / 检查ROS环境.bat
│       ├── _web_bridge.bat      后台（脱离终端）起 bridge，日志到 _server.log —— 日常用这个
│       ├── _rosrun.bat          在任何脚本前套上 ROS2 环境（凡 import rclpy/car_web/nav 的都要走它）
│       ├── _spawn.ps1           无头启动进程（WMI Win32_Process.Create）
│       ├── build_agent.bat      自编译 Windows 版 micro-ROS Agent
│       ├── fix_pixi_paths.bat   给 pixi 版 Humble 的 CMake 目标镜像出缺失的 python310.lib / numpy 头
│       ├── vehicle_config/
│       │   ├── car_config.py    串口改车载参数：dump / apply / reboot / factory
│       │   ├── wifi_apply.py    从 netsh 读已存 WiFi 密码并写入车（不打印、不落盘）
│       │   └── config_robot.py  厂家串口协议类（0x01 WiFi / 0x03 Agent IP / 0x06 DOMAIN_ID / 0x30 相机 …）
│       │                        厂家原件在 `22.程序源码汇总/ros功能源码/mircoros_v2-1.0.1.zip` 的
│       │                        `mircoros_v2/.system_cfg/robot_manager/config_robot.py`，需解出放同目录
│       └── _*_test.py …         离线验收与诊断脚本（详见 §6）
│
└── 1.产品简介 … 24.虚拟机镜像     厂家原封资料（教程 PDF、源码 zip、固件 bin、烧录工具、VMware 镜像说明）
```

## 3. 功能清单

### 3.1 提供给大模型的 MCP 工具（:8000，共 15 个 —— 数字以 `:8020/tools` 实测为准）

| 工具 | 作用 | 依据 |
|---|---|---|
| `Move(direction, distance, angle)` | 闭环走/转：forward / backward / turn_left / turn_right（单次建议 ≤0.5 m 或 ≤90°）；走不动时自己判"被挡住/打滑"并最多脱困两招 | 里程计净位移反馈 + `nav.achieved()` 相对判据 + 绝对下限 + 途中障碍中止 |
| `Stop()` | 立刻零速，并请求中断正在跑的 Tour/闭环 | — |
| `Unstick(back, aim)` | 主动脱困一次：倒退 `back` 米 + 读四周雷达/地图找缺口 + 朝最开的方向转 | 三路定性（雷达扇区 / 图上格 / 陀螺转角）|
| `Tour(seconds)` | 自主巡游建图：沿房间轮廓走、边写图边规划、能倒车出死角 | 全局占据栅格 + 前沿 |
| `MapSpin(deg)` | 原地转一圈补视角（低电或空间窄时比平移划算） | — |
| `ScanRing(sectors)` | 把 `/scan` 压成 N 个扇区距离，给模型当"身体感觉" | — |
| `CheckAhead()` | 行进方向走廊净空判定（含云台盲区楔的处理） | — |
| `SeeWhat()` | 从图传相机抓一帧 JPEG 存到 PC 本地 cache 并返回路径；`agent.py` 在同机上直接读该路径喂给视觉模型 | — |
| `AdjustCameraView(pitch, yaw)` | 云台相对转动（带限位） | — |
| `GetStatus()` | 位姿 / 朝向 / 电量 / 雷达新鲜度 / 里程计新鲜度 | — |
| `MapInfo()` | **车的全局自我印象**（只读，Tour 跑着也能调）：我在图中的位置 + 重定位修正 + 未测格数/最近前沿方位/12 扇区未测直方图、雷达↔地图吻合度与判定、漂移抑制已花、是否已冻结、电机归属 | `nav.frontier_view` + 三个检测器 |
| `RateTest(vx, wz, seconds)` | 单段速度标定爆发，喂给最小二乘拟合 | — |
| `Remember(label)` | 把**车此刻脚下那一点**钉成词（只读，不动车）：写进 `places.json`，坐标存地图框，所以 bias 被改掉时词跟着墙不动 | 与 `:8010` 的"地点"面板同一个 `POST /api/places` |
| `Forget(label)` | 忘掉一个词（只读）：全名最稳，2 字以上片段可批量，单字片段不参与匹配 | 同上 |
| `Beep(duration_ms)` | 蜂鸣（找车用） | — |

`agent.py` 侧的安全边界：`Move / TargetTrack / FollowLine / Navigation` 属"移动类"，
默认要人工确认；`--max-distance`（默认 1.0 m）、`--max-angle`（默认 360°）、`--max-steps`（默认 10）
是硬约束，`--dry-run` 只打印不下发。两个 .bat 都显式传 `--max-distance 1.5 --max-angle 90`；
截断发生在 `call` 事件**之前**（`agent.clamp_motion`），所以网页回显和模型下一轮读到的都是截断后的数，
不会"显示 2 米、跑 1.5 米"；提示词第 5 条里的那两个数字由 `agent.system_prompt(cfg)` 现填，改上限不用改提示词。
确认与输出都走 `cfg.hooks`（`ConsoleHooks` 用 stdin/stdout，
`car_chat.py` 注入 `WebHooks` 变成页面上的确认卡与 SSE 事件），**同一条工具循环，两种外壳**。
两条实测语义：拒绝 = 本轮定论（模型再请求也不重新问，直接按拒绝回给它，防止它拿重试磨人）；
60 秒没人点 = 按拒绝处理。

### 3.2 网页控制台（:8010，手机也能用）

摄像头画面与抓帧、雷达缩略图（叠在地图朝向上）、实时地图（含 odom 坐标轴、黄色车头箭头、
粉色原点十字）、点动方向键 + 速度滑条 + 急停、云台控制、蜂鸣、状态栏
（位置 / 朝向 / 里程计 / 雷达 / 电池 / 电机占用 / 建图 / 雷达↔地图 / 漂移抑制）、
速度标定面板（测一次、四个预设、从日志拟合、手写真实反馈）、操作日志、
"跑 Tour 绕行"按钮、以及搬动/重定位横幅（可点选候选位置）。

后端 REST 面：`GET /api/state|scan|map.png|shot|log|drift`，
`POST /api/teleop|stop|ptz|mapping|reloc|origin|calib|tour|gains|fit|feedback|beep|places`。

### 3.3 网页聊天入口（:8020，`car_chat.py` + `car_chat_ui.html`）

左边是车（地图 / 实时画面 / 位姿 / 电量 / 建图与冻结状态 / 15 个工具名，`:8020/tools` 实测），
右边是话（你的输入框、模型每一步的卡片、助手回复、急停与清空上下文）。要点：

- **过程摊开给看**：`call → result` 每条都带工具自报的原文与耗时；自主动作（`Move/Tour/MapSpin`）
  前后各读一次 :8010 的位姿，另起一行**实测位移/转向**。工具说"完成 0.12 米"和轮子真的走了
  0.114 米是两件事，页面同时给，别只信前者。
- **确认是输入框下面一个复选框**（`执行是否需要确认`，**默认不勾选 = 直接执行**，选择记在
  `localStorage`）：不勾选时不弹卡片，`call → result` 照旧摊开；勾选后才逐条等"允许/拒绝"，
  拒绝或 60 秒超时不碰电机。开关状态由每条 `/chat` 请求自带（服务端 `cfg.confirm` 按轮生效，
  一轮只跑一条所以安全）。`/approval` 一张卡只作废一次（两个标签页抢答时后来者拿 409）。
  不勾选不等于没有约束：`--max-distance` / `--max-angle`、`Move` 的 `--safe-distance` 净空检查、
  行进中重定位中止、急停全都照旧。
- **只有一条回合**：`/chat` 在执行中再发拿 409；急停任何时候可点。
- **它不碰 ROS**：只是 MCP(:8000) 与控制台(:8010) 的 HTTP 客户端，所以不需要 `_rosrun.bat`，
  也不需要为它重启 bridge；默认只监听 `127.0.0.1`，并拒绝跨站 POST（急停除外）。
- 刷新/重开标签页不丢过程：事件按 `seq` 环形保留 240 条，SSE 连上先补发快照再续流。
- **语音**：输入框旁 `按住说话`（按住=采音，松手=转写），`朗读回复` 复选框把助手回信念出来。
  细节见 §3.5。

### 3.5 本地语音：按住说话与朗读回复（:8020，零新增依赖）

**入口。** 聊天页输入框下面那一排：`按住说话` 按钮 + `朗读回复` 复选框；状态栏"语音"一行
显示 whisper 模型名、嗓名，念不出声时把那一次异常直接写在这行里。

**怎么走的（全程不出本机、不用 ffmpeg、不装 Python 包）。**
浏览器 `AudioContext({sampleRate:16000})` 采 `Float32Array` → 手拼 44 字节 RIFF 头 →
`POST /asr`（原始字节）→ 后端一次性起 `whisper-cli.exe -l zh -nt -sns --prompt <简体样例>` →
逐字繁转简 + "是不是人话"判据 → **文字只填进输入框**。
反方向：助手回复 → `POST /speak` → 起 PowerShell 用 `System.Speech` 念。

**三条设计取舍。**
1. **语音绝不直接动车。** 转写结果落在输入框，要你按"发送"才成为指令 —— 识别错一个字
   就是一次按键失误，不是一次撞墙。听错时页面写"没听清"，不塞一句胡话给你。
2. **一次性子进程，不做常驻服务。** 常驻要管端口、生命周期、崩溃重启；单条转写实测
   0.5-1.2 s，不值得。代价是每次起进程，所以同一时刻只接一条（第二条拿"上一条还在转写"）。
3. **急停同时闭嘴。** `/stop` 除了停车还 `speaker.stop()`，`朗读回复` 取消勾选立刻掐当前这句。

**装（一次）。** `python C:\microros-agent\asr\_asr_setup.py` 会取 whisper.cpp 的 **预编译 CPU 版**
（约 23 MB）与 `ggml-base.bin`（约 148 MB）放到 `asr/bin`、`asr/models`。
这台机器上 `github.com` 的 HTTPS 直连超时，脚本走的是量出来的另一条路：
`api.github.com/repos/ggml-org/whisper.cpp/releases/assets/<id>` + `Accept: application/octet-stream`
（支持 `Range` 续传），模型走 `hf-mirror.com`。脚本幂等，已就位就打"已就位"。

**必须装在纯英文路径下。** whisper.cpp 的 Windows 版按当前 ANSI 码页读命令行参数，路径里带中文时
模型打不开：在离线包 `0.离线软件包\07_PC侧代码\asr\` 里原地跑实测退 127，Python 起它退 0xC0000409
（fail-fast），一个字都不转 —— 像程序坏了，其实是地方不对。`_asr_setup.py` 一发现自己在带中文的路径
里就拒绝下载；`car_chat.py` 每次转写前先查 `exe/模型/临时目录` 三条路径，带中文直接回"挪到纯英文路径"，
状态栏"语音"那行也会写出来，不会让人对着一次崩溃猜。默认位置 `C:\microros-agent\asr` 是干净的。

**为什么是 base 而不是更大的模型。** 同一批 40 句真指令：base **0.5-1.2 s**、small 1.7-3.6 s、
medium-q5_0 **6.7-8.0 s 且准确率没换来东西**。慢一档换不到准就是纯亏。
繁体输出用两个杠杆治：简体中文 `--prompt`（26/40 → 5/40）+ 手抄 217 对繁简字对表（→ 0/40）。
`-sns` 治好"听不清就吐一串注音 `ㄇㄇㄇ`"那一类；剩下的短clip幻觉由 `looks_like_speech()` 挡在输入框外。

**朗读速度。** 实测约 **0.27 s/字**（24 字 6.5 s、12 字 4.5 s），所以一句长回复会念很久，
`--voice-chars`（默认 400）是截断上限，不是"念完为止"的保证。

### 3.4 自主性四步（全部真车验收）

1. **搬动检测 + 自保**：odom 说静止时，把当前雷达扫过地图做 ±1.6 m 平移搜索；
   发现"另一种解释更合适"就急停、停止折叠、出红色横幅。实测干净图 `agree=0.994`，
   人为把地图平移 1 m 后一次检查就恢复出 `(+1.00, +0.50)` 的偏移。
2. **自动重定位**：把找到的偏移写回位姿（`Car.bias`），走"试 → 验 → 定"三段门控，
   失败就整体退回并冻结；两个候选分不开时交给人点。地图 / 原点 / 偏置三件套同存同取。
3. **连续漂移抑制**：运动中也做 scan-to-map，门控后的残差用 EMA 小步高频补回位姿。
   真车 A/B：地图清晰度（平均离空格数）1.064 vs 1.113，2 格厚墙 161 vs 319 个。
4. **行中检查接进 Move/Tour**：电机被占用时仍在打分，连续两次"说不通"就中止命令，
   并把"是搬动"还是"地图画错了"分开判定（看得穿墙的假墙走琥珀色横幅，不改位姿不停建图）。

### 3.6 定时任务（外部调度，`car_cron.py`，零新增依赖）

第 5 步补的是**"到点谁开口"**这一层：`安装定时任务.bat` 往任务计划程序注册一个每分钟醒一次的
`MicroROS-CarCron`，它只跑 `car_cron.py --once`；脚本自己判"这一班该不该发"（同一天同一班只发一次、
宽限不跨午夜、`enabled=false` 永不发），到点就 `POST :8020/chat`（`confirm=false`）把那句话
**当成用户打的话**送进同一条会话。所以：**没有在 bridge 里造第二个 scheduler**（所有 ROS 订阅仍只住
在 `car_mcp_server.py` 一个进程里），也没有第二套确认、第二个日志、第二个急停 —— 结果就在 `:8020` 页面看。

发之前先体检，读的都是现成端点：`:8020/state` 的 `chat.busy`、`car.online/busy/volts/mapping/frozen/agree/odom_age`
+ `:8010/api/state` 的 `occ`。任何一条不过就跳过并写人话理由（同一串理由一天只骂一次），
被拦记 `blocked` ≠ 已发，车回来当天宽限内仍可补跑。三班兜底：`cron.pause` 文件（全局停发，连手动点火也挡）、
`:8020` 页面的急停、每条任务的 `max_minutes` 到点自己 `POST :8020/stop`。

两个只有真跑才撞上的事实：① **任务计划程序建出来的任务默认"只在交流电下启动、错过不补"**，
`schtasks` 的命令行不暴露这三个开关 —— 这台车的上位机是笔记本（2026-10-19 的语音实测就是拔着充电线做的），
按默认值等于一拔电源就**永远不发且日志一个字都没有**（任务根本没醒，轮不到脚本回答）。bat 用
`Set-ScheduledTask` 翻掉它们并把读回的值念出来。② bat 里注册的那条命令、`car_cron.py --self-test`
第 10 段会去**读 bat 的文本**逐条核对（`car_cron.py --once`、三个开关、同一个任务名、两份 bat 都纯 ASCII），
两边各写各的早晚会漂 —— 和"prompt 里的工具名要对着 `tools/list` 核"是同一类检查。

**验到哪一步**：离线判据 55 项全过；真服务上的 `--dry-run` 体检；计划程序→cmd→python→日志这条链
用真任务装过、醒过、删过；两个 bat 各双击一遍。**没做的**：一班真的动作都没发过（2026-10-20 全程
`:8020` 的 `turns` 恒为 0、位姿一个数没变）。第一次真跑请挑只读的那条、人在旁边看。

## 4. 技术栈

| 层 | 选型 | 为什么 |
|---|---|---|
| 机器人运行时 | **ROS2 Humble，Windows 原生**（官方 pixi 版 zip 解压到 `C:\opt\ros\humble\x64`） | 免虚拟机；话题/消息直接用标准 `sensor_msgs/LaserScan`、`nav_msgs/Odometry` |
| DDS↔单片机 | **micro-ROS / XRCE-DDS**，Agent 为本项目**自编译**的 `micro_ros_agent.exe`（`udp4 --port 8090`） | Humble 不提供 Windows 预编译 Agent；厂家方案把它关在 VM 里 |
| Python | **3.10**（`C:\Users\Leo\AppData\Local\Programs\Python\Python310`），唯一第三方依赖 `numpy`（`rclpy` 硬要求）+ `pyserial`（配网） | `rclpy` 只支持 3.10；3.12/3.14 导入即失败 |
| MCP 服务端 | **标准库 `http.server.ThreadingHTTPServer` 手写**（`car_mcp_server.py` 内的 `McpServer`） | 本机已装的 `mcp` 是 2.x（`FastMCP` 已改名），不为一个桥去降级依赖 |
| MCP 客户端 / 智能体 | **纯标准库**（`agent.py`：JSON-RPC over POST，兼容 json 与 SSE 两种响应） | 无依赖、可读、可换后端 |
| 大模型 | **Ollama + `qwen3.5:9b`**（本机 :11434，具备 tools + vision） | 全离线；官方 `GetBbox` 走阿里云 dashscope，本项目不碰那个 key |
| 建图 | **自写对数几率占据栅格**（`mapper.py` / `car_web.py`） | 这台 Windows ROS2 只有 tf2 + laser_geometry，**没有 slam_toolbox / cartographer / nav2** |
| 定位 | **自写 scan-to-map 平移搜索**（饱和距离变换 + 高斯衰减打分，`nav.scan_offset`） | 同上；且只要平移搜索（旋转已被陀螺仪记进 odom yaw） |
| 规划 | **自写 Costmap + 8 邻域 A\* + 动态窗口轨迹评分**（`nav.py`），倒车是合法候选 | 反应式规则顶住墙还在走；换成"地图+实时雷达取 max 的代价栅格 + 可倒车轨迹"才能脱困 |
| 前端 | **单文件原生 HTML/Canvas/JS**，无框架、无 CDN、无打包 | 断网可用；每次请求从磁盘读，改完刷新即生效 |
| 语音 | **whisper.cpp 预编译 CPU 版**（一次性子进程）+ **Windows 自带 `System.Speech`** + 浏览器 Web Audio | 全离线、零新增 Python 依赖、**整条链路不需要 ffmpeg**；`ggml-base` 在这台 CPU 上 0.5-1.2 s，用不上 5090 也无所谓 |
| 图像 | 图传相机 HTTP（`:81 /stream` MJPEG、单帧接口） | 与 ROS 链路无关，取帧失败不影响控制 |
| 配网 | **pyserial + 厂家串口协议**（Type-C，115200） | WiFi 密码只需配一次；日常走 WiFi |

## 5. 硬约束（改动这套代码前必须知道）

1. **所有 ROS 订阅必须活在一个进程里。** 新建的 rclpy 节点收不到车上任何话题
   （同一时刻 bridge 的 `/scan`、`/odom_raw` 是 0.0 秒新鲜，独立节点 8~12 秒 0 帧）。
   所以控制台、建图、定位、规划、诊断全部写在 `car_mcp_server.py` 这一个长驻进程内，
   `car_web.py` 是它的线程而不是另一个节点。
2. **杀掉 bridge 进程 = 车立刻停**（板子自带看门清）。这是目前唯一可靠的急停手段；
   `Stop()` 与 `/api/stop` 停得住点动和 Tour，原本**压不住一条正在跑的 `Move`**——它的循环不读
   `nav_stop`（聊天页做急停按钮时撞上的，见第 14 部分）。已补，2026-10-18 重启后真车验证：
   命令 0.50 m，按下急停时走了 0.110 m，Move 回包"收到停止指令（已完成 0.12/0.50）"，
   最终停在 0.140 m（滑移 0.030 m）。**当时同一条通道还会被机器自己的搬动误判按下**，
   误判于是从"被无视"升级成"真的把车停住"——现已分成两条：人工急停走 `nav_stop`，
   机器自停走 `abort_reason` 带原因中止（见第 15 部分）；
3. **订阅 micro-ROS 话题必须 `BEST_EFFORT`**，reliable 订阅静默收不到。
4. **Agent 必须 `udp4`**（Windows 上 `udp6` 收不到 ESP32 的 IPv4 包）。
5. **`ROS_DOMAIN_ID` 必须在 `rclpy.init()` 之前落到环境变量**（本项目为 40），
   只传命令行参数会连到 domain 0 上"话题全都不见了"。
6. **这车没有 `/odom`**，板上只发 `/odom_raw`；`/odom` 是厂家 VM 里融合出来的。
7. **正前方约 44° 是硬件盲区**（探进扫描面的相机云台自己），机体 −24..+20° 恒为 0.06~0.11 m。
   桥接层按 `SELF_RANGE_MAX=0.15` 剔自回波、`BLIND_WEDGE=(-24,20)` **当未知而不是空旷**。
8. **底盘响应远差于命令值**（拟合自真车）：`v_act = 0.82·v_cmd`、
   `w_act = 0.857·w_cmd − 0.101·sign(w)`，低于 **0.118 rad/s 完全不转**。
   规划器与仿真都按真实响应预测弧线（`nav.achieved()`），系数会随电量漂，用控制台重新拟合。
9. **地图坐标手性已定案**：`SCAN_SIGN=1.0`，原始雷达角是标准逆时针，与 odom yaw 同向，地图 +y 是车的左。
   地图永远按 **odom 轴**画，显示原点可平移但**绝不旋转**。
10. **"无回波"不等于空旷**（暗布 2 m 就没回波），必须留 UNKNOWN，否则凭空长出穿透墙的白走廊。
11. **手挪车对里程计完全不可见**，且车重启会把 `/odom_raw` 归零 → 旧图与坐标系脱钩。
12. **低压锁**：3S 锂电低于约 10.2~10.5 V 时蜂鸣器"滴—滴—滴"且固件拒绝出力，
    症状像"链路坏了"但 odom/scan 仍然新鲜。`Tour` 在电量过低时直接拒绝执行。
13. 厂家 `robot_info.json` 里的 dashscope 云 API key **不使用、不外传**；
    `wifi_apply.py` **不打印、不落盘** WiFi 密码。
14. **语音的六条经验，改这条链路前要知道。** ① whisper 听中文**默认吐繁体**，
    必须同时给简体 `--prompt` 与逐字繁转简表，只做一半仍然看得见繁体；
    ② 听不清时它会**编**（`ㄇㄇㄇ`、`補药痘`、`傾斜`），`-sns` 治掉注音那一类，
    剩下的靠 `looks_like_speech()` 挡；注意 `str.isalnum()` 对注音 `ㄅ`(U+3105) 返回 True，不能拿来当判据；
    ③ **`-pi` 不存在**，长写法是 `--prompt`；④ 中文文本**不要塞命令行**（上游 shell 会吃字，
    `.ps1` 只收 UTF-8 文件路径），也**不要用"日志安静"证明念出声了** —— 看 `tts.last` 的时长；
    ⑤ **`asr` 只能装在纯英文路径**：whisper 按 ANSI 码页读 argv，路径带中文时打不开模型
    （实测退 127 / 崩 0xC0000409），看着像程序坏了。`Asr.path_error()` 会在起子进程前把它拦下来
    并写进状态行，`_asr_setup.py` 也一样会拒绝。
    ⑥ **同音字要靠一张纠正表，不能只怪模型**。`网前走依米`→`往前走1米`、`延强边`→`沿墙边`
    这类错，`ASR_FIX`（`car_chat.py`）在 `looks_like_speech()` 之后、进输入框之前逐条换掉，
    换到**不再变化**为止（最多 4 遍，因为"沿墙边"会先被听成"言强边"、一次过只救回一半）；
    改动会原样显示在页面（"原听成…纠了…"）让人一眼能否决。**这张表和 prompt 是绑死的**：
    换模型档或改 `--asr-prompt` 之后必须重跑 `_asr_audit.py`（回放 40 句测试语料）再补表，
    实测加过 prompt 之后旧错没了、新错（`少描移下`）才冒出来。有两类**故意不进表**：
    `继续`听成`倾斜`（倾斜是真词，改了就是吞指令）、带书名号的整段（whisper 幻觉，不是同音字）。


## 6. 离线验收脚本（不需要车，改代码后跑）

| 脚本 | 覆盖 |
|---|---|
| `_motion_test.py` | 第 4 步行中检查的六个用例：该停的停了、不该停的一次都没停、"地图画错了"必须走否决门而不是靠运气（含 `no_veto()` 反证） |
| `_stall_test.py` | 停滞检测与脱困：真值表（直行/转向 × 全速/爬行 × 窗口边界）、真车日志回放零误报、雷达/地图/陀螺三路定性、冻结位姿下跑真 `_closed_loop` 验"只试两招就收手、不烧死线、嵌套不擦 `nav_stop`"、**打滑用例**（里程计在涨而身后雷达不认账 → 阶梯必须打得着）|
| `_voice_test.py` | 语音链路八段：繁简字对表自审 → `Speaker` 的闸 → `Asr` 的三道输入门（太短/不是 WAV/太大，必须**不起子进程**就退回）→ SAPI 合成一句中文指令让**真** whisper-cli 听写 → 临时 HTTP 验 `/asr` 原始字节与状态码 → 念长句掐得掉 → **"没出声"要能看出来 → "装在带中文的路径里"要当场认出来 → 同音纠正表**（16 个必须等值 + 链式 + 幂等，且**40 句本来就对的原话一句都不许被改坏**）。第 6 段会真出声；缺 whisper 时第 3 段 SKIP 而非 FAIL |
| `_asr_audit.py` | 把 40 句测试语料灌进**正在服务的** `/asr`，分"本来就对 / 靠纠正表救回来 / 还不对"三档打表（当前 27+11=**38/40**）。只读：不碰 bias、不发电机命令。改完 prompt 或换模型档就重跑它，再把新错补进 `ASR_FIX` |
| `_reloc_test.py` | 合成矩形房间上的搬动检测/重定位：honest 0.999、搬走 0.010→0.999、单次搜索 0.5 ms |
| `_drift_test.py` / `_drift_tune.py` / `_drift_stats.py` | 漂移抑制的门控、限速、净偏移上界与参数扫描 |
| `_nav_test.py` / `_sim.py` / `_time_nav.py` | 规划器与真实响应模型的行为、耗时 |
| `_why_mismatch.py` / `_predict_abort.py` | 从磁盘地图 + 现场雷达反推"为什么这次对不上"；**下发电机命令前先在纸上跑一遍** |
| `_map_check.py` / `_where.py` / `_live_sharp.py` / `_edge_test.py` | 玫瑰图对照、方位查格、地图清晰度（离空距离/同方位多表面） |
| `_scan_stability.py` / `_odom_stability.py` / `_imu_probe.py` / `_yaw_jitter.py` | 传感器与位姿是否可信（排查顺序：先各自证明，再怀疑融合） |

跑法：`_rosrun.bat <脚本名>`（`import car_web` / `nav` / `rclpy` 的都必须走它）。
只发 HTTP 请求的 stdlib 脚本用 PATH 上的 python（3.14）即可，但要把 stdout 重配成 UTF-8，
否则 GBK 控制台会崩在中文上。

## 7. 端口与产物

| 端口 | 用途 |
|---|---|
| UDP 8090 | micro-ROS Agent（车 → PC） |
| TCP 8000 | MCP streamable-http（`/mcp`） |
| TCP 8010 | 网页控制台 |
| TCP 8020 | 网页聊天入口（默认只绑 `127.0.0.1`，`--chat-host` 可改） |
| TCP 81（车侧相机） | 图传 HTTP / MJPEG |
| TCP 11434 | Ollama |

磁盘产物（都在 PC 侧代码目录）：
`cache_console/grid.npy + origin.json + bias.json`（控制台地图三件套，**必须同存同取**；
每 5 s 自动存盘、启动时形状一致就恢复。地图 PNG **不落盘**，`/api/map.png` 是按需渲染的），
`cache_console/places.json`（记住的"地点"，**独立于 grid.npy**：备份图要连它一起备份，清空地图会把它一并删掉），
`cron_tasks.json`（定时任务表）+ `_cron.log` / `_cron_last.json` / `_cron_fired.json`（调度流水、最近一班、去重表）
+ `cron.pause`（**空文件一存在就全局停发**）+ `_cron_task.log`（计划任务那层的崩溃输出，正常为空），
`nav_gains.json`（拟合出的底盘系数）、`feedback_log.jsonl`（人工口述真值）、
`_server.log`（后台 bridge 日志）、`cache*/`（`mapper.py` 的 `map.png` + `map.json`，以及历次实验快照）、
`asr/bin/`（whisper 预编译，约 23 MB）、`asr/models/ggml-base.bin`（约 148 MB，实际用的就这一个；
`ggml-small.bin` 与 `ggml-medium-q5_0.bin` 是选档时 A/B 留下来的，可以删）、
`asr/req/`（每次转写的临时 WAV，用完即删）、`asr/say/`（每次朗读的临时文本，念完即删）、
`_chat_server.log`（聊天页日志，`[speak]` 行是"这句可能没出声"的落点）。

## 8. 接下来做什么（未完成项）

- **语音这一半：真麦克风已经跑通了**（2026-10-19 拔线后用户说话连发几条前进指令，车确实走了；
  内置预览浏览器不给 `getUserMedia`，只能在真浏览器里做）。仍然缺的是**真人耳听一次朗读**
  —— 程序只能证明"念了 23 s / 85 字"，证明不了咬字对不对；
  每一次转写现在都往 `_chat_server.log` 落一行 `[asr] 60.3 KB -> 「向前50厘米」0.73 s`，
  以前只有口头一句"刚才我用语音走过"，事后无从核对；
- **打滑这一半已经真车复现**（2026-10-19：书挡在轮前，车头不动、书被推走）—— 它暴露出停滞判据
  原来只有里程计一把尺子，而打滑时撒谎的正是里程计，现已由"身后雷达环"背书（只在前进时用，
  倒车方向被云台自遮挡且扇区最近回波会换目标，误报过一次真后退）。**仍缺**：真·物理顶住不动
  （`blocked`，要 ≤15 cm 硬矮障，书太软会被推着走），以及"脱困途中按急停"的物理版
  （现在只有桩件用例）；
- `STALL_* / ESCAPE_*` 这些数字是在插着电源线的 ≤0.5 m 短命令上量的；**≥1 m 的直线已经补量**
  （2026-10-19 拔线：前进 1.0 m 实测 0.98 m 峰差 1.5°、后退 1.0 m 实测 0.98 m 峰差 1.1°、
  原地 30° 实测 29.81°），但**停滞/脱困的判据仍只在 0.5 m 上验过**，长命令上的窗口表现没量；
- Tour 磨墙产生假墙（重定位假告警的源头），需在规划侧抑制；
- 全局观已经通过 `MapInfo` 递给大模型（未测格数、最近前沿距离与方位、12 扇区未测直方图、
  吻合度与判定、漂移花费）；**语义记忆层已交付**（`Remember`/`Forget`，词与格子同框、清空地图即作废，
  `MapInfo` 结尾按远近报出地点并说清"共 N 个 / 只列最近 4 个"）；**外部调度也已交付**（§3.6：`car_cron.py`
  到点把一句话发给会话，不在 bridge 里造 scheduler）。还缺的只有两件事：
  **① 一班真的定时动作从没发过** —— 随包两条都是 `enabled: false`，第一次请挑只读那条、人在旁边看；
  **② 长程目标循环**（把"还剩哪儿没看"变成它自己下一轮的目标，而不是等人设闹钟）

**这一轮收掉的旧账**（留个名字，免得下次又当未知去查）：人工急停与机器自停已分道；
"直行命令下底盘自转 34~43°"的根因是控制台每秒插进 `Move` 的**零速点刹**，修完 0.5 m 的朝向峰差
只剩 0.7-1.1°；小角度转弯的固定滑行改成按释放速率提前松手（30° 的过冲 +3.7° → +1.6°）；
45° 转不到头是收尾律落进底盘死区，`TURN_MIN` 抬到 0.20 rad/s 解决。

安装与使用见 **[用户操作手册.md](用户操作手册.md)**；全过程与实测数据见 **[开发过程实录.md](开发过程实录.md)**。
