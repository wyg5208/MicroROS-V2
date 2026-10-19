"""PC 侧联调自检：图传可达 + ROS 话题是否在线 + Agent 端口和防火墙。用法：自检.bat"""
import os
import re
import subprocess
import sys
import time

from car_mcp_server import grab_jpeg

RULE_NAME = "microros-agent-udp-8090"


def _run(cmd):
    try:
        p = subprocess.run(cmd, capture_output=True, timeout=20, shell=True)
    except (OSError, subprocess.SubprocessError):
        return ""
    return (p.stdout or b"").decode("utf-8", "ignore") + (p.stderr or b"").decode("utf-8", "ignore")


def port_check():
    """Is anything listening on udp/8090 -- the one step the car side cannot see."""
    # UDP has no connection state, so netstat leaves the State column blank -- match the row
    listening = bool(re.search(r"^\s*UDP\s+\S*:8090\s", _run("netstat -ano -p udp"), re.M))
    print(("[OK ] Agent 在听 udp/8090" if listening else "[FAIL] 没有进程监听 udp/8090"
           "（先跑 启动Agent.bat）"))
    return listening


def firewall_check(received):
    """Whether Windows Firewall lets the board reach the Agent, read together with whether
    anything actually arrived. A fresh machine blocks unsolicited inbound silently -- the Agent
    runs, the car associates, and not one topic shows -- which is why vendor doc 4 ends its
    five-step list with this. But a missing rule on a machine whose network profile already
    allows it is not a fault, so the verdict has to be anchored to the evidence."""
    rules = _run("netsh advfirewall firewall show rule name=all")
    if RULE_NAME in rules:
        print("[OK ] 防火墙已放行 UDP 8090（规则 %s）" % RULE_NAME)
    elif "8090" in rules:
        print("[-- ] 有涉及 8090 的防火墙规则，但没找到名为 %s 的那条，"
              "无法判断是放行还是阻止；如需重来请以管理员身份运行 agent_firewall.bat" % RULE_NAME)
    elif received:
        print("[OK ] 没有 UDP 8090 的入站规则，但车的数据进得来——当前网络档位不拦"
              "（多半是“专用网络”）。换 WiFi 或改成公用档位后可能失效，届时以管理员身份"
              "运行 agent_firewall.bat")
    else:
        print("[WARN] 没有 UDP 8090 的入站规则，且话题一个都没进来。默认入站策略若是“阻止”，"
              "这就是原因：请以管理员身份运行 agent_firewall.bat，然后重跑本自检")


def main():
    camera_ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.31.150"
    domain = sys.argv[2] if len(sys.argv) > 2 else "40"
    # must be set before rclpy creates a participant, or we look at domain 0 and see nothing
    os.environ["ROS_DOMAIN_ID"] = domain

    link_ok = port_check()

    data = grab_jpeg(camera_ip, 81, 6.0)
    if data:
        path = f"cache/selfcheck_{int(time.time())}.jpg"
        with open(path, "wb") as f:
            f.write(data)
        print(f"[OK ] 图传 {camera_ip}:81 取到一帧 {len(data)} bytes -> {path}")
    else:
        print(f"[FAIL] 图传 {camera_ip}:81 取帧失败（车通电？摄像头连上同一 WiFi？IP 变了？)")

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    rclpy.init()
    node = Node("pc_selfcheck")
    time.sleep(1.5)
    names = {t[0] for t in node.get_topic_names_and_types()}
    # micro-ROS endpoints are BEST_EFFORT; a RELIABLE subscription matches nothing
    best = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                      reliability=ReliabilityPolicy.BEST_EFFORT)
    got = {}
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    node.create_subscription(Odometry, "/odom_raw",
                             lambda m: got.setdefault("odom_raw", 0), best)
    node.create_subscription(LaserScan, "/scan",
                             lambda m: got.setdefault("scan", 0), best)
    end = time.time() + 6.0
    while time.time() < end and len(got) < 2:
        rclpy.spin_once(node, timeout_sec=0.2)
    for t in ["/cmd_vel", "/odom_raw", "/scan", "/imu"]:
        print(("[OK ] " if t in names else "[-- ] ") + t)
    firewall_check(bool(got))
    print("数据回传: %s" % (sorted(got) or "无"))
    if not got:
        print("排查顺序（厂商 4.快速启动 的五步清单，PC 侧能自己确认的打头）："
              "① Agent 是否在听 8090 → %s；② 防火墙是否放行入站 → 看上面防火墙那行；"
              "③ 车上 agent-ip/DOMAIN_ID 是否指向本机 → 跑 car_config.py dump；"
              "④ 车是否连上了和笔记本同一个 WiFi → 见上面图传那行"
              % ("在听" if link_ok else "没在听"))
    print("ROS_DOMAIN_ID=%s 当前可见话题: %s" % (domain, sorted(names) or "无"))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
