"""Windows-side CLI for reconfiguring the MicroROS-V2 control board over its
Type-C USB-UART link, using the vendor's MicroROS_Robot protocol class.

    python car_config.py dump
    python car_config.py apply --ssid Xiaomi_23F3 --passwd ****** \
                               --agent-ip 192.168.31.210 --domain-id 40
    python car_config.py scale --show
    python car_config.py scale --linear 1.22 --angular 1.15
    python car_config.py factory

`config_robot.py` next to this file is the vendor's own protocol class, byte-identical
(md5 84becf59f8db8a57e922685c45a0407d) to `mircoros_v2/.system_cfg/robot_manager/
config_robot.py` inside 22.程序源码汇总/ros功能源码/mircoros_v2-1.0.1.zip. Do not "fix" it.
"""
import argparse
import time

import serial
from serial.tools import list_ports

from config_robot import MicroROS_Robot


def find_port(prefer=None):
    if prefer:
        return prefer
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("未发现任何串口。请确认已用【数据线】连接车控板 Type-C 口，"
                         "并在设备管理器里能看到 COM 口。")
    ranked = sorted(ports, key=lambda p: (
        0 if any(k in (p.description or "").lower()
                 for k in ("esp32", "usb-serial", "usb serial", "ch34", "cp210", "ftdi")) else 1,
        p.device))
    if len(ports) > 1:
        print("候选串口: " + ", ".join("%s(%s)" % (p.device, p.description) for p in ranked))
    return ranked[0].device


def open_robot(args):
    port = find_port(args.port)
    print("串口: %s @115200" % port)
    robot = MicroROS_Robot(port=port, debug=args.verbose)
    time.sleep(0.5)
    robot._MicroROS_Robot__ser.flushInput()
    robot._MicroROS_Robot__ser.flushOutput()
    if not robot.auto_enter_config_window(ready_timeout=12.0, interval=0.2):
        raise SystemExit("ERROR: 固件配置窗口无响应。检查串口/线缆，或手动按车上 RST 后重试。")
    print("配置窗口已就绪，固件版本: %s" % robot.read_version())
    return robot


def cmd_dump(args):
    robot = open_robot(args)
    try:
        robot.print_all_firmware_parm()
        # the vendor's own dump omits the velocity calibration, and it is the one field
        # whose value decides whether /odom_raw can be trusted
        print("velocity_scale:", robot.read_velocity_scale())
    finally:
        robot._MicroROS_Robot__send_payload(0x22)


def cmd_scale(args):
    """Read or write the firmware's own speed calibration (command 0x0A).

    This multiplies what the board reports on /odom_raw -- it is NOT the response gain the
    PC-side planner fits. Ours measures "commanded -> achieved" (0.82 of a command gets
    delivered); the firmware scale corrects "reported -> real", worked out with a tape measure
    over a known distance. Write the wrong one and odom gets worse, not better.
    """
    robot = open_robot(args)
    try:
        cur = robot.read_velocity_scale()
        if not cur:
            raise SystemExit("ERROR: 固件没有回答 0x0A 读取，未做任何修改。"
                             "（可能固件版本过旧，或串口被其它程序占用）")
        print("当前固件速度标定: linear=%s angular=%s" % (cur[0], cur[1]))
        if args.show or (args.linear is None and args.angular is None):
            print("出厂值参考: linear=1.26 angular=1.0708（厂家表格的左右轮是 1.0708/1.0863，"
                  "但协议只有一个角速度字段，写进去的是同一个值）")
            return
        lin = args.linear if args.linear is not None else cur[0]
        ang = args.angular if args.angular is not None else cur[1]
        print("写入 VELOCITY_SCALE(0x0A): linear=%.4f angular=%.4f ..." % (lin, ang))
        got = robot.set_velocity_scale(lin, ang)
        print("回读校验通过: %r" % (got,))
        print("注意：odom 尺度已变，PC 侧 nav_gains.json 的响应增益与既有地图都建立在旧 odom 上，"
              "写完请重跑 RateTest/拟合，并重新建图或重定位确认。")
    finally:
        robot._MicroROS_Robot__send_payload(0x22)


def cmd_apply(args):
    if not args.ssid:
        raise SystemExit("apply 需要 --ssid")
    robot = open_robot(args)
    agent_ip = [int(x) for x in args.agent_ip.split(".")]
    if len(agent_ip) != 4:
        raise SystemExit("--agent-ip 必须是四段 IPv4")
    try:
        print("写入主控 WiFi ...")
        robot.set_wifi_config(args.ssid, args.passwd)
        print("写入摄像头 WiFi（与主控同一网络）...")
        robot.set_camera_wifi_config(args.ssid, args.passwd)
        print("写入 micro-ROS Agent 地址 %s:%d ..." % (args.agent_ip, args.agent_port))
        robot.set_udp_config(agent_ip, args.agent_port)
        robot.set_ros_domain_id(args.domain_id)
        robot.set_ros_namespace(args.namespace or "")
        robot.set_pwm_servo_offset(1, args.servo1)
        robot.set_pwm_servo_offset(2, args.servo2)
        robot.set_motor_pid_parm(*args.motor_pid)
        robot.set_imu_yaw_pid_parm(*args.imu_pid)

        robot.set_camera_auto_config(not args.no_camera_auto)
        time.sleep(0.3)
        expect = 1 if not args.no_camera_auto else 0
        got = robot.read_camera_auto_config()
        if got != expect:
            raise RuntimeError("camera auto-config 校验失败: expected=%d actual=%r" % (expect, got))

        if not args.skip_camera:
            print("立即下发摄像头 WiFi ...")
            robot.apply_camera_wifi_config_now()
            time.sleep(2)
            camera_ip = None
            for _ in range(5):
                camera_ip = robot.query_camera_ip_now(wait_timeout=4.0)
                if camera_ip:
                    break
                time.sleep(2)
            print("摄像头 IP: %s" % (camera_ip or "查询失败（稍后可再 dump）"))

        robot.print_all_firmware_parm()
        print("配置完成，参数立即生效。")
        robot._MicroROS_Robot__send_payload(0x22)
    except Exception:
        robot._MicroROS_Robot__send_payload(0x22)
        raise


def cmd_reboot(args):
    """用 USB 串口上的自动复位电路重启控板（DTR=EN, RTS=GPIO0）。"""
    port = find_port(args.port)
    print("串口: %s，发送复位脉冲 ..." % port)
    ser = serial.Serial(port, 115200)
    ser.setDTR(True)
    ser.setRTS(True)
    time.sleep(0.15)
    ser.setDTR(False)
    ser.setRTS(False)
    ser.close()
    print("已复位，控板正在重启（约 5~10 秒后重新连 WiFi 并向 agent 发起会话）")


def cmd_factory(args):
    robot = open_robot(args)
    robot.reset_config()
    print("已恢复出厂配置，等待重启 ...")
    time.sleep(2.5)
    robot._MicroROS_Robot__ser.flushInput()
    robot._MicroROS_Robot__ser.flushOutput()
    if not robot.auto_enter_config_window(ready_timeout=12.0, interval=0.2):
        raise SystemExit("ERROR: 重启后配置窗口无响应。")
    robot.print_all_firmware_parm()
    robot._MicroROS_Robot__send_payload(0x22)


def main():
    p = argparse.ArgumentParser(description="MicroROS-V2 车载配置（Type-C 串口）")
    p.add_argument("--port", help="COM 口，如 COM5；默认自动探测")
    p.add_argument("--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("dump", help="只读取当前固件参数，不修改")
    d.set_defaults(func=cmd_dump)

    a = sub.add_parser("apply", help="写入 WiFi / Agent IP / DOMAIN_ID")
    a.add_argument("--ssid", required=True)
    a.add_argument("--passwd", default="")
    a.add_argument("--agent-ip", required=True, help="笔记本在车上局域网里的 IP")
    a.add_argument("--agent-port", type=int, default=8090)
    a.add_argument("--domain-id", type=int, default=40)
    a.add_argument("--namespace", default="")
    a.add_argument("--servo1", type=int, default=-5)
    a.add_argument("--servo2", type=int, default=0)
    a.add_argument("--motor-pid", type=float, nargs=3, default=[1.8, 0.12, 0.0])
    a.add_argument("--imu-pid", type=float, nargs=3, default=[2.4, 0.0, 0.16])
    a.add_argument("--no-camera-auto", action="store_true")
    a.add_argument("--skip-camera", action="store_true", help="不配置图传摄像头")
    a.set_defaults(func=cmd_apply)

    r = sub.add_parser("reboot", help="通过串口复位控板（不改参数）")
    r.set_defaults(func=cmd_reboot)

    s = sub.add_parser("scale", help="读/写固件速度标定 0x0A（改的是 odom 尺度，不是 PC 侧响应增益）")
    s.add_argument("--linear", type=float, help="线速度尺度，0.1~5.0；不给就保持当前值")
    s.add_argument("--angular", type=float, help="角速度尺度，0.1~5.0；协议只有一个字段，左右共用")
    s.add_argument("--show", action="store_true", help="只读不写")
    s.set_defaults(func=cmd_scale)

    f = sub.add_parser("factory", help="恢复出厂默认（擦除 NVS）")
    f.set_defaults(func=cmd_factory)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
