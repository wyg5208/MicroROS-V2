"""读取本机已保存的 WiFi 密码（netsh profile），写入小车固件。密码不落盘、不打印。"""
import re
import subprocess
import sys


def _key_content(ssid):
    out = subprocess.run(["netsh", "wlan", "show", "profile", f"name={ssid}", "key=clear"],
                         capture_output=True).stdout
    for enc in ("utf-8", "gbk", "cp437"):
        try:
            text = out.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    for line in text.splitlines():
        if re.match(r"^\s*(关键内容|Key Content)\s*[:：]", line):
            return line.split(":", 1)[-1].split("：", 1)[-1].strip()
    raise SystemExit("没能从 netsh 输出里取到密码，请手动用 --passwd 传入")


if __name__ == "__main__":
    ssid = sys.argv[1]
    passwd = _key_content(ssid)
    if not passwd:
        raise SystemExit(f"网络 {ssid} 无密码或取不到")
    print(f"已取到 {ssid} 的密码（{len(passwd)} 位），开始写入小车 ...")
    import car_config
    argv = ["--port", sys.argv[2] if len(sys.argv) > 2 else "COM3",
            "apply", "--ssid", ssid, "--passwd", passwd] + sys.argv[3:]
    sys.argv = ["car_config.py"] + argv
    car_config.main()
