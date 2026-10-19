"""逐个调用 MCP 工具做联调（不需要大模型）。用法：python mcp_probe.py [SeeWhat GetStatus ...]"""
import sys

from agent import McpClient

DEFAULT = ["GetStatus", "SeeWhat", "AdjustCameraView", "Stop"]


def main():
    url = "http://127.0.0.1:8000/mcp"
    names = sys.argv[1:] or DEFAULT
    args = {"AdjustCameraView": {"pitch": 5, "yaw": 0}}
    client = McpClient(url)
    client.connect()
    print("已连接 %s，%d 个工具" % (client.server_name, len(client.list_tools())))
    for name in names:
        res = client.call_tool(name, args.get(name, {}))
        print("%-18s -> %s" % (name, str(res)[:220]))


if __name__ == "__main__":
    main()
