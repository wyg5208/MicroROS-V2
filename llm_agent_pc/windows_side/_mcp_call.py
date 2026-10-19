# -*- coding: utf-8 -*-
"""Call one MCP tool on the running bridge: _mcp_call.py <Tool> ['json-args']
(or `_mcp_call.py -list` to print the tools the live bridge actually exposes).

Exists because the interesting state (the growing map, the mid-motion verdict) is only
reachable through the bridge's own :8000 endpoint while the car is actually running -- an
offline test can replay residuals, but it cannot show what MapInfo answers mid-tour.
"""
import json
import sys
import urllib.request

URL = "http://127.0.0.1:8000/mcp"


def rpc(payload, sid=None):
    headers = {"Content-Type": "application/json",
               "Accept": "application/json, text/event-stream"}
    if sid:
        headers["mcp-session-id"] = sid
    req = urllib.request.Request(URL, data=json.dumps(payload).encode("utf-8"), headers=headers)
    with urllib.request.urlopen(req, timeout=30) as r:
        return r.read().decode("utf-8", "replace"), r.headers.get("mcp-session-id")


def _body(text):
    # streamable-http may answer with an SSE frame or bare JSON depending on the request
    for line in text.splitlines():
        if line.startswith("data:"):
            return json.loads(line[5:].strip())
    return json.loads(text) if text.strip() else {}


def main():
    name = sys.argv[1]
    args = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    _t, sid = rpc({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                   "params": {"protocolVersion": "2024-11-05", "capabilities": {},
                              "clientInfo": {"name": "probe", "version": "1"}}})
    if name == "-list":
        out, _ = rpc({"jsonrpc": "2.0", "id": 2, "method": "tools/list", "params": {}}, sid)
        for t in _body(out)["result"]["tools"]:
            print("%-12s %s" % (t["name"], t["description"][:110].replace("\n", " ")))
        return
    out, _ = rpc({"jsonrpc": "2.0", "id": 2, "method": "tools/call",
                  "params": {"name": name, "arguments": args}}, sid)
    res = _body(out).get("result", _body(out))
    for part in res.get("content", []):
        print(part.get("text", ""))


if __name__ == "__main__":
    main()
