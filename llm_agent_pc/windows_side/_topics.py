"""Dump the topic graph the board publishes, so the heading sources can be checked."""
import time
import rclpy
from rclpy.node import Node

rclpy.init()
n = Node("topic_dump")
seen = {}
for _ in range(8):
    for nm, ty in n.get_topic_names_and_types():
        seen[nm] = ",".join(ty)
    time.sleep(0.5)
for nm in sorted(seen):
    print("%-28s %s" % (nm, seen[nm]))
