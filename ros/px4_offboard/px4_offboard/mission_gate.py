"""Keyboard/topic mission gate. Publishes Bool on start_mission and end_mission.

ENTER or "start" begins the mission; "terminate" commands landing. Topics are
relative, so launch remapping/namespacing decides who hears them.
"""
from __future__ import annotations

import select
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class MissionGate(Node):
    def __init__(self):
        super().__init__("mission_gate")
        self._start_pub = self.create_publisher(Bool, "start_mission", 10)
        self._end_pub = self.create_publisher(Bool, "end_mission", 10)
        self._started = False
        self._ended = False
        print("Mission gate ready. ENTER/'start' to begin, 'terminate' to land.")

    def trigger_start(self, source: str):
        if self._started:
            return
        self._started = True
        self.get_logger().info(f"start triggered by {source}.")
        self._start_pub.publish(Bool(data=True))

    def trigger_end(self, source: str):
        if self._ended:
            return
        self._ended = True
        self.get_logger().info(f"end triggered by {source}.")
        self._end_pub.publish(Bool(data=True))

    @property
    def ended(self) -> bool:
        return self._ended


def main(args=None):
    rclpy.init(args=args)
    node = MissionGate()
    try:
        while rclpy.ok() and not node.ended:
            if select.select([sys.stdin], [], [], 0.2)[0]:
                command = sys.stdin.readline().rstrip("\n").strip().lower()
                if command in ("", "start", "start mission"):
                    node.trigger_start("keyboard")
                elif command in ("terminate", "terminate mission", "land"):
                    node.trigger_end("keyboard")
                else:
                    print("Unknown command. ENTER/start to begin, terminate to land.")
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
