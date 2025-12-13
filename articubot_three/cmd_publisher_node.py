#!/usr/bin/env python3
# cmd_publisher_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import articubot_three.hw_config as cfg

class CmdPublisher(Node):
    def __init__(self):
        super().__init__("cmd_publisher")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(String, cfg.TOPIC_CMD_SERIAL, qos)
        self.cmd_id = 0

    def send(self, cmd):
        self.cmd_id += 1
        msg = String(data=f"{self.cmd_id}:{cmd}")
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CmdPublisher()
    node.send("M 0.0 0.0")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
