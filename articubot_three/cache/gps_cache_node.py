#!/usr/bin/env python3
# gps_cache_node.py

import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import articubot_three.hw_config as cfg

class GPSCache(Node):
    def __init__(self):
        super().__init__("gps_cache")
        self.last = None

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub = self.create_subscription(
            NavSatFix, cfg.TOPIC_GPS, self.cb, sub_qos)

        self.pub = self.create_publisher(
            NavSatFix, cfg.LOCAL_GPS, pub_qos)

    def cb(self, msg):
        self.last = msg
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(GPSCache())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
