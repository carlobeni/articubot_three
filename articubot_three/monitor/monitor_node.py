#!/usr/bin/env python3
# monitor_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import NavSatFix
import articubot_three.hw_config as cfg

class Monitor(Node):
    def __init__(self):
        super().__init__("monitor")

        self.state = {}

        self.create_subscription(
            Vector3Stamped, cfg.LOCAL_IMU_ACCEL,
            lambda m: self.store(cfg.LOCAL_IMU_ACCEL, m), 10)

        self.create_subscription(
            NavSatFix, cfg.LOCAL_GPS,
            lambda m: self.store(cfg.LOCAL_GPS, m), 10)

    def store(self, topic, msg):
        self.state[topic] = msg

def main():
    rclpy.init()
    rclpy.spin(Monitor())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
