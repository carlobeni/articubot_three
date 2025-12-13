#!/usr/bin/env python3
# imu_cache_node.py

import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import articubot_three.hw_config as cfg

class ImuCache(Node):
    def __init__(self):
        super().__init__("imu_cache")

        self.last_time = 0.0
        self.valid = False

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_acc = self.create_subscription(
            Vector3Stamped, cfg.TOPIC_IMU_ACCEL, self.cb_acc, sub_qos)

        self.pub_acc = self.create_publisher(
            Vector3Stamped, cfg.LOCAL_IMU_ACCEL, pub_qos)

    def cb_acc(self, msg):
        self.last_time = time.time()
        self.valid = True
        self.pub_acc.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ImuCache())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
