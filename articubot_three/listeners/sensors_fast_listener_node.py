#!/usr/bin/env python3
# sensors_fast_listener_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Range, CompressedImage
from std_msgs.msg import Int32MultiArray, Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import articubot_three.hw_config as cfg

class SensorsFastListener(Node):
    def __init__(self):
        super().__init__("sensors_fast_listener")
        cfg.check_domain_id(self.get_logger())

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Vector3Stamped, cfg.TOPIC_IMU_ACCEL, self.cb, qos)
        self.create_subscription(Vector3Stamped, cfg.TOPIC_IMU_MAG, self.cb, qos)
        self.create_subscription(Float64, cfg.TOPIC_IMU_COMPASS, self.cb, qos)
        self.create_subscription(Range, cfg.TOPIC_ULTRASONIC, self.cb, qos)
        self.create_subscription(Int32MultiArray, cfg.TOPIC_IR, self.cb, qos)
        self.create_subscription(CompressedImage, cfg.TOPIC_CAMERA, self.cb, qos)

    def cb(self, msg):
        pass  # Solo escucha (útil para debug DDS)

def main():
    rclpy.init()
    rclpy.spin(SensorsFastListener())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
