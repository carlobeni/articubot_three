#!/usr/bin/env python3
# monitor_node.py  (CONTROL DEVICE)

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import Range, NavSatFix, CompressedImage
from geometry_msgs.msg import Vector3Stamped

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

import hw_config as cfg


class MonitorNode(Node):

    def __init__(self):
        super().__init__("monitor_node")
        cfg.check_domain_id(self.get_logger())

        # =====================================================
        # QoS PROFILES
        # =====================================================
        self.qos_sensors = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.qos_commands = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # =====================================================
        # FLAGS
        # =====================================================
        self.declare_parameter("monitor_cmd_serial", True)
        self.declare_parameter("monitor_ir", True)
        self.declare_parameter("monitor_ultrasonic", True)
        self.declare_parameter("monitor_gps", True)
        self.declare_parameter("monitor_imu_accel", True)
        self.declare_parameter("monitor_imu_mag", True)
        self.declare_parameter("monitor_imu_compass", True)
        self.declare_parameter("monitor_camera", True)

        # =====================================================
        # DATA STORAGE
        # data[topic] = (timestamp_or_id, value)
        # =====================================================
        self.data = {}

        # ================= Sensors =================
        self._sub(Int32MultiArray, cfg.TOPIC_IR, self.qos_sensors)
        self._sub(Range, cfg.TOPIC_ULTRASONIC, self.qos_sensors)
        self._sub(NavSatFix, cfg.TOPIC_GPS, self.qos_sensors)
        self._sub(Vector3Stamped, cfg.TOPIC_IMU_ACCEL, self.qos_sensors)
        self._sub(Vector3Stamped, cfg.TOPIC_IMU_MAG, self.qos_sensors)
        self._sub(Vector3Stamped, cfg.TOPIC_IMU_COMPASS, self.qos_sensors)
        self._sub(CompressedImage, cfg.TOPIC_CAMERA, self.qos_sensors)

        # ================= Commands =================
        self._sub(
            String,
            cfg.TOPIC_CMD_SERIAL,
            self.qos_commands,
            is_command=True
        )

        self.timer = self.create_timer(0.2, self.print_table)

    # -----------------------------------------------------

    def _sub(self, msg_type, topic, qos, is_command=False):
        self.data[topic] = ("-", "---")

        def cb(msg):
            if is_command:
                # ===== COMMAND: keep ID =====
                if ":" in msg.data:
                    msg_id, value = msg.data.split(":", 1)
                    self.data[topic] = (msg_id.strip(), value.strip())
                else:
                    self.data[topic] = ("?", msg.data)
            else:
                # ===== SENSOR: use timestamp =====
                if hasattr(msg, "header"):
                    stamp = msg.header.stamp
                    timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"
                else:
                    timestamp = "N/A"

                value = self._format(msg, topic)
                self.data[topic] = (timestamp, value)

        self.create_subscription(msg_type, topic, cb, qos)

    # -----------------------------------------------------

    def _format(self, msg, topic):
        if isinstance(msg, Int32MultiArray):
            return str(msg.data)

        if isinstance(msg, Range):
            return f"{msg.range:.3f} m"

        if isinstance(msg, NavSatFix):
            return f"{msg.latitude:.6f}, {msg.longitude:.6f}"

        if isinstance(msg, Vector3Stamped):
            if topic == cfg.TOPIC_IMU_COMPASS:
                return f"heading={msg.vector.x:.2f}°"
            return (
                f"x={msg.vector.x:.2f}, "
                f"y={msg.vector.y:.2f}, "
                f"z={msg.vector.z:.2f}"
            )

        if isinstance(msg, CompressedImage):
            return "frame"

        return "?"

    # -----------------------------------------------------

    def print_table(self):
        print("\033[2J\033[H", end="")
        print("=== MONITOR (CONTROL DEVICE) ===")
        print(f"ROS_DOMAIN_ID = {cfg.ROS_DOMAIN_ID}\n")

        # -------- SENSORS --------
        print("---- SUBSCRIBERS (SENSORS) ----")
        print("{:<35} | {:<22} | {:<40}".format("TOPIC", "TIMESTAMP", "VALUE"))
        print("-" * 95)

        for topic in self.data:
            if topic == cfg.TOPIC_CMD_SERIAL:
                continue
            ts, value = self.data[topic]
            print("{:<35} | {:<22} | {:<40}".format(topic, ts, value))

        # -------- COMMANDS --------
        print("\n---- SUBSCRIBERS (COMMANDS) ----")
        print("{:<35} | {:<22} | {:<40}".format("TOPIC", "ID", "VALUE"))
        print("-" * 95)

        if cfg.TOPIC_CMD_SERIAL in self.data:
            msg_id, value = self.data[cfg.TOPIC_CMD_SERIAL]
            print("{:<35} | {:<22} | {:<40}".format(
                cfg.TOPIC_CMD_SERIAL, msg_id, value
            ))

        print("\nCtrl+C to exit")


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
