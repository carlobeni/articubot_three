from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(package="articubot_three", executable="sensors_fast_listener_node.py"),
        Node(package="articubot_three", executable="sensors_reliable_listener_node.py"),

        Node(package="articubot_three", executable="imu_cache_node.py"),
        Node(package="articubot_three", executable="gps_cache_node.py"),

        Node(package="articubot_three", executable="cmd_publisher_node.py"),

        Node(package="articubot_three", executable="monitor_node.py"),
    ])
