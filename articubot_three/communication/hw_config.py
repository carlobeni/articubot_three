import os

# ================= ROS DOMAIN =================
ROS_DOMAIN_ID = 42

def check_domain_id(logger=None):
    env = os.getenv("ROS_DOMAIN_ID")
    msg = (
        f"ROS_DOMAIN_ID={env} (expected {ROS_DOMAIN_ID})"
        if env else
        f"ROS_DOMAIN_ID NOT set (recommended {ROS_DOMAIN_ID})"
    )
    if logger:
        logger.warning(msg)
    else:
        print(msg)

# ================= TOPICS (FROM PI5) =================
TOPIC_IR = "/sensor/IR_measure"
TOPIC_ULTRASONIC = "/sensor/ultrasonic_read"

TOPIC_IMU_ACCEL = "/sensor/imu/accel"
TOPIC_IMU_MAG = "/sensor/imu/mag"
TOPIC_IMU_COMPASS = "/sensor/imu/compass"

TOPIC_GPS = "/sensor/gps/fix"
TOPIC_CAMERA = "/camera/image_raw/compressed"

# ================= COMMAND =================
TOPIC_CMD_VEL_ROBOT = "/cmd_vel_robot"
TOPIC_CMD_SERIAL = "/cmd_serial"

