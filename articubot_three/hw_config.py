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
        logger.warn(msg)
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

# ================= LOCAL (CACHED) =================
LOCAL_PREFIX = "/local"

LOCAL_IR = f"{LOCAL_PREFIX}/ir"
LOCAL_ULTRASONIC = f"{LOCAL_PREFIX}/ultrasonic"

LOCAL_IMU_ACCEL = f"{LOCAL_PREFIX}/imu/accel"
LOCAL_IMU_MAG = f"{LOCAL_PREFIX}/imu/mag"
LOCAL_IMU_COMPASS = f"{LOCAL_PREFIX}/imu/compass"

LOCAL_GPS = f"{LOCAL_PREFIX}/gps"

# ================= COMMAND =================
TOPIC_CMD_SERIAL = "/cmd_serial"
LOCAL_CMD_SERIAL = f"{LOCAL_PREFIX}/cmd_serial"
