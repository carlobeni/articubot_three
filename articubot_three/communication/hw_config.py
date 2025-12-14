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
# READY TO USE
TOPIC_IMU_GIR_ACC = "pi/sensor/imu_data"
TOPIC_IMU_MAG = "pi/sensor/imu_mag"
TOPIC_GPS = "pi/sensor/gps_fix"
TOPIC_CAMERA = "pi/camera/image_raw"
# used only internally
TOPIC_MONITOR = "pi/system/monitor"  
TOPIC_CMD_SERIAL_MEGA = "/cmd_serial"

# IN DEVELOPMENT
TOPIC_IR = "pi/sensor/IR_measure"
TOPIC_ULTRASONIC = "pi/sensor/ultrasonic_read"


# ================= COMMAND =================
TOPIC_CMD_VEL_ROBOT = "/cmd_vel_robot"
TOPIC_CMD_SERIAL = "/cmd_serial"

