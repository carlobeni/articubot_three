cmake_minimum_required(VERSION 3.5)
project(articubot_three)

# =====================================================
# Standards
# =====================================================
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================================================
# Dependencies (Python ROS 2)
# =====================================================
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =====================================================
# Install launch files
# =====================================================
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# =====================================================
# Install Python nodes (EXACT FILENAMES)
# =====================================================
install(PROGRAMS
  # -------- listeners --------
  articubot_three/listeners/sensors_fast_listener_node.py
  articubot_three/listeners/sensors_reliable_listener_node.py

  # -------- cache --------
  articubot_three/cache/imu_cache_node.py
  articubot_three/cache/gps_cache_node.py

  # -------- command --------
  articubot_three/command/cmd_publisher_node.py

  # -------- monitor --------
  articubot_three/monitor/monitor_node.py

  DESTINATION lib/${PROJECT_NAME}
)

# =====================================================
# Testing
# =====================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
