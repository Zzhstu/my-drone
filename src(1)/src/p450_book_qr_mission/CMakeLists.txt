cmake_minimum_required(VERSION 3.0.2)
project(p450_book_qr_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_srvs
  cv_bridge
  prometheus_msgs
  spirecv_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/book_qr_patrol_mission.py
  scripts/pose_sampler.py
  scripts/qr_probe.py
  scripts/odom_to_path.py
  scripts/waypoints_to_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
