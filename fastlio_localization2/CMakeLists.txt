cmake_minimum_required(VERSION 3.8)
project(fast_lio_localization)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  fast_lio_localization/global_localization.py
  fast_lio_localization/publish_initial_pose.py
  fast_lio_localization/transform_fusion.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
