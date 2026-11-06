cmake_minimum_required(VERSION 3.10)
project(hexapod_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/gait_odom_tf.py DESTINATION lib/${PROJECT_NAME} RENAME gait_odom_tf)
install(PROGRAMS scripts/camera_bridge.py DESTINATION lib/${PROJECT_NAME} RENAME camera_bridge)
install(PROGRAMS scripts/camera_vision.py DESTINATION lib/${PROJECT_NAME} RENAME camera_vision)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gait_odom_tf test/test_gait_odom_tf.py
    ENV HEXA_ODOM_TF=${CMAKE_CURRENT_SOURCE_DIR}/scripts/gait_odom_tf.py)
  ament_add_pytest_test(test_camera_bridge test/test_camera_bridge.py
    ENV HEXA_CAMERA_BRIDGE=${CMAKE_CURRENT_SOURCE_DIR}/scripts/camera_bridge.py)
  ament_add_pytest_test(test_camera_vision test/test_camera_vision.py)
endif()

ament_package()
