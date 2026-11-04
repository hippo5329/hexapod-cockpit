cmake_minimum_required(VERSION 3.10)
project(hexapod_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/gait_odom_tf.py DESTINATION lib/${PROJECT_NAME} RENAME gait_odom_tf)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gait_odom_tf test/test_gait_odom_tf.py
    ENV HEXA_ODOM_TF=${CMAKE_CURRENT_SOURCE_DIR}/scripts/gait_odom_tf.py)
endif()

ament_package()
