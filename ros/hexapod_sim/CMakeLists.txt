cmake_minimum_required(VERSION 3.10)
project(hexapod_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# The robot model, the sensors, the AHRS and the gait are the firmware's own firmware/lib, compiled
# here as they are for the host simulator: build this package from a checkout of the whole
# repository.
set(HEXA_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../.. CACHE PATH "hexapod-cockpit checkout")
set(HEXA_LIB ${HEXA_ROOT}/firmware/lib)
add_library(hexa_world STATIC
  ${HEXA_LIB}/hexa_world/hexa_world.cpp
  ${HEXA_LIB}/hexa_sensors/hexa_sensors.cpp
  ${HEXA_LIB}/hexa_ahrs/hexa_ahrs.cpp
  ${HEXA_LIB}/hexa_kin/hexa_kin.cpp
  ${HEXA_LIB}/hexa_gait/hexa_gait.cpp
  ${HEXA_LIB}/hexa_servo/hexa_servo.cpp)
target_include_directories(hexa_world PUBLIC
  ${HEXA_LIB}/hexa_world ${HEXA_LIB}/hexa_sensors ${HEXA_LIB}/hexa_ahrs ${HEXA_LIB}/hexa_kin ${HEXA_LIB}/hexa_gait ${HEXA_LIB}/hexa_servo)
set_target_properties(hexa_world PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(hexapod_sim_node src/hexapod_sim_node.cpp)
target_link_libraries(hexapod_sim_node hexa_world rclcpp::rclcpp
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${visualization_msgs_TARGETS})
install(TARGETS hexapod_sim_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_node test/test_sim_node.py
    ENV HEXA_SIM_NODE=$<TARGET_FILE:hexapod_sim_node>
    TIMEOUT 120)
  ament_add_pytest_test(test_ekf test/test_ekf.py
    ENV HEXA_SIM_NODE=$<TARGET_FILE:hexapod_sim_node>
    TIMEOUT 120)
endif()

ament_package()
