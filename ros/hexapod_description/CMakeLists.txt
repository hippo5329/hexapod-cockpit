cmake_minimum_required(VERSION 3.10)
project(hexapod_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf rviz launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The FK test compiles the firmware's hexa_kin; point it at the repo's firmware/lib.
  ament_add_pytest_test(test_urdf_fk test/test_urdf_fk.py
    ENV HEXA_FIRMWARE_LIB=${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/lib
        HEXA_XACRO=${CMAKE_CURRENT_SOURCE_DIR}/urdf/minihexa.urdf.xacro)
endif()

ament_package()
