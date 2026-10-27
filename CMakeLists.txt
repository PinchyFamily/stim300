cmake_minimum_required(VERSION 3.8)
project(stim300)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(driver_stim300
  src/driver_stim300.cpp
  src/serial_unix.cpp
  src/datagram_parser.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sam_msgs REQUIRED)
find_package(std_srvs REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Executable for the node
add_executable(stim300_driver src/stim300_driver_node.cpp)
target_link_libraries(stim300_driver driver_stim300)
ament_target_dependencies(stim300_driver
  rclcpp 
  sensor_msgs
  sam_msgs
  std_srvs
)

install(TARGETS
  # driver_stim300
  stim300_driver
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(check_driver_stim300
    test/check_driver_stim300.cpp
    test/mock_serial_driver.h
  )
  if(TARGET check_driver_stim300)
    target_link_libraries(check_driver_stim300 driver_stim300)
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  ament_add_gmock(check_configure_sensor
    test/check_configure_sensor.cpp
    test/mock_serial_driver.h
  )
  if(TARGET check_configure_sensor)
    target_link_libraries(check_configure_sensor driver_stim300)
  endif()
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(check_datasheet_constanst
    test/check_stim300_constants.cpp
  ) 
  if(TARGET check_datasheet_constanst)
    target_link_libraries(check_datasheet_constanst driver_stim300)
  endif()
endif()

ament_python_install_package(stim300_tools)
install(PROGRAMS
  scripts/imu_stats
  scripts/static_noise
  scripts/allan_variance
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_stim300_tools test/test_stim300_tools.py)
endif()

ament_package()
