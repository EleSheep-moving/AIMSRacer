cmake_minimum_required(VERSION 3.16)
project(aims_racer_system)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(rear_axle_imu STATIC src/rear_axle_imu.cpp)
target_compile_features(rear_axle_imu PUBLIC cxx_std_17)
target_include_directories(rear_axle_imu PUBLIC include)
target_link_libraries(rear_axle_imu PUBLIC Eigen3::Eigen)
target_compile_options(rear_axle_imu PRIVATE -Wall -Wextra -Wpedantic)

add_executable(imu_to_rear_axle src/imu_to_rear_axle.cpp)
target_link_libraries(imu_to_rear_axle rear_axle_imu)
ament_target_dependencies(imu_to_rear_axle rclcpp sensor_msgs nav_msgs)
target_compile_options(imu_to_rear_axle PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS imu_to_rear_axle DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
file(GLOB system_scripts CONFIGURE_DEPENDS scripts/*.py)
install(PROGRAMS ${system_scripts} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params behaviour_trees rviz
  DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(rear_axle_contract tests/test_rear_axle_contract.py
    ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs")
  ament_add_pytest_test(nav_map_argument tests/test_nav_map_argument.py)
endif()

ament_package()
