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
find_package(tf2_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED COMPONENTS common io kdtree)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)

add_library(localization_health STATIC src/localization_health.cpp)
target_compile_features(localization_health PUBLIC cxx_std_17)
target_include_directories(localization_health PUBLIC include)
target_compile_options(localization_health PRIVATE -Wall -Wextra -Wpedantic)

add_library(localization_quality STATIC src/localization_quality.cpp)
target_compile_features(localization_quality PUBLIC cxx_std_17)
target_include_directories(localization_quality PUBLIC include)
target_include_directories(localization_quality SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(localization_quality PUBLIC Eigen3::Eigen PRIVATE ${PCL_LIBRARIES} OpenSSL::Crypto)
ament_target_dependencies(localization_quality PUBLIC sensor_msgs)
target_compile_options(localization_quality PRIVATE -Wall -Wextra -Wpedantic)

add_executable(localization_monitor src/localization_monitor.cpp)
target_link_libraries(localization_monitor localization_health localization_quality)
ament_target_dependencies(localization_monitor rclcpp nav_msgs sensor_msgs tf2_msgs diagnostic_msgs std_msgs)
target_compile_options(localization_monitor PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS localization_monitor DESTINATION lib/${PROJECT_NAME})

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
install(PROGRAMS
  scripts/activate_ndt.py scripts/relocalize_known_map.py
  scripts/lio_to_rear_axle.py scripts/livox_gyro_bias.py
  scripts/localization_policy.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  add_executable(tf_authority_audit ${CMAKE_CURRENT_SOURCE_DIR}/../../verification/localization/tf_authority_audit.cpp)
  target_compile_features(tf_authority_audit PRIVATE cxx_std_17)
  ament_target_dependencies(tf_authority_audit rclcpp tf2_msgs)
  target_compile_options(tf_authority_audit PRIVATE -Wall -Wextra -Wpedantic)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rear_axle_imu_test tests/test_rear_axle_imu.cpp)
  target_link_libraries(rear_axle_imu_test rear_axle_imu)
  target_compile_definitions(rear_axle_imu_test PRIVATE
    IMU_REFERENCE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/imu_python_reference.csv")
  add_executable(benchmark_rear_axle_imu tests/benchmark_rear_axle_imu.cpp)
  target_link_libraries(benchmark_rear_axle_imu rear_axle_imu)
  add_executable(rear_axle_force_test tests/test_rear_axle_force.cpp)
  target_link_libraries(rear_axle_force_test PRIVATE rear_axle_imu)
  target_compile_options(rear_axle_force_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME rear_axle_force COMMAND rear_axle_force_test)
  add_executable(localization_alignment_test tests/test_localization_alignment.cpp)
  target_link_libraries(localization_alignment_test PRIVATE localization_health)
  target_compile_options(localization_alignment_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME localization_alignment COMMAND localization_alignment_test)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(rear_axle_contract tests/test_rear_axle_contract.py
    ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs")
  ament_add_pytest_test(gyro_bias tests/test_gyro_bias.py)
  ament_add_pytest_test(current_launch tests/test_current_launch.py)
  ament_add_pytest_test(localization_policy tests/test_localization_policy.py)
  ament_add_pytest_test(initializer_lifecycle tests/test_initializer_lifecycle.py)
  ament_add_pytest_test(replay_sensor_qos tests/test_replay_sensor_qos.py)
  ament_add_pytest_test(replay_acceptance tests/test_replay_acceptance.py)
  ament_add_pytest_test(monitor_alignment_delivery tests/test_monitor_alignment_delivery.py
    ENV "MONITOR_TEST_EXECUTABLE=$<TARGET_FILE:localization_monitor>"
        "ROS_DOMAIN_ID=177" "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/alignment_test_logs")
endif()

ament_package()
