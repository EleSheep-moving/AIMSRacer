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
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
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
ament_target_dependencies(imu_to_rear_axle rclcpp sensor_msgs)
target_compile_options(imu_to_rear_axle PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS imu_to_rear_axle DESTINATION lib/${PROJECT_NAME})

add_library(rear_axle_odometry STATIC src/rear_axle_odometry.cpp)
target_compile_features(rear_axle_odometry PUBLIC cxx_std_17)
target_include_directories(rear_axle_odometry PUBLIC include)
target_link_libraries(rear_axle_odometry PUBLIC Eigen3::Eigen)
ament_target_dependencies(rear_axle_odometry PUBLIC nav_msgs)
target_compile_options(rear_axle_odometry PRIVATE -Wall -Wextra -Wpedantic)

add_executable(lio_to_rear_axle src/lio_to_rear_axle.cpp)
target_link_libraries(lio_to_rear_axle rear_axle_odometry)
ament_target_dependencies(lio_to_rear_axle rclcpp nav_msgs sensor_msgs geometry_msgs tf2_ros)
target_compile_options(lio_to_rear_axle PRIVATE -Wall -Wextra -Wpedantic)

add_library(longitudinal_observer STATIC src/longitudinal_observer.cpp)
target_compile_features(longitudinal_observer PUBLIC cxx_std_17)
target_include_directories(longitudinal_observer PUBLIC include)
target_link_libraries(longitudinal_observer PUBLIC Eigen3::Eigen)
target_compile_options(longitudinal_observer PRIVATE -Wall -Wextra -Wpedantic)

add_executable(longitudinal_acceleration src/longitudinal_acceleration.cpp)
target_link_libraries(longitudinal_acceleration longitudinal_observer)
ament_target_dependencies(longitudinal_acceleration rclcpp geometry_msgs nav_msgs sensor_msgs diagnostic_msgs)
target_compile_options(longitudinal_acceleration PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS lio_to_rear_axle longitudinal_acceleration DESTINATION lib/${PROJECT_NAME})

add_library(body_acceleration_core STATIC src/body_acceleration.cpp)
target_compile_features(body_acceleration_core PUBLIC cxx_std_17)
target_include_directories(body_acceleration_core PUBLIC include)
target_link_libraries(body_acceleration_core PUBLIC Eigen3::Eigen)
target_compile_options(body_acceleration_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(body_acceleration src/body_acceleration_node.cpp)
target_link_libraries(body_acceleration body_acceleration_core)
ament_target_dependencies(body_acceleration rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs)
target_compile_options(body_acceleration PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS body_acceleration DESTINATION lib/${PROJECT_NAME})

add_library(yaw_rate_core STATIC src/yaw_rate.cpp)
target_compile_features(yaw_rate_core PUBLIC cxx_std_17)
target_include_directories(yaw_rate_core PUBLIC include)
target_compile_options(yaw_rate_core PRIVATE -Wall -Wextra -Wpedantic)
add_executable(yaw_rate src/yaw_rate_node.cpp)
target_link_libraries(yaw_rate yaw_rate_core)
ament_target_dependencies(yaw_rate rclcpp sensor_msgs geometry_msgs diagnostic_msgs)
target_compile_options(yaw_rate PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS yaw_rate DESTINATION lib/${PROJECT_NAME})

add_library(velocity_core STATIC src/velocity.cpp)
target_compile_features(velocity_core PUBLIC cxx_std_17)
target_include_directories(velocity_core PUBLIC include)
target_link_libraries(velocity_core PUBLIC Eigen3::Eigen)
target_compile_options(velocity_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(vx_observer src/velocity_node.cpp)
target_link_libraries(vx_observer velocity_core)
ament_target_dependencies(vx_observer rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs)
target_compile_options(vx_observer PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS vx_observer DESTINATION lib/${PROJECT_NAME})

add_executable(state_observer src/state_observer_node.cpp)
target_compile_features(state_observer PRIVATE cxx_std_17)
target_link_libraries(state_observer velocity_core body_acceleration_core yaw_rate_core)
ament_target_dependencies(state_observer rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs)
target_compile_options(state_observer PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS state_observer DESTINATION lib/${PROJECT_NAME})

add_library(gyro_bias STATIC src/gyro_bias.cpp)
target_compile_features(gyro_bias PUBLIC cxx_std_17)
target_include_directories(gyro_bias PUBLIC include)
target_link_libraries(gyro_bias PUBLIC Eigen3::Eigen)
ament_target_dependencies(gyro_bias PUBLIC sensor_msgs)
target_compile_options(gyro_bias PRIVATE -Wall -Wextra -Wpedantic)

add_executable(livox_gyro_bias src/livox_gyro_bias.cpp)
target_link_libraries(livox_gyro_bias gyro_bias)
ament_target_dependencies(livox_gyro_bias rclcpp sensor_msgs nav_msgs diagnostic_msgs)
target_compile_options(livox_gyro_bias PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS livox_gyro_bias DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/activate_ndt.py scripts/relocalize_known_map.py
  scripts/localization_policy.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

if(BUILD_TESTING)
  add_executable(yaw_rate_core_test tests/test_yaw_rate.cpp)
  target_link_libraries(yaw_rate_core_test yaw_rate_core)
  target_compile_options(yaw_rate_core_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME yaw_rate_core COMMAND yaw_rate_core_test)
  add_executable(yaw_rate_trace tests/yaw_rate_trace.cpp)
  target_link_libraries(yaw_rate_trace yaw_rate_core)
  add_executable(tf_authority_audit ${CMAKE_CURRENT_SOURCE_DIR}/../../verification/localization/tf_authority_audit.cpp)
  target_compile_features(tf_authority_audit PRIVATE cxx_std_17)
  ament_target_dependencies(tf_authority_audit rclcpp tf2_msgs)
  target_compile_options(tf_authority_audit PRIVATE -Wall -Wextra -Wpedantic)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(velocity_core_test tests/test_velocity.cpp)
  target_link_libraries(velocity_core_test velocity_core)
  add_executable(velocity_trace tests/velocity_trace.cpp)
  target_link_libraries(velocity_trace velocity_core)
  ament_add_gtest(rear_axle_imu_test tests/test_rear_axle_imu.cpp)
  target_link_libraries(rear_axle_imu_test rear_axle_imu)
  target_compile_definitions(rear_axle_imu_test PRIVATE
    IMU_REFERENCE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/data/imu_python_reference.csv")
  ament_add_gtest(rear_axle_odometry_test tests/test_rear_axle_odometry.cpp)
  target_link_libraries(rear_axle_odometry_test rear_axle_odometry)
  ament_add_gtest(longitudinal_observer_test tests/test_longitudinal_observer.cpp)
  target_link_libraries(longitudinal_observer_test longitudinal_observer)
  target_compile_options(longitudinal_observer_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_executable(longitudinal_observer_trace tests/longitudinal_observer_trace.cpp)
  target_link_libraries(longitudinal_observer_trace longitudinal_observer)
  add_executable(gyro_bias_test tests/test_gyro_bias.cpp)
  target_link_libraries(gyro_bias_test gyro_bias)
  target_compile_options(gyro_bias_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_test(NAME native_gyro_bias COMMAND gyro_bias_test)
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
  ament_add_gtest(body_acceleration_core_test tests/test_body_acceleration.cpp)
  target_link_libraries(body_acceleration_core_test body_acceleration_core)
  target_compile_options(body_acceleration_core_test PRIVATE -UNDEBUG -Wall -Wextra -Wpedantic)
  add_executable(body_acceleration_trace tests/body_acceleration_trace.cpp)
  target_link_libraries(body_acceleration_trace body_acceleration_core)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(state_observer_runtime tests/test_state_observer_runtime.py TIMEOUT 180
    ENV "STATE_OBSERVER_EXECUTABLE=$<TARGET_FILE:state_observer>"
        "ROS_DOMAIN_ID=193" "ROS_LOCALHOST_ONLY=1"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/state_observer_test_logs")
  ament_add_pytest_test(velocity_runtime tests/test_velocity_runtime.py TIMEOUT 120
    ENV "VELOCITY_EXECUTABLE=$<TARGET_FILE:vx_observer>" "ROS_DOMAIN_ID=197" "ROS_LOCALHOST_ONLY=1"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/velocity_test_logs")
  ament_add_pytest_test(planar_acceleration_runtime tests/test_planar_acceleration_runtime.py
    ENV "BODY_ACCELERATION_EXECUTABLE=$<TARGET_FILE:body_acceleration>"
        "ROS_DOMAIN_ID=196" "ROS_LOCALHOST_ONLY=1"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/planar_test_logs")
  ament_add_pytest_test(yaw_rate_runtime tests/test_yaw_rate_runtime.py
    ENV "YAW_RATE_EXECUTABLE=$<TARGET_FILE:yaw_rate>"
        "ROS_DOMAIN_ID=194" "ROS_LOCALHOST_ONLY=1"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/yaw_rate_test_logs")
  ament_add_pytest_test(rear_axle_contract tests/test_rear_axle_contract.py
    ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_logs")
  ament_add_pytest_test(rear_axle_timestamps tests/test_rear_axle_timestamps.py
    ENV "ROS_DOMAIN_ID=188" "ROS_LOCALHOST_ONLY=1"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/timestamp_test_logs")
  ament_add_pytest_test(gyro_bias tests/test_gyro_bias.py)
  ament_add_pytest_test(current_launch tests/test_current_launch.py)
  ament_add_pytest_test(controller_calibration_launch tests/test_controller_calibration_launch.py)
  ament_add_pytest_test(full_step_launch tests/test_full_step_launch.py)
  ament_add_pytest_test(longitudinal_observer
    ${CMAKE_CURRENT_SOURCE_DIR}/../../verification/acceleration/test_observer.py)
  ament_add_pytest_test(longitudinal_acceleration tests/test_longitudinal_acceleration.py
    ENV "NATIVE_ACCELERATION_EXECUTABLE=$<TARGET_FILE:longitudinal_acceleration>"
        "ROS_DOMAIN_ID=189" "ROS_LOCALHOST_ONLY=1"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/acceleration_test_logs")
  ament_add_pytest_test(native_acceleration_runtime tests/test_native_acceleration_runtime.py
    ENV "NATIVE_ACCELERATION_EXECUTABLE=$<TARGET_FILE:longitudinal_acceleration>"
        "ROS_DOMAIN_ID=190" "ROS_LOCALHOST_ONLY=1"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/native_acceleration_test_logs")
  ament_add_pytest_test(body_acceleration_runtime tests/test_body_acceleration_runtime.py
    TIMEOUT 120
    ENV "BODY_ACCELERATION_EXECUTABLE=$<TARGET_FILE:body_acceleration>"
        "ROS_DOMAIN_ID=192" "ROS_LOCALHOST_ONLY=1"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/body_acceleration_test_logs")
  ament_add_pytest_test(native_gyro_runtime tests/test_native_gyro_runtime.py
    ENV "NATIVE_GYRO_EXECUTABLE=$<TARGET_FILE:livox_gyro_bias>"
        "ROS_DOMAIN_ID=191" "ROS_LOCALHOST_ONLY=1"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/native_gyro_test_logs")
  ament_add_pytest_test(native_observer_parity tests/test_native_observer_parity.py
    ENV "NATIVE_OBSERVER_TRACE_EXECUTABLE=$<TARGET_FILE:longitudinal_observer_trace>")
  ament_add_pytest_test(localization_policy tests/test_localization_policy.py)
  ament_add_pytest_test(initializer_lifecycle tests/test_initializer_lifecycle.py)
  ament_add_pytest_test(replay_sensor_qos tests/test_replay_sensor_qos.py)
  ament_add_pytest_test(replay_acceptance tests/test_replay_acceptance.py)
  ament_add_pytest_test(monitor_alignment_delivery tests/test_monitor_alignment_delivery.py
    ENV "MONITOR_TEST_EXECUTABLE=$<TARGET_FILE:localization_monitor>"
        "ROS_DOMAIN_ID=177" "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/alignment_test_logs")
endif()

ament_package()
