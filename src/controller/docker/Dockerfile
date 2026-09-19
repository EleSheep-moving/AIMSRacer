FROM ros:humble@sha256:39b4e11de5796d70e2177833f48ff68db317018e86fc66e865e6226814fce416 AS dependencies
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 \
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-pytest python3-colcon-common-extensions python3-venv \
    ros-humble-ackermann-msgs ros-humble-diagnostic-updater \
    ros-humble-tf2-geometry-msgs ros-humble-ament-cmake-auto \
    ros-humble-ament-lint-auto ros-humble-ament-lint-common \
    ros-humble-launch-testing-ros ros-humble-launch-testing-ament-cmake \
    ros-humble-rosbag2-storage-default-plugins \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir casadi==3.7.2 numpy==1.26.4 scipy==1.15.3 PyYAML==6.0.2
WORKDIR /ws

FROM dependencies AS ros_dependencies
COPY src/ackermann_mux /ws/src/ackermann_mux
COPY src/ros2_crsf_receiver/crsf_receiver_msg /ws/src/crsf_receiver_msg
COPY src/vesc/vesc_msgs /ws/src/vesc_msgs
COPY src/vesc/vesc_ackermann /ws/src/vesc_ackermann
RUN source /opt/ros/humble/setup.bash && colcon build --packages-up-to ackermann_mux vesc_ackermann \
    --cmake-args -DBUILD_TESTING=OFF
COPY src/aims_racer_system/params/vesc.yaml /ws/test_config/vesc.yaml

FROM ros_dependencies AS runtime
RUN python3 -m pip install --no-cache-dir matplotlib==3.8.4
COPY src/controller /ws/src/controller
RUN source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && colcon build --packages-select aims_mpcc
COPY src/controller/docker/entrypoint.sh /mpcc-entrypoint.sh
ENTRYPOINT ["/bin/bash", "/mpcc-entrypoint.sh"]
CMD ["python3", "-m", "pytest", "-q", "/ws/src/controller/test"]
