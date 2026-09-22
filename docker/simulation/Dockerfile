FROM ros:humble@sha256:39b4e11de5796d70e2177833f48ff68db317018e86fc66e865e6226814fce416

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    OPENBLAS_NUM_THREADS=1 \
    OMP_NUM_THREADS=1 \
    MKL_NUM_THREADS=1

RUN apt-get update && apt-get install -y --no-install-recommends \
    ccache gcc g++ python3-colcon-common-extensions python3-pip \
    ros-humble-ackermann-msgs ros-humble-ament-cmake-auto \
    ros-humble-diagnostic-updater ros-humble-tf2-geometry-msgs \
    ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir \
    casadi==3.7.2 numpy==1.26.4 scipy==1.15.3 PyYAML==6.0.2

WORKDIR /ws
COPY src/ackermann_mux /ws/src/ackermann_mux
COPY src/ros2_crsf_receiver/crsf_receiver_msg /ws/src/crsf_receiver_msg
COPY src/vesc/vesc_msgs /ws/src/vesc_msgs
COPY src/vesc/vesc_ackermann /ws/src/vesc_ackermann
COPY src/controller /ws/src/controller
COPY src/aims_mpcc_sim /ws/src/aims_mpcc_sim
COPY src/aims_gazebo_sim /ws/src/aims_gazebo_sim
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-up-to aims_gazebo_sim --cmake-args -DBUILD_TESTING=OFF

COPY docker/simulation/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
