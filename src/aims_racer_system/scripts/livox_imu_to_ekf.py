#!/usr/bin/env python3
"""Convert Livox IMU messages into an EKF-friendly IMU topic."""

from __future__ import annotations

import math
from typing import Iterable, List

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def _mat_vec_mul(mat: List[List[float]], vec: Iterable[float]) -> List[float]:
    values = list(vec)
    return [sum(mat[r][c] * values[c] for c in range(3)) for r in range(3)]


def _mat_mul(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    return [
        [sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)]
        for r in range(3)
    ]


def _transpose(mat: List[List[float]]) -> List[List[float]]:
    return [[mat[c][r] for c in range(3)] for r in range(3)]


def _flat_to_mat(values: Iterable[float]) -> List[List[float]]:
    vals = list(values)
    return [vals[0:3], vals[3:6], vals[6:9]]


def _mat_to_flat(mat: List[List[float]]) -> List[float]:
    return [mat[r][c] for r in range(3) for c in range(3)]


def _diag_covariance(value: float) -> List[float]:
    return [
        value, 0.0, 0.0,
        0.0, value, 0.0,
        0.0, 0.0, value,
    ]


def _has_covariance(values: Iterable[float]) -> bool:
    return any(abs(float(v)) > 1e-12 for v in values)


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    rx = [
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ]
    ry = [
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ]
    rz = [
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ]
    return _mat_mul(_mat_mul(rz, ry), rx)


def _rotate_covariance(
    values: Iterable[float],
    rotation: List[List[float]],
    scale: float = 1.0,
) -> List[float]:
    covariance = _flat_to_mat(values)
    if scale != 1.0:
        covariance = [[scale * scale * v for v in row] for row in covariance]
    rotated = _mat_mul(_mat_mul(rotation, covariance), _transpose(rotation))
    return _mat_to_flat(rotated)


class LivoxImuToEkf(Node):
    def __init__(self) -> None:
        super().__init__('livox_imu_to_ekf')

        self.input_topic = str(self.declare_parameter('input_topic', '/livox/imu').value)
        self.output_topic = str(self.declare_parameter('output_topic', '/livox/imu_ekf').value)
        self.output_frame_id = str(self.declare_parameter('output_frame_id', 'imu_link').value)
        self.accel_scale = float(self.declare_parameter('accel_scale', 9.80665).value)
        self.rotation_rpy = self._vector_param(
            'rotation_rpy',
            [math.pi, 0.0, 0.0],
        )
        self.gravity_vector_mps2 = self._vector_param(
            'gravity_vector_mps2',
            [0.0, 0.0, -9.80665],
        )

        self.rotation = _rotation_from_rpy(*self.rotation_rpy)

        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.publisher = self.create_publisher(Imu, self.output_topic, output_qos)
        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            input_qos,
        )

        self.get_logger().info(
            'livox_imu_to_ekf: %s -> %s, frame=%s, rotation_rpy=%s, accel_scale=%.5f'
            % (
                self.input_topic,
                self.output_topic,
                self.output_frame_id,
                [round(v, 6) for v in self.rotation_rpy],
                self.accel_scale,
            )
        )

    def _vector_param(self, name: str, default: List[float]) -> List[float]:
        value = self.declare_parameter(name, default).value
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise RuntimeError(f"Parameter '{name}' must be a 3-element numeric list")
        try:
            return [float(v) for v in value]
        except Exception as exc:
            raise RuntimeError(f"Parameter '{name}' must contain numeric values") from exc

    def imu_callback(self, msg: Imu) -> None:
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id

        out.orientation.x = 0.0
        out.orientation.y = 0.0
        out.orientation.z = 0.0
        out.orientation.w = 1.0
        out.orientation_covariance = [
            -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ]

        gyro = _mat_vec_mul(
            self.rotation,
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
        )
        out.angular_velocity.x = gyro[0]
        out.angular_velocity.y = gyro[1]
        out.angular_velocity.z = gyro[2]

        accel_scaled = [
            msg.linear_acceleration.x * self.accel_scale,
            msg.linear_acceleration.y * self.accel_scale,
            msg.linear_acceleration.z * self.accel_scale,
        ]
        accel_rotated = _mat_vec_mul(self.rotation, accel_scaled)
        out.linear_acceleration.x = accel_rotated[0] - self.gravity_vector_mps2[0]
        out.linear_acceleration.y = accel_rotated[1] - self.gravity_vector_mps2[1]
        out.linear_acceleration.z = accel_rotated[2] - self.gravity_vector_mps2[2]

        if _has_covariance(msg.angular_velocity_covariance):
            out.angular_velocity_covariance = _rotate_covariance(
                msg.angular_velocity_covariance,
                self.rotation,
            )
        else:
            out.angular_velocity_covariance = _diag_covariance(0.01)

        if _has_covariance(msg.linear_acceleration_covariance):
            out.linear_acceleration_covariance = _rotate_covariance(
                msg.linear_acceleration_covariance,
                self.rotation,
                self.accel_scale,
            )
        else:
            out.linear_acceleration_covariance = _diag_covariance(0.05)

        self.publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxImuToEkf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
