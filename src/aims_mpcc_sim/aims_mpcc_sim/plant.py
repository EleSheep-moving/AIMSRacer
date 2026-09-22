"""Lagged kinematic bicycle plant driven by the VESC converter outputs."""

from dataclasses import dataclass
import math


def autonomous_channels() -> list[int]:
    """Return the V2 transmitter state selecting unlocked speed-mode autonomy."""
    values = [992] * 16
    values[4] = 1810
    values[5] = 172
    values[6] = 1810
    values[7] = 172
    values[9] = 172
    return values


def odometry_fields(plant: 'LaggedBicyclePlant') -> dict[str, float]:
    return {
        'x': plant.x,
        'y': plant.y,
        'yaw': plant.yaw,
        'speed': plant.speed,
        'yaw_rate': plant.speed * math.tan(plant.steering) / plant.wheelbase,
    }


@dataclass
class LaggedBicyclePlant:
    wheelbase: float
    speed_to_erpm_gain: float
    steering_gain: float
    steering_offset: float
    speed_tau: float
    steering_tau: float
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    steering: float = 0.0
    speed_target: float = 0.0
    steering_target: float = 0.0

    def receive_erpm(self, erpm: float) -> None:
        self.speed_target = erpm / self.speed_to_erpm_gain

    def receive_servo(self, servo: float) -> None:
        self.steering_target = (servo - self.steering_offset) / self.steering_gain

    def step(self, dt: float) -> None:
        self.speed += (self.speed_target - self.speed) * (1.0 - math.exp(-dt / self.speed_tau))
        self.steering += (self.steering_target - self.steering) * (1.0 - math.exp(-dt / self.steering_tau))
        yaw_rate = self.speed * math.tan(self.steering) / self.wheelbase
        heading = self.yaw + 0.5 * yaw_rate * dt
        self.x += self.speed * math.cos(heading) * dt
        self.y += self.speed * math.sin(heading) * dt
        self.yaw += yaw_rate * dt
