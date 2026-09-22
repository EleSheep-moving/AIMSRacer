#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from crsf_receiver_msg.msg import CRSFChannels16
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Bool


class JoystickControl(Node):
    CHANNEL_PROFILE = "sequential_ch1_ch2"
    DEFAULT_CHANNELS = {
        "speed_channel": 1,
        "steering_channel": 2,
        "lock_channel": 3,
        "esc_mode_channel": 4,
        "control_source_channel": 5,
        "limit_channel": 6,
        "calib_mode_channel": 7,
    }

    def __init__(self):
        super().__init__("joystick_control")
        self.get_logger().info(
            "joystick_control started with channel profile: %s"
            % self.CHANNEL_PROFILE
        )
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # 订阅CRSFChannels16消息
        self.subscription_joystick = self.create_subscription(
            CRSFChannels16, "/rc/channels", self.joystick_callback, qos_profile
        )
        
        self.ackermann_subscriber = self.create_subscription(
            AckermannDriveStamped,
            "/drive",
            self.ackermann_callback,
            qos_profile
        )

        self.ackermann_calib_subscriber = self.create_subscription(
            AckermannDriveStamped,
            "/calib/ackermann_cmd",
            self.calib_ackermann_callback,
            qos_profile
        )
        
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        # Selection status is independent of /drive availability (no enable cycle).
        self.autonomy_status_publisher = self.create_publisher(
            Bool, "/control/autonomy_speed_enabled", 10
        )

        self.direction_reverse = self.declare_parameter("direction_reverse", False).value

        # CRSF channel numbers are 1-based here: self.channel[1] == msg.ch1.
        self.speed_channel = self.declare_parameter(
            "speed_channel", self.DEFAULT_CHANNELS["speed_channel"]
        ).value
        self.steering_channel = self.declare_parameter(
            "steering_channel", self.DEFAULT_CHANNELS["steering_channel"]
        ).value
        self.lock_channel = self.declare_parameter(
            "lock_channel", self.DEFAULT_CHANNELS["lock_channel"]
        ).value
        self.esc_mode_channel = self.declare_parameter(
            "esc_mode_channel", self.DEFAULT_CHANNELS["esc_mode_channel"]
        ).value
        self.control_source_channel = self.declare_parameter(
            "control_source_channel",
            self.DEFAULT_CHANNELS["control_source_channel"],
        ).value
        self.limit_channel = self.declare_parameter(
            "limit_channel", self.DEFAULT_CHANNELS["limit_channel"]
        ).value
        self.calib_mode_channel = self.declare_parameter(
            "calib_mode_channel", self.DEFAULT_CHANNELS["calib_mode_channel"]
        ).value

        # Keep the old channel8 parameter names as fallbacks for existing launch files.
        old_limit_min_value = self.declare_parameter("channel8_min_value", 172).value
        old_limit_max_value = self.declare_parameter("channel8_max_value", 1810).value
        self.limit_min_value = self.declare_parameter("limit_min_value", old_limit_min_value).value
        self.limit_max_value = self.declare_parameter("limit_max_value", old_limit_max_value).value

        old_speed_limit_min_speed = self.declare_parameter("speed_channel8_min_speed", 2.0).value
        old_speed_limit_max_speed = self.declare_parameter("speed_channel8_max_speed", 12.0).value
        self.speed_limit_min_speed = self.declare_parameter(
            "speed_limit_min_speed", old_speed_limit_min_speed
        ).value
        self.speed_limit_max_speed = self.declare_parameter(
            "speed_limit_max_speed", old_speed_limit_max_speed
        ).value
        self.steering_limit = self.declare_parameter("steering_limit", 0.4751).value
        self.steering_reverse = self.declare_parameter("steering_reverse", True).value
        old_channel_mid = self.declare_parameter("steering_channel_mid", 992).value
        self.channel_mid = self.declare_parameter("channel_mid", old_channel_mid).value
        self.channel_deadzone = self.declare_parameter(
            "channel_deadzone", 100
        ).value
        self.channel_min_range = self.declare_parameter("channel_min_range", 172).value
        self.channel_max_range = self.declare_parameter("channel_max_range", 1810).value
        self.switch_mid_value = self.declare_parameter("switch_mid_value", self.channel_mid).value
        default_switch_low_threshold = (self.channel_min_range + self.switch_mid_value) / 2.0
        default_switch_high_threshold = (self.switch_mid_value + self.channel_max_range) / 2.0
        self.switch_low_threshold = self.declare_parameter(
            "switch_low_threshold", default_switch_low_threshold
        ).value
        self.switch_high_threshold = self.declare_parameter(
            "switch_high_threshold", default_switch_high_threshold
        ).value

        self.channel = None
        self.nav_ackermann_msg = None
        self.calib_ackermann_msg = None

        

        old_current_limit_min_current = self.declare_parameter("current_channel8_min_current", 3.0).value
        old_current_limit_max_current = self.declare_parameter("current_channel8_max_current", 20.0).value
        self.current_limit_min_current = self.declare_parameter(
            "current_limit_min_current", old_current_limit_min_current
        ).value
        self.current_limit_max_current = self.declare_parameter(
            "current_limit_max_current", old_current_limit_max_current
        ).value
        
        # 状态变量
        self.rc_connected = False
        self.locked = False
        self.control_mode = "none"  # 可能的值: "teleop", "nav", "none"
        self.esc_control_mode = "none"  # 可能的值: "speed", "current", "none"
        self.calib_mode = False
        self.last_joystick_time = self.get_clock().now()
        self.last_nav_time = self.get_clock().now()
        
        # 追踪状态变化，只在改变时print
        self.last_control_mode = "none"
        self.last_esc_control_mode = "none"
        self.last_calib_mode = False
        self.last_locked = False

        self.get_logger().info("direction_reverse: %s" % self.direction_reverse)
        self.get_logger().info("speed_channel: %d" % self.speed_channel)
        self.get_logger().info("steering_channel: %d" % self.steering_channel)
        self.get_logger().info("lock_channel: %d" % self.lock_channel)
        self.get_logger().info("esc_mode_channel: %d" % self.esc_mode_channel)
        self.get_logger().info("control_source_channel: %d" % self.control_source_channel)
        self.get_logger().info("limit_channel: %d" % self.limit_channel)
        self.get_logger().info("calib_mode_channel: %d" % self.calib_mode_channel)
        self.get_logger().info("limit_min_value: %d, max_value: %d" % (self.limit_min_value, self.limit_max_value))
        self.get_logger().info("speed_limit_min_speed: %f, max_speed: %f" % (self.speed_limit_min_speed, self.speed_limit_max_speed))
        self.get_logger().info("steering_limit: %f" % self.steering_limit)
        self.get_logger().info("steering_reverse: %s" % self.steering_reverse)
        self.get_logger().info("channel_min/mid/max: %d / %d / %d" % (self.channel_min_range, self.channel_mid, self.channel_max_range))
        self.get_logger().info("channel_deadzone: %d" % self.channel_deadzone)
        self.get_logger().info("switch_mid_value: %d" % self.switch_mid_value)
        self.get_logger().info("switch_low/high_threshold: %.1f / %.1f" % (self.switch_low_threshold, self.switch_high_threshold))
        # 200hz
        self.timer = self.create_timer(0.005, self.timer_callback)

    def joystick_callback(self, msg):
        self.channel = [
            0,
            msg.ch1,
            msg.ch2,
            msg.ch3,
            msg.ch4,
            msg.ch5,
            msg.ch6,
            msg.ch7,
            msg.ch8,
            msg.ch9,
            msg.ch10,
            msg.ch11,
            msg.ch12,
            msg.ch13,
            msg.ch14,
            msg.ch15,
            msg.ch16,
        ]

        self.locked = self.channel[self.lock_channel] < self.switch_mid_value
        self.control_mode = "teleop" if self.channel[self.control_source_channel] < self.switch_mid_value else "nav"
        
        if self.channel[self.esc_mode_channel] < self.switch_low_threshold:
            self.esc_control_mode = "speed"
        elif self.channel[self.esc_mode_channel] < self.switch_high_threshold:
            self.esc_control_mode = "current"
        else:
            self.esc_control_mode = "duty"
            
        if self.channel[self.calib_mode_channel] > self.switch_mid_value:
            self.calib_mode = True
        else:
            self.calib_mode = False
            
        self.last_joystick_time = self.get_clock().now()
        
        # 只在状态改变时print
        if self.control_mode != self.last_control_mode:
            self.get_logger().info("Control mode changed: %s -> %s" % (self.last_control_mode, self.control_mode))
            self.last_control_mode = self.control_mode
            
        if self.esc_control_mode != self.last_esc_control_mode:
            self.get_logger().info("ESC control mode changed: %s -> %s" % (self.last_esc_control_mode, self.esc_control_mode))
            self.last_esc_control_mode = self.esc_control_mode
            
        if self.calib_mode != self.last_calib_mode:
            self.get_logger().info("Calibration mode changed: %s -> %s" % (self.last_calib_mode, self.calib_mode))
            self.last_calib_mode = self.calib_mode
        
        if self.locked != self.last_locked:
            self.get_logger().info("Locked changed: %s -> %s" % (self.last_locked, self.locked))
            self.last_locked = self.locked
            
        # Calibration mode is routed by upstream /calib/ackermann_cmd.drive.jerk.
        # Do not rely on esc_control_mode during calibration; keep RC mode switching lightweight.


    def ackermann_callback(self, msg):
        self.nav_ackermann_msg = msg
        self.last_nav_time = self.get_clock().now()  # ✅ 添加这一行
        
    def calib_ackermann_callback(self, msg):
        self.calib_ackermann_msg = msg
    def handle_channel_input(self, channel_index,
                            limit_min_value, limit_max_value,
                            limit_min_range, limit_max_range):
        raw_value = self.channel[channel_index]
        
        limit_value = self.channel[self.limit_channel]
        limit_value = max(limit_min_value, min(limit_value, limit_max_value))
        if limit_max_value <= limit_min_value:
            ratio = 1.0
        else:
            ratio = (limit_value - limit_min_value) / (
                limit_max_value - limit_min_value
            )
        current_max = limit_min_range + ratio * (limit_max_range - limit_min_range)
        current_min = -current_max

        # 处理死区
        if abs(raw_value - self.channel_mid) > self.channel_deadzone:
            # Normalize to -1 to 1
            if raw_value > self.channel_mid:
                normalized = (raw_value - self.channel_mid) / (
                    self.channel_max_range - self.channel_mid
                )
            else:
                normalized = (raw_value - self.channel_mid) / (
                    self.channel_mid - self.channel_min_range
                )
            
            # ✅ 直接乘以 current_max（normalized 已经是 -1 到 1）
            value = normalized * current_max
            
            # Clamp to min and max（其实已经不需要了，但保险起见可以保留）
            value = max(current_min, min(value, current_max))
        else:
            value = 0.0
            
        return value

    def handle_teleop_speed(self):
        speed_value = self.handle_channel_input(
            self.speed_channel,
            self.limit_min_value,
            self.limit_max_value,
            self.speed_limit_min_speed,
            self.speed_limit_max_speed
        )
        
        if self.direction_reverse:
            speed_value = -speed_value
            
        return speed_value
        
    def handle_teleop_current(self):
        current_value = self.handle_channel_input(
            self.speed_channel,
            self.limit_min_value,
            self.limit_max_value,
            self.current_limit_min_current,
            self.current_limit_max_current
        )
        
        if self.direction_reverse:
            current_value = -current_value
            
        return current_value


    def handle_teleop_duty(self):
        duty_value = self.handle_channel_input(
            self.speed_channel,
            self.limit_min_value,
            self.limit_max_value,
            0.0, # 0.0 to 1.0 min 
            0.8 # 0.0 to 1.0 max
        )
        if self.direction_reverse:
            duty_value = -duty_value
            
        return duty_value 
    def handle_teleop_steer(self):
    
        raw_steering = self.channel[self.steering_channel]
                
        # if abs(raw_steering - self.channel_mid) <= self.channel_deadzone*0.2:
        #     return 0.0
        
        # Normalize steering value to -1 to 1
        if raw_steering > self.channel_mid:
            normalized_steering = (raw_steering - self.channel_mid) / (
                self.channel_max_range - self.channel_mid
            )
        else:
            normalized_steering = (raw_steering - self.channel_mid) / (
                self.channel_mid - self.channel_min_range
            )
        steering_range = self.steering_limit
        steering_value = normalized_steering * steering_range
        if self.steering_reverse:
            steering_value = -steering_value
        return steering_value

        
    def publish_ackermann_none(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        self.ackermann_publisher.publish(msg)

    def publish_ackermann(self, steering_angle, speed):
        # 常规steering_angle + speed
        # 用于遥控器 和 nav2导航模式
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0 # 标志位，用来区分是否是常规模式
        self.ackermann_publisher.publish(msg)

    def publish_ackermann_acceleration(self, steering_angle, speed,acceleration):
        # 加速度前馈模式 steering_angle + speed + acceleration
        # 用于mpc输出acc + speed
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = acceleration
        msg.drive.jerk = 1.0 # 标志位，用来区分是否是加速度前馈模式
        self.ackermann_publisher.publish(msg)

    def publish_ackermann_current(self, steering_angle,current):
        # 电流控制模式 steering_angle + current
        # 用于teleop模式 和 calibration模式
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = current
        msg.drive.jerk = 2.0 # 标志位，用来区分是否current直接驱动模式
        self.ackermann_publisher.publish(msg)

    def publish_ackermann_duty(self, steering_angle,duty):
        # 占空比控制模式 steering_angle + duty
        # 用于teleop模式 和 calibration模式
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = steering_angle
        msg.drive.steering_angle_velocity = 0.0
        msg.drive.acceleration = duty
        msg.drive.jerk = 3.0 # 标志位，用来区分是否duty直接驱动模式
        self.ackermann_publisher.publish(msg)

        
    def timer_callback(self):
        selection = Bool()
        rc_fresh = (self.channel is not None and
                    0.0 <= (self.get_clock().now() - self.last_joystick_time).nanoseconds / 1e9 <= 0.2)
        selection.data = bool(rc_fresh and not self.locked and
                              self.control_mode == "nav" and
                              self.esc_control_mode == "speed" and not self.calib_mode)
        self.autonomy_status_publisher.publish(selection)
        
        if self.channel is None:
            self.get_logger().warn("Waiting for RC input...", throttle_duration_sec=1.0)
            return

        if self.locked:
            self.publish_ackermann_none()
            return

        # 检查rc输入是否超时
        rc_timeout = (self.get_clock().now() - self.last_joystick_time).nanoseconds / 1e9 > 0.2
        # 检查drive输入是否超时
        nav_timeout = (self.get_clock().now() - self.last_nav_time).nanoseconds / 1e9 > 0.2
        
        
        # 判断rc响应
        if not rc_timeout:
            self.rc_connected = True
        else:
            self.rc_connected = False
            self.get_logger().warn("No joystick input")
            self.publish_ackermann_none()
            if (self.get_clock().now() - self.last_joystick_time).nanoseconds / 1e9 > 5.0:
                self.get_logger().warn("No joystick input for 5 seconds, shutting down...")
                rclpy.shutdown()
            return
            
        if self.calib_mode and not rc_timeout:
            # Calibration passthrough from /calib/ackermann_cmd.
            # Mode is encoded in drive.jerk (consistent with vesc_ackermann):
            # 0.0=speed, 2.0=current, 3.0=duty, 1.0=accel (optional).
            if self.calib_ackermann_msg is None:
                self.get_logger().warn("No calibration message")
                self.publish_ackermann_none()
                return

            jerk = float(self.calib_ackermann_msg.drive.jerk)
            steer = float(self.calib_ackermann_msg.drive.steering_angle)

            if jerk == 0.0:
                # Speed mode
                self.publish_ackermann(steer, float(self.calib_ackermann_msg.drive.speed))
                return

            if jerk == 2.0:
                # Current mode: current is carried in drive.acceleration
                self.publish_ackermann_current(steer, float(self.calib_ackermann_msg.drive.acceleration))
                return

            if jerk == 3.0:
                # Duty mode: duty is carried in drive.acceleration
                self.publish_ackermann_duty(steer, float(self.calib_ackermann_msg.drive.acceleration))
                return

            if jerk == 1.0:
                # Accel feedforward mode (only if downstream supports it)
                self.publish_ackermann_acceleration(
                    steer,
                    float(self.calib_ackermann_msg.drive.speed),
                    float(self.calib_ackermann_msg.drive.acceleration),
                )
                return

            self.get_logger().warn(f"Unknown calib jerk={jerk:.2f}; stopping")
            self.publish_ackermann_none()
            return
        
        if self.esc_control_mode == "speed":
            if not self.rc_connected:
                self.publish_ackermann_none()
                return

            if self.control_mode == "nav":
                if self.nav_ackermann_msg is None:
                    self.get_logger().warn("No nav message received yet", throttle_duration_sec=1.0)
                    self.publish_ackermann_none()
                    return
                elif nav_timeout:
                    self.get_logger().warn("No nav message received for 0.2 seconds")
                    self.publish_ackermann_none()
                else:    
                    self.publish_ackermann(
                        self.nav_ackermann_msg.drive.steering_angle,
                        self.nav_ackermann_msg.drive.speed
                    )
            elif self.control_mode == "teleop":
                speed_value = self.handle_teleop_speed()
                steering_value = self.handle_teleop_steer()
                self.publish_ackermann(steering_value, speed_value)
        elif self.esc_control_mode == "current":
            if not self.rc_connected:
                self.publish_ackermann_none()
                return

            if self.control_mode == "nav":
                if self.nav_ackermann_msg is None:
                    self.get_logger().warn("No nav message received yet", throttle_duration_sec=1.0)
                    self.publish_ackermann_none()
                    return
                elif nav_timeout:
                    self.get_logger().warn("No nav message received for 0.2 seconds")
                else:    
                    self.publish_ackermann_acceleration(
                        self.nav_ackermann_msg.drive.steering_angle,
                        self.nav_ackermann_msg.drive.speed,
                        self.nav_ackermann_msg.drive.acceleration
                    )
                    
            elif self.control_mode == "teleop":
                current_value = self.handle_teleop_current()
                steering_value = self.handle_teleop_steer()
                self.publish_ackermann_current(steering_value, current_value)

        elif self.esc_control_mode == "duty":
            if not self.rc_connected:
                self.publish_ackermann_none()
                return
            
            if self.control_mode == "teleop":
                duty_value = self.handle_teleop_duty()
                steering_value = self.handle_teleop_steer()
                self.publish_ackermann_duty(steering_value, duty_value)
            elif self.control_mode == "nav":
                if self.nav_ackermann_msg is None:
                    self.get_logger().warn("No nav message received yet", throttle_duration_sec=1.0)
                    self.publish_ackermann_none()
                    return
                elif nav_timeout:
                    self.get_logger().warn("No nav message received for 0.2 seconds")
                else:    
                    self.get_logger().warn("Duty mode not supported for nav")
def main(args=None):
    rclpy.init(args=args)
    joystick_control = JoystickControl()
    rclpy.spin(joystick_control)
    # Clean up
    joystick_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
