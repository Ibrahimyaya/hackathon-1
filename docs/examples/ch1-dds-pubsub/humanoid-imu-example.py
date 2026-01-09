#!/usr/bin/env python3
"""
Realistic Humanoid IMU Sensor Example

This example demonstrates a realistic humanoid robot architecture:
- IMU sensor publishes orientation data at 100 Hz
- Planning node subscribes to IMU for balance monitoring
- Stabilization node subscribes to IMU for active balancing
- Logging node subscribes to IMU for diagnostics

Purpose: Show how multiple independent nodes interact via pub-sub,
decoupled and asynchronous, as in real humanoid robots.

Reference: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Topics.html
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math


class IMUSensorPublisher(Node):
    """
    Simulates a humanoid robot's IMU sensor.

    A real IMU measures:
    - Linear acceleration (3 axes)
    - Angular velocity (3 axes)
    - Magnetic field (3 axes)

    For humanoids, the most important is orientation/tilt detection
    for balance control.
    """

    def __init__(self):
        """Initialize IMU sensor node."""
        super().__init__('imu_sensor_publisher')

        # Sensor QoS: BEST_EFFORT at 100 Hz
        # We drop messages if subscriber can't keep up (OK for sensors)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST(1),
            depth=1
        )

        # Publisher for IMU data
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu/data',
            sensor_qos
        )

        # Publish at 100 Hz (typical for humanoid IMU)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.counter = 0

        self.get_logger().info('IMU Sensor ready on /imu/data (100 Hz)')

    def timer_callback(self):
        """Publish simulated IMU data."""
        msg = Imu()

        # Header with timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate humanoid tilting (like walking)
        # Tilt angle oscillates between ±5 degrees
        tilt_angle = 0.087 * math.sin(self.counter * 0.05)  # 5° = 0.087 rad

        # Linear acceleration: gravity (9.81 m/s²) + tilting
        msg.linear_acceleration.x = 9.81 * math.sin(tilt_angle)
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81 * math.cos(tilt_angle)

        # Angular velocity: humanoid's angular motion
        msg.angular_velocity.x = 0.1 * math.cos(self.counter * 0.05)  # Tilting rate
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        # Covariance (measurement uncertainty)
        # Diagonal elements: variance for each axis
        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        self.imu_publisher.publish(msg)
        self.counter += 1

        if self.counter % 100 == 0:
            self.get_logger().info(f'Published {self.counter} IMU messages')


class BalancePlanningSubscriber(Node):
    """
    Humanoid planning node that monitors balance.

    Subscribes to IMU to detect if robot is losing balance
    and needs corrective action.
    """

    def __init__(self):
        """Initialize balance planning node."""
        super().__init__('balance_planning_subscriber')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST(1),
                depth=1
            )
        )

        self.message_count = 0
        self.get_logger().info('Balance Planner ready, monitoring /imu/data')

    def imu_callback(self, msg):
        """Monitor IMU for balance detection."""
        self.message_count += 1

        # Calculate tilt angle from acceleration
        tilt = math.atan2(msg.linear_acceleration.x, msg.linear_acceleration.z)
        tilt_degrees = math.degrees(tilt)

        # Check for excessive tilt (> 10 degrees = unstable)
        if abs(tilt_degrees) > 10.0:
            self.get_logger().warn(f'HIGH TILT: {tilt_degrees:.1f}° - CORRECTING!')
        elif self.message_count % 100 == 0:
            self.get_logger().info(f'Balance stable, tilt: {tilt_degrees:.1f}°')


class StabilizationControlSubscriber(Node):
    """
    Humanoid stabilization controller.

    Real-time node that reacts to IMU data to maintain balance
    by commanding leg motors.
    """

    def __init__(self):
        """Initialize stabilization controller node."""
        super().__init__('stabilization_control_subscriber')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST(1),
                depth=1
            )
        )

        # (In real code, this would publish motor commands)
        self.motor_command_publisher = self.create_publisher(
            Float32,
            '/motor_command_ankles',
            10
        )

        self.message_count = 0
        self.get_logger().info('Stabilization Controller ready')

    def imu_callback(self, msg):
        """React to IMU changes with motor commands."""
        self.message_count += 1

        # Calculate correction needed
        tilt = math.atan2(msg.linear_acceleration.x, msg.linear_acceleration.z)
        correction = -tilt * 10.0  # PID-like correction

        # Publish motor command
        motor_cmd = Float32()
        motor_cmd.data = correction
        self.motor_command_publisher.publish(motor_cmd)

        if self.message_count % 100 == 0:
            self.get_logger().info(f'Published {self.message_count} motor commands')


class DiagnosticsLoggingSubscriber(Node):
    """
    Humanoid diagnostics and logging node.

    Slow subscriber for long-term health monitoring.
    Not time-critical, so can handle some message loss.
    """

    def __init__(self):
        """Initialize diagnostics logging node."""
        super().__init__('diagnostics_logging_subscriber')

        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST(1),
                depth=1
            )
        )

        self.message_count = 0
        self.get_logger().info('Diagnostics Logging ready')

    def imu_callback(self, msg):
        """Log IMU data for diagnostics (low frequency)."""
        self.message_count += 1

        # Only log every 10 seconds (100 Hz * 10 sec = 1000 messages)
        if self.message_count % 1000 == 0:
            acc = msg.linear_acceleration
            ang = msg.angular_velocity
            self.get_logger().info(
                f'Diagnostics: Accel=({acc.x:.2f}, {acc.y:.2f}, {acc.z:.2f}) '
                f'Angular=({ang.x:.2f}, {ang.y:.2f}, {ang.z:.2f})'
            )


def main(args=None):
    """
    Run all humanoid IMU nodes simultaneously.

    This demonstrates the decoupled architecture:
    - IMU publishes data at 100 Hz
    - Three subscribers react independently
    - Each subscriber has different timing/purpose
    - No coupling between subscribers
    """
    print('=== Humanoid IMU Sensor Example ===\n')

    rclpy.init(args=args)

    # Create all nodes
    nodes = [
        IMUSensorPublisher(),
        BalancePlanningSubscriber(),
        StabilizationControlSubscriber(),
        DiagnosticsLoggingSubscriber(),
    ]

    # Spin all nodes together using MultiThreadedExecutor
    executor = rclpy.executors.MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    print('All nodes running. Press Ctrl+C to stop.\n')

    try:
        executor.spin()
    except KeyboardInterrupt:
        print('\n\nShutting down...')
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

    print('=== Example Complete ===')


if __name__ == '__main__':
    main()
