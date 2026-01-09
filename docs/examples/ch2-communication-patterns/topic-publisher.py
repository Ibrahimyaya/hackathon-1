#!/usr/bin/env python3
"""
ROS 2 Topic Publisher: Humanoid Joint States

This example demonstrates publishing realistic humanoid robot joint state data
at high frequency (100 Hz) with proper QoS settings for sensor streams.

Purpose: Show how to publish sensor data continuously to multiple subscribers
Reference: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Topics.html
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
import math


class HumanoidJointStatePublisher(Node):
    """
    Publishes joint state data for a humanoid robot.

    A real humanoid robot has 12+ joints across:
    - Left arm: shoulder (3 DOF), elbow (1 DOF), wrist (2 DOF)
    - Right arm: shoulder (3 DOF), elbow (1 DOF), wrist (2 DOF)
    - Left leg: hip (3 DOF), knee (1 DOF), ankle (2 DOF)
    - Right leg: hip (3 DOF), knee (1 DOF), ankle (2 DOF)
    - Waist/torso: 1 DOF
    - Head: 1-2 DOF

    This example simulates a simplified 12-joint humanoid.
    """

    def __init__(self):
        """Initialize the joint state publisher."""
        super().__init__('humanoid_joint_state_publisher')

        # Sensor QoS: BEST_EFFORT for high-frequency sensor streams
        # - BEST_EFFORT: Don't retry if delivery fails (fast)
        # - KEEP_LAST(1): Only keep newest message (fresh data)
        # - 100 Hz is typical for robot sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST(1),
            depth=1
        )

        # Create publisher on standard joint states topic
        # sensor_msgs/JointState is the standard ROS 2 message for robot joint data
        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_states',
            sensor_qos
        )

        # Publish at 100 Hz (0.01 seconds between updates)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.counter = 0

        # Define humanoid joint names and initial positions
        self.joint_names = [
            # Left arm (3 + 1 + 2 DOF)
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow_pitch',
            'left_wrist_roll', 'left_wrist_pitch',
            # Right arm (3 + 1 + 2 DOF)
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
            'right_elbow_pitch',
            'right_wrist_roll', 'right_wrist_pitch',
        ]

        # Initial joint positions (radians)
        # These are rest positions (standing upright)
        self.joint_positions = [0.0] * len(self.joint_names)

        # Joint limits (min, max) in radians for each joint
        # These define the valid range of motion
        self.joint_limits = {
            'left_shoulder_pitch': (-1.57, 1.57),   # ±90°
            'left_shoulder_roll': (-1.57, 1.57),    # ±90°
            'left_shoulder_yaw': (-1.57, 1.57),     # ±90°
            'left_elbow_pitch': (0, 2.36),          # 0-135°
            'left_wrist_roll': (-0.785, 0.785),     # ±45°
            'left_wrist_pitch': (-0.785, 0.785),    # ±45°
            'right_shoulder_pitch': (-1.57, 1.57),
            'right_shoulder_roll': (-1.57, 1.57),
            'right_shoulder_yaw': (-1.57, 1.57),
            'right_elbow_pitch': (0, 2.36),
            'right_wrist_roll': (-0.785, 0.785),
            'right_wrist_pitch': (-0.785, 0.785),
        }

        self.get_logger().info(
            f'Joint State Publisher ready on /joint_states (100 Hz)\n'
            f'Publishing {len(self.joint_names)} joints'
        )

    def timer_callback(self):
        """Publish joint state message every 0.01 seconds."""
        # Create JointState message
        msg = JointState()

        # Header: timestamp and frame reference
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # Reference frame for all joints

        # Joint names
        msg.name = self.joint_names

        # Simulate realistic joint motion
        # Using sine waves to show cyclic arm motion (like walking)
        # Each joint oscillates slightly around its rest position
        time_sec = self.counter * 0.01  # Convert counter to seconds

        positions = []
        velocities = []
        efforts = []

        for i, joint_name in enumerate(self.joint_names):
            # Get joint limits
            limit_min, limit_max = self.joint_limits[joint_name]
            limit_range = limit_max - limit_min
            limit_mid = (limit_min + limit_max) / 2.0

            # Simulate motion: oscillate within limits
            # Different frequencies for different joints to create realistic motion
            if 'shoulder' in joint_name:
                # Shoulders move slowly
                frequency = 0.5  # 0.5 Hz (1 complete cycle per 2 seconds)
                amplitude = limit_range * 0.1  # 10% of range
            elif 'elbow' in joint_name:
                # Elbows move with arm motion
                frequency = 0.7
                amplitude = limit_range * 0.15
            else:
                # Wrists have more subtle motion
                frequency = 1.0
                amplitude = limit_range * 0.05

            # Calculate position: rest + oscillation
            position = limit_mid + amplitude * math.sin(2 * math.pi * frequency * time_sec)

            # Calculate velocity (derivative of position)
            velocity = (amplitude * 2 * math.pi * frequency *
                       math.cos(2 * math.pi * frequency * time_sec))

            # Effort (torque) is proportional to velocity in this simple model
            # Real robots would have complex dynamics
            effort = 0.5 * velocity

            positions.append(position)
            velocities.append(velocity)
            efforts.append(effort)

        # Populate message
        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts

        # Publish the message
        self.publisher_.publish(msg)
        self.counter += 1

        # Log statistics every 100 messages (every 1 second at 100 Hz)
        if self.counter % 100 == 0:
            self.get_logger().info(
                f'Published {self.counter} joint state messages '
                f'(frequency: 100 Hz)'
            )

            # Show sample joint data
            sample_idx = 0  # Show first joint
            self.get_logger().debug(
                f'  {self.joint_names[sample_idx]}: '
                f'pos={positions[sample_idx]:.3f} rad, '
                f'vel={velocities[sample_idx]:.3f} rad/s'
            )


def main(args=None):
    """Entry point for the joint state publisher node."""
    rclpy.init(args=args)

    publisher = HumanoidJointStatePublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
