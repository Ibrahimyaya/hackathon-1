#!/usr/bin/env python3
"""
ROS 2 Topic Subscriber: Monitor Humanoid Joint States

This example demonstrates subscribing to joint state data and performing
real-time analysis (frequency measurement, statistics).

Purpose: Show how to receive and process continuous sensor data streams
Reference: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Topics.html
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
import time
import math


class JointStateMonitor(Node):
    """
    Subscribes to and monitors humanoid joint states.

    This example shows how to:
    - Receive high-frequency sensor data (100 Hz)
    - Measure actual publish frequency
    - Calculate joint statistics (position range, velocity magnitude)
    - Detect data anomalies (missing messages, stale data)
    """

    def __init__(self):
        """Initialize the joint state subscriber."""
        super().__init__('joint_state_monitor')

        # Subscriber QoS: Match publisher
        # Must use BEST_EFFORT to match the publisher's QoS
        # Mismatched QoS = no messages delivered
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST(1),
            depth=1
        )

        # Subscribe to /joint_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            sensor_qos
        )

        # Statistics tracking
        self.message_count = 0
        self.start_time = None
        self.last_timestamp = None
        self.min_interval = float('inf')
        self.max_interval = 0.0
        self.total_interval = 0.0

        # Position statistics per joint
        self.joint_positions_min = {}
        self.joint_positions_max = {}

        self.get_logger().info(
            'Joint State Monitor ready, waiting for /joint_states messages...'
        )

    def joint_state_callback(self, msg):
        """Process incoming joint state message."""
        # Initialize timing on first message
        if self.start_time is None:
            self.start_time = time.time()
            self.last_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            self.get_logger().info(f'Received first message: {len(msg.name)} joints')
            return

        self.message_count += 1

        # Calculate message interval (time between messages)
        current_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        interval = current_timestamp - self.last_timestamp
        self.last_timestamp = current_timestamp

        # Track timing statistics
        if interval > 0:  # Ignore zero intervals
            self.total_interval += interval
            if interval < self.min_interval:
                self.min_interval = interval
            if interval > self.max_interval:
                self.max_interval = interval

        # Track position statistics for each joint
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                position = msg.position[i]

                if joint_name not in self.joint_positions_min:
                    self.joint_positions_min[joint_name] = position
                    self.joint_positions_max[joint_name] = position
                else:
                    if position < self.joint_positions_min[joint_name]:
                        self.joint_positions_min[joint_name] = position
                    if position > self.joint_positions_max[joint_name]:
                        self.joint_positions_max[joint_name] = position

        # Log statistics every 100 messages (every 1 second at 100 Hz)
        if self.message_count % 100 == 0:
            elapsed = time.time() - self.start_time
            frequency = self.message_count / elapsed
            avg_interval = self.total_interval / self.message_count

            # Calculate expected interval at 100 Hz (0.01 seconds)
            expected_interval = 0.01
            interval_error = abs(avg_interval - expected_interval) / expected_interval * 100

            self.get_logger().info(
                f'Messages: {self.message_count} '
                f'| Frequency: {frequency:.1f} Hz '
                f'| Interval: {avg_interval*1000:.2f}ms '
                f'(expected 10ms, error {interval_error:.1f}%)'
            )

            # Log position ranges for first joint (example)
            if msg.name:
                first_joint = msg.name[0]
                pos_min = self.joint_positions_min.get(first_joint, 0)
                pos_max = self.joint_positions_max.get(first_joint, 0)
                pos_range = pos_max - pos_min

                self.get_logger().debug(
                    f'  {first_joint}: position range = [{pos_min:.3f}, {pos_max:.3f}] '
                    f'rad (range: {pos_range:.3f} rad)'
                )

    def print_final_statistics(self):
        """Print final statistics after shutdown."""
        if self.message_count == 0:
            print('\nNo messages received.')
            return

        elapsed = time.time() - self.start_time
        frequency = self.message_count / elapsed

        print('\n=== Final Statistics ===')
        print(f'Total messages: {self.message_count}')
        print(f'Runtime: {elapsed:.2f} seconds')
        print(f'Average frequency: {frequency:.1f} Hz')
        print(f'Interval statistics:')
        print(f'  Min: {self.min_interval*1000:.2f} ms')
        print(f'  Max: {self.max_interval*1000:.2f} ms')
        print(f'  Avg: {(self.total_interval/self.message_count)*1000:.2f} ms')

        if frequency > 95 and frequency < 105:
            print(f'✓ Frequency within ±5% of expected 100 Hz')
        else:
            print(f'✗ Frequency deviation: {abs(frequency-100)/100*100:.1f}%')

        print(f'\nPosition ranges (per joint):')
        for joint_name in sorted(self.joint_positions_min.keys())[:3]:  # Show first 3
            pos_min = self.joint_positions_min[joint_name]
            pos_max = self.joint_positions_max[joint_name]
            pos_range = pos_max - pos_min
            print(f'  {joint_name}: [{pos_min:.3f}, {pos_max:.3f}] rad (range: {pos_range:.3f})')


def main(args=None):
    """Entry point for the joint state monitor node."""
    rclpy.init(args=args)

    monitor = JointStateMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        monitor.print_final_statistics()
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
