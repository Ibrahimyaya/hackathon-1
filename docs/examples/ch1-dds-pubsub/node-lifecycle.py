#!/usr/bin/env python3
"""
ROS 2 Node Lifecycle Example

This example demonstrates the node lifecycle phases:
- Initialization: Create node and resources
- Spinning: Event loop processes callbacks
- Shutdown: Clean up and gracefully exit

Purpose: Show how ROS 2 nodes are structured and managed

Reference: https://design.ros2.org/articles/node_lifecycle.html
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class LifecycleExampleNode(Node):
    """Node demonstrating lifecycle phases with clear logging."""

    def __init__(self):
        """Initialization phase."""
        # Phase 1: Initialization - Node creation and resource setup
        print('>>> PHASE 1: INITIALIZATION')
        print('    Creating node...')
        super().__init__('lifecycle_example_node')

        print('    Creating publisher...')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        print('    Creating subscription...')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )

        print('    Creating timer...')
        self.timer = self.create_timer(0.1, self.timer_callback)

        # State tracking
        self.publish_count = 0
        self.receive_count = 0
        self.start_time = time.time()

        print('>>> INITIALIZATION COMPLETE\n')

    def timer_callback(self):
        """Called periodically during Phase 2 (Spinning)."""
        # Phase 2: Spinning - Event loop is active
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1']
        msg.position = [0.5]
        msg.velocity = [0.0]
        msg.effort = [0.0]

        self.publisher_.publish(msg)
        self.publish_count += 1

    def listener_callback(self, msg):
        """Receive callback during Phase 2 (Spinning)."""
        self.receive_count += 1

    def shutdown(self):
        """Phase 3: Shutdown - Clean up resources."""
        elapsed = time.time() - self.start_time
        print('\n>>> PHASE 3: SHUTDOWN')
        print(f'    Node ran for {elapsed:.2f} seconds')
        print(f'    Published: {self.publish_count} messages')
        print(f'    Received: {self.receive_count} messages')
        print('    Destroying node...')
        self.destroy_node()
        print('>>> SHUTDOWN COMPLETE')


def main(args=None):
    """
    Main entry point demonstrating all lifecycle phases.
    """
    print('=== ROS 2 Node Lifecycle Example ===\n')

    # Initialize ROS 2 system
    print('Initializing ROS 2...')
    rclpy.init(args=args)
    print()

    # Create node (Initialization phase begins)
    node = LifecycleExampleNode()

    # Spin the node (Spinning phase)
    print('>>> PHASE 2: SPINNING')
    print('    Event loop is running...')
    print('    Press Ctrl+C to proceed to shutdown phase\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Shutdown phase begins
        pass

    # Shutdown (Phase 3)
    node.shutdown()
    rclpy.shutdown()
    print('\n=== Lifecycle Example Complete ===')


if __name__ == '__main__':
    main()
