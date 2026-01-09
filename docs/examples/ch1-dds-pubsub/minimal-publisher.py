#!/usr/bin/env python3
"""
Minimal ROS 2 Publisher Example

This example demonstrates the simplest possible ROS 2 publisher:
- Creates a node
- Creates a publisher
- Publishes dummy sensor data every 0.1 seconds (10 Hz)
- Runs indefinitely until interrupted (Ctrl+C)

Purpose: Show core ROS 2 pub-sub pattern
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MinimalPublisher(Node):
    """A simple publisher node that publishes joint state data."""

    def __init__(self):
        """Initialize the node and create a publisher."""
        # Initialize the ROS 2 node with name 'minimal_publisher'
        super().__init__('minimal_publisher')

        # Create a publisher for sensor_msgs/JointState messages
        # Arguments:
        #   1. Message type: sensor_msgs.msg.JointState
        #   2. Topic name: '/joint_states' (standard topic for robot joints)
        #   3. Queue size: 10 (keep 10 messages in history)
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Create a timer to publish every 0.1 seconds (10 Hz)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for demo output
        self.counter = 0

    def timer_callback(self):
        """Called every timer_period seconds to publish a message."""
        # Create a new JointState message
        msg = JointState()

        # Populate the message with dummy data
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']  # 3 dummy joints
        msg.position = [0.1, 0.2, 0.3]  # Current joint angles (radians)
        msg.velocity = [0.0, 0.0, 0.0]  # Joint velocities
        msg.effort = [0.0, 0.0, 0.0]    # Joint torques/forces

        # Publish the message to the '/joint_states' topic
        self.publisher_.publish(msg)

        # Log to console
        self.counter += 1
        if self.counter % 10 == 0:  # Print every 1 second
            self.get_logger().info(f'Published {self.counter} messages')


def main(args=None):
    """Entry point for the ROS 2 node."""
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create the publisher node
    minimal_publisher = MinimalPublisher()

    # Spin the node (start the event loop)
    # This keeps the node running, calling timer_callback periodically
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print('\nShutting down...')
    finally:
        # Clean up resources
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
