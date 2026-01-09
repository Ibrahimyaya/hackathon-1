#!/usr/bin/env python3
"""
Minimal ROS 2 Subscriber Example

This example demonstrates the simplest possible ROS 2 subscriber:
- Creates a node
- Creates a subscription to a topic
- Receives and prints messages as they arrive
- Measures message arrival rate

Purpose: Show how to receive messages from a publisher
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class MinimalSubscriber(Node):
    """A simple subscriber node that receives joint state data."""

    def __init__(self):
        """Initialize the node and create a subscription."""
        # Initialize the ROS 2 node with name 'minimal_subscriber'
        super().__init__('minimal_subscriber')

        # Create a subscription to the '/joint_states' topic
        # Arguments:
        #   1. Message type: sensor_msgs.msg.JointState
        #   2. Topic name: '/joint_states' (must match publisher topic)
        #   3. Callback function: called when a message arrives
        #   4. Queue size: 10 (keep 10 messages in history)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10
        )

        # Track message statistics
        self.message_count = 0
        self.start_time = time.time()

    def listener_callback(self, msg):
        """Called whenever a message arrives on the subscribed topic."""
        self.message_count += 1

        # Extract data from the message
        joint_names = msg.name
        joint_positions = msg.position

        # Log every 10th message
        if self.message_count % 10 == 0:
            elapsed = time.time() - self.start_time
            frequency = self.message_count / elapsed
            self.get_logger().info(
                f'Received {self.message_count} messages '
                f'(frequency: {frequency:.1f} Hz) - '
                f'Joints: {joint_names}'
            )


def main(args=None):
    """Entry point for the ROS 2 node."""
    # Initialize the ROS 2 system
    rclpy.init(args=args)

    # Create the subscriber node
    minimal_subscriber = MinimalSubscriber()

    # Spin the node (start the event loop)
    # This keeps the node running, processing messages as they arrive
    try:
        print('Waiting for messages on /joint_states topic...')
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print(f'\nReceived {minimal_subscriber.message_count} total messages')
        print('Shutting down...')
    finally:
        # Clean up resources
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
