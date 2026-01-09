#!/usr/bin/env python3
"""
DDS Quality of Service (QoS) Settings Example

This example demonstrates different QoS configurations:
- BEST_EFFORT vs. RELIABLE reliability
- KEEP_LAST(N) history settings
- Impact on message delivery and network load

Purpose: Show how QoS choices affect pub-sub behavior in humanoid robots

Reference: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service.html
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class QoSPublisher(Node):
    """Publisher demonstrating different QoS profiles."""

    def __init__(self, qos_profile, topic_name, node_name):
        """Initialize with specified QoS profile."""
        super().__init__(node_name)

        # Create publisher with specified QoS
        self.publisher_ = self.create_publisher(
            JointState,
            topic_name,
            qos_profile
        )

        # Publish at 100 Hz (typical for robot sensors)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        self.topic_name = topic_name
        self.qos_name = self._get_qos_name(qos_profile)

    def _get_qos_name(self, profile):
        """Get human-readable QoS description."""
        rel = 'BEST_EFFORT' if profile.reliability == ReliabilityPolicy.BEST_EFFORT else 'RELIABLE'
        hist = f'KEEP_LAST({profile.history._depth})' if hasattr(profile.history, '_depth') else 'KEEP_ALL'
        return f'{rel}+{hist}'

    def timer_callback(self):
        """Publish a message with current counter value."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['sensor_1']
        msg.position = [float(self.counter) / 100.0]  # 0.0 to 10.0 over 100 messages
        msg.velocity = [0.0]
        msg.effort = [0.0]

        self.publisher_.publish(msg)
        self.counter += 1

        if self.counter % 100 == 0:
            self.get_logger().info(
                f'{self.topic_name} ({self.qos_name}): '
                f'Published {self.counter} messages'
            )


class QoSSubscriber(Node):
    """Subscriber demonstrating how QoS affects message reception."""

    def __init__(self, qos_profile, topic_name, node_name):
        """Initialize with specified QoS profile."""
        super().__init__(node_name)

        # Create subscription with specified QoS
        self.subscription = self.create_subscription(
            JointState,
            topic_name,
            self.listener_callback,
            qos_profile
        )

        self.message_count = 0
        self.last_position = None
        self.topic_name = topic_name
        self.qos_name = self._get_qos_name(qos_profile)

    def _get_qos_name(self, profile):
        """Get human-readable QoS description."""
        rel = 'BEST_EFFORT' if profile.reliability == ReliabilityPolicy.BEST_EFFORT else 'RELIABLE'
        hist = f'KEEP_LAST({profile.history._depth})' if hasattr(profile.history, '_depth') else 'KEEP_ALL'
        return f'{rel}+{hist}'

    def listener_callback(self, msg):
        """Receive and log messages."""
        self.message_count += 1
        current_position = msg.position[0]

        # Detect gaps (messages were dropped)
        if self.last_position is not None:
            gap = current_position - self.last_position
            if abs(gap) > 0.02:  # More than 2 message gap expected
                self.get_logger().warn(
                    f'Detected gap! Previous: {self.last_position:.2f}, '
                    f'Current: {current_position:.2f}'
                )

        self.last_position = current_position

        if self.message_count % 100 == 0:
            self.get_logger().info(
                f'{self.topic_name} ({self.qos_name}): '
                f'Received {self.message_count} messages'
            )


def main(args=None):
    """
    Demonstrate different QoS configurations.

    Run this example in multiple terminals:
    Terminal 1: ros2 run <package> qos-settings.py publisher-reliable
    Terminal 2: ros2 run <package> qos-settings.py publisher-best-effort
    Terminal 3: ros2 run <package> qos-settings.py subscriber-reliable
    Terminal 4: ros2 run <package> qos-settings.py subscriber-best-effort
    """
    rclpy.init(args=args)

    # Define two QoS profiles
    # Profile 1: RELIABLE delivery (guarantees all messages arrive)
    reliable_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST(1),
        depth=1
    )

    # Profile 2: BEST_EFFORT delivery (fast, may drop messages)
    best_effort_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST(1),
        depth=1
    )

    # Create nodes based on command line argument
    # (In real usage, you'd parse sys.argv or ROS 2 parameters)
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else 'all'

    nodes = []

    if mode in ['all', 'publisher-reliable']:
        pub_rel = QoSPublisher(
            reliable_qos,
            '/joint_states_reliable',
            'qos_publisher_reliable'
        )
        nodes.append(pub_rel)

    if mode in ['all', 'publisher-best-effort']:
        pub_be = QoSPublisher(
            best_effort_qos,
            '/joint_states_best_effort',
            'qos_publisher_best_effort'
        )
        nodes.append(pub_be)

    if mode in ['all', 'subscriber-reliable']:
        sub_rel = QoSSubscriber(
            reliable_qos,
            '/joint_states_reliable',
            'qos_subscriber_reliable'
        )
        nodes.append(sub_rel)

    if mode in ['all', 'subscriber-best-effort']:
        sub_be = QoSSubscriber(
            best_effort_qos,
            '/joint_states_best_effort',
            'qos_subscriber_best_effort'
        )
        nodes.append(sub_be)

    # Run all nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        print(f'Running {len(nodes)} node(s)...')
        executor.spin()
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
