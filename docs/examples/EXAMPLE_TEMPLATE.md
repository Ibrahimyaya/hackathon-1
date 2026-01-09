# Code Example Template

All examples in this book follow a consistent structure to ensure clarity, reproducibility, and completeness.

## Template Structure

### 1. Metadata Block (YAML)

```yaml
---
title: "Example Title"
chapter: 1  # or 2, or 3
concepts: ["concept1", "concept2"]  # Topics, Services, URDF, etc.
difficulty: "beginner" | "intermediate" | "advanced"
time_estimate: "5-10 minutes"
files:
  - "filename.py"
  - "filename.urdf"
dependencies: ["package1", "package2"]
---
```

### 2. Description (2-3 sentences)

Clear, concise explanation of what this example demonstrates and why it matters.

Example:
> This example demonstrates a basic ROS 2 publisher node that streams sensor data (joint states) to a ROS 2 topic at 100 Hz. Understanding pub-sub communication is foundational to all distributed ROS 2 systems.

### 3. Concepts Covered

- **Key Concept 1**: Brief explanation
- **Key Concept 2**: Brief explanation

### 4. Files Included

List all files needed to run the example:

```
minimal-publisher.py       Main Python script
README.md                  This example's instructions
```

### 5. Complete Code

Full, runnable code with line numbers and explanatory comments.

**Important**: Include ALL imports, setup, and boilerplate. Readers should be able to copy-paste and run immediately without guessing what's missing.

```python
#!/usr/bin/env python3
# Filename: minimal-publisher.py
# Description: Minimal ROS 2 publisher demonstrating pub-sub communication

import rclpy  # ROS 2 Python library
from rclpy.node import Node
from sensor_msgs.msg import JointState  # Standard ROS 2 message type

class MinimalPublisher(Node):
    """A simple ROS 2 node that publishes joint state data."""

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create a publisher that sends JointState messages to the '/joint_states' topic
        # qos_profile_sensor_data: Quality of Service for sensor data (reliable, best-effort)
        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_states',
            10  # Queue size for buffering messages
        )

        # Timer: call publish_message every 0.01 seconds (100 Hz)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.publish_message)

        self.i = 0  # Message counter

    def publish_message(self):
        """Publish joint state data at every timer tick."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [float(self.i), float(self.i + 1), float(self.i + 2)]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')

        self.i += 1


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)  # Initialize ROS 2
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)  # Keep node running
    minimal_publisher.destroy_node()
    rclpy.shutdown()  # Cleanup


if __name__ == '__main__':
    main()
```

### 6. Expected Output

Show exactly what the output should look like when the example runs correctly:

```
[INFO] [minimal_publisher]: Publishing joint states: [0.0, 1.0, 2.0]
[INFO] [minimal_publisher]: Publishing joint states: [1.0, 2.0, 3.0]
[INFO] [minimal_publisher]: Publishing joint states: [2.0, 3.0, 4.0]
...
```

### 7. How to Run

Step-by-step instructions:

1. **Open a terminal** and navigate to the example directory:
   ```bash
   cd docs/examples/ch1-dds-pubsub/
   ```

2. **Make the script executable** (one-time):
   ```bash
   chmod +x minimal-publisher.py
   ```

3. **Run the example**:
   ```bash
   python3 minimal-publisher.py
   ```

4. **Observe the output**: You should see "Publishing..." messages at ~100 Hz

5. **Stop the node**: Press Ctrl+C to exit

### 8. Try This

Variations for readers to experiment with:

- Change the publish frequency (e.g., `timer_period = 0.1` for 10 Hz)
- Add more joint names to the message
- Modify the position values to see them change
- Create a subscriber node to listen to the published messages

### 9. Explanation

Detailed explanation of what the code does, broken into logical sections:

#### Imports and Setup
```
rclpy: ROS 2 Python library for creating nodes and publishing
Node: Base class for ROS 2 nodes
JointState: Message type for joint sensor data (position, velocity, effort)
```

#### Class Definition
```
MinimalPublisher: A ROS 2 node that publishes joint state data
- __init__: Initialize the publisher and timer
- publish_message: Called every 10ms (100 Hz) to send data
```

#### Publisher Creation
```python
self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
```

This line creates a publisher that:
- Sends **JointState** messages (position, velocity, effort for joints)
- To the **'/joint_states'** topic (standard ROS 2 topic for joint data)
- With a queue size of **10** (buffer up to 10 messages if subscriber is slow)

#### Timer Setup
```python
self.timer = self.create_timer(0.01, self.publish_message)
```

This creates a timer that calls `publish_message()` every 0.01 seconds = **100 Hz**.

Why 100 Hz? For humanoid robots, sensor data (IMU, joint encoders) typically comes at 50-100 Hz. This example matches realistic sensor frequencies.

#### Message Creation
```python
msg = JointState()
msg.name = ['joint1', 'joint2', 'joint3']
msg.position = [0.0, 1.0, 2.0]
```

JointState is a standard ROS 2 message with:
- **name**: Names of the joints (strings)
- **position**: Current angle (radians)
- **velocity**: Rate of change (rad/s)
- **effort**: Force/torque being applied (Nm)

### 10. Key Concepts Reinforced

- **ROS 2 Node**: An executable process that acts as publisher/subscriber
- **Publisher**: Sends messages to a topic without waiting for response
- **Topic**: Named channel where messages are published and subscribed
- **Message Type**: Standardized data structure (JointState, Pose, etc.)
- **Timer**: Mechanism to execute code at regular intervals (e.g., 100 Hz)

### 11. Common Errors and Fixes

| Error | Cause | Solution |
|-------|-------|----------|
| `ModuleNotFoundError: No module named 'rclpy'` | ROS 2 not installed or not sourced | Run: `source /opt/ros/humble/setup.bash` |
| `AttributeError: 'MinimalPublisher' object has no attribute 'publisher_'` | Typo in attribute name | Ensure `self.publisher_` is spelled correctly |
| `Topic '/joint_states' has no subscribers` (warning) | Normal; no subscriber connected | This is expected; warnings do not prevent execution |
| Output stops after a few messages | Exception occurred | Scroll up in terminal to see full error |

### 12. Citations and References

Link to official documentation that explains concepts used:

- **JointState Message**: https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html
- **rclpy Publisher**: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_publisher
- **Timer in ROS 2**: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_timer
- **ROS 2 Executors (spinning)**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html

### 13. Next Steps

Suggest where readers should go next:

> **You've learned pub-sub communication!** Next, try:
> - [Minimal Subscriber](./minimal-subscriber.py) to receive these messages
> - [QoS Settings](./qos-settings.py) to understand reliability trade-offs
> - [Chapter 2](../part2-communication/04-nodes-and-lifecycle.md) for more complex patterns

---

## Formatting Guidelines

- **Code blocks**: Use triple backticks with language specifier (python, bash, yaml, xml)
- **Inline code**: Use single backticks for variable names, filenames, terminal commands
- **Paths**: Use absolute paths or relative from book root (e.g., `docs/examples/...`)
- **Line length**: Keep to ~100 characters for code examples (readable in terminal)
- **Comments**: Explain "why" not "what"; code is self-explanatory
- **Reproducibility**: Assume reader has fresh Ubuntu 22.04 + ROS 2 Humble; include all setup
- **Official sources**: Cite ROS 2 official docs, not blog posts or StackOverflow

---

## Checklist for Every Example

Before submitting an example, verify:

- [ ] Code is syntactically correct (runs without errors)
- [ ] All imports are included (no "guess what's missing")
- [ ] Output section shows exactly what readers will see
- [ ] Instructions are step-by-step (beginner can follow)
- [ ] Comments explain non-obvious behavior (e.g., why QoS is RELIABLE)
- [ ] Citations link to official ROS 2 documentation
- [ ] Example has been tested on clean Ubuntu 22.04 + ROS 2 Humble
- [ ] Filename is descriptive (e.g., `minimal-publisher.py` not `example.py`)
- [ ] README explains every step to run the example
- [ ] "Try This" section gives readers something to experiment with
- [ ] No hardcoded paths; use relative paths or assume running from example directory

---

## Example File Structure

```
docs/examples/ch1-dds-pubsub/
├── minimal-publisher.py         # Main example code
├── minimal-subscriber.py        # Related example
├── qos-settings.py             # Advanced example
├── README.md                   # Overview of all examples
└── expected-output.txt         # Reference output for comparison
```

---

This template ensures all examples in the book are clear, complete, and reproducible.
