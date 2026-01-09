---
sidebar_position: 200
---

# Quickstart: Your First ROS 2 Program (30 minutes)

In this quickstart, you'll write and run your first ROS 2 node—a simple publisher that sends sensor data. This will take about 30 minutes.

## Prerequisites

- [Setup Guide](./setup-guide.md) completed (ROS 2 Humble installed)
- Terminal access
- Any text editor (nano, vim, VS Code, etc.)

## Step 1: Create a Python File (2 minutes)

Open a terminal and create a new file:

```bash
mkdir -p ~/ros2_examples
cd ~/ros2_examples
nano minimal_publisher.py
```

## Step 2: Write the Publisher Code (5 minutes)

Copy this code into your editor:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # 10 Hz (every 0.1 seconds)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [float(self.i), float(self.i+1), float(self.i+2)]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: positions={msg.position}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Save and exit (Ctrl+X, Y, Enter in nano).

## Step 3: Make It Executable (1 minute)

```bash
chmod +x minimal_publisher.py
```

## Step 4: Run Your First ROS 2 Program (3 minutes)

```bash
python3 minimal_publisher.py
```

You should see output like:

```
[INFO] [minimal_publisher]: Published: positions=[0.0, 1.0, 2.0]
[INFO] [minimal_publisher]: Published: positions=[1.0, 2.0, 3.0]
[INFO] [minimal_publisher]: Published: positions=[2.0, 3.0, 4.0]
...
```

Press Ctrl+C to stop.

## Step 5: Listen to Your Messages (5 minutes)

Open a **second terminal** and run a subscriber:

```bash
ros2 topic echo /joint_states
```

Go back to the first terminal and run your publisher again:

```bash
python3 minimal_publisher.py
```

In the second terminal, you'll see:

```
header:
  stamp:
    sec: 1704706800
    nsec: 12345678
  frame_id: ''
name:
- joint1
- joint2
- joint3
position:
- 0.0
- 1.0
- 2.0
velocity:
- 0.0
- 0.0
- 0.0
effort:
- 0.0
- 0.0
- 0.0
---
...
```

## Step 6: Verify Frequency (5 minutes)

In the second terminal, measure message frequency:

```bash
ros2 topic hz /joint_states
```

You should see: `average rate: 10.00` (confirming 10 Hz publication)

## Congratulations! 🎉

You've successfully:
- ✅ Written a ROS 2 publisher node in Python
- ✅ Published sensor data to a ROS 2 topic
- ✅ Listened to messages from another terminal
- ✅ Verified message frequency

## What You Just Learned

- **Node**: Your program is a ROS 2 node (it imports `Node` and extends it)
- **Publisher**: You created a publisher that sends `JointState` messages
- **Topic**: Messages go to the `/joint_states` topic
- **Decoupling**: The publisher doesn't know or care if anyone is listening
- **Timer**: The timer causes your code to run at regular intervals (10 Hz)

## Next Steps

Ready to learn more? Pick your path:

1. **[Understand the Code](./part1-foundations/01-ros2-overview.md)** - Learn ROS 2 concepts in depth
2. **[Try More Patterns](./part2-communication/04-nodes-and-lifecycle.md)** - Learn Services and Actions
3. **[Build a Robot](./part3-robot-structure/09-urdf-fundamentals.md)** - Write URDF and simulate

## Variations to Try

Now that your program works, try modifying it:

1. **Change frequency**: Set `timer_period = 0.01` for 100 Hz (matching real sensor rates)
2. **Add more joints**: Extend `msg.name` and `msg.position` lists
3. **Publish different data**: Change the position values to something interesting
4. **Create a subscriber**: Write a node that listens to `/joint_states` and prints positions

---

**Having trouble?** See the [Setup Guide](./setup-guide.md) or [Known Issues](./known-issues.md).
