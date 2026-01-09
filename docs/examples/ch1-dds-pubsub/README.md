# Chapter 1: ROS 2 Fundamentals and DDS - Code Examples

This directory contains working code examples demonstrating ROS 2 pub-sub patterns, Quality of Service (QoS) settings, and node lifecycle management.

**Target**: Ubuntu 22.04 LTS with ROS 2 Humble
**Language**: Python 3.10+ with rclpy
**Duration**: 30-45 minutes to run all examples

---

## Quick Start

1. **Ensure ROS 2 is sourced** in your shell:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Run examples in separate terminals** (ROS 2 requires each node in its own terminal):
   ```bash
   # Terminal 1: Start publisher
   python3 minimal-publisher.py

   # Terminal 2: Start subscriber (will immediately see messages)
   python3 minimal-subscriber.py
   ```

3. **Stop examples** by pressing `Ctrl+C` in each terminal

---

## Examples

### 1. Minimal Publisher (`minimal-publisher.py`)

**What it demonstrates**:
- Creating a ROS 2 node
- Creating a publisher
- Publishing messages periodically

**How to run**:
```bash
python3 minimal-publisher.py
```

**Expected output**:
```
Published 10 messages
Published 20 messages
Published 30 messages
...
```

**Explanation**:
- Creates a node called `minimal_publisher`
- Creates a publisher on topic `/joint_states` (standard ROS 2 topic for robot joint data)
- Publishes `sensor_msgs/JointState` messages at 10 Hz (every 0.1 seconds)
- Uses default QoS settings (RELIABLE + KEEP_LAST(10))

**Key learning**:
This is the simplest possible ROS 2 publisher. It shows the core pattern: node → publisher → topic.

**Next**: Run the subscriber in another terminal to receive these messages.

---

### 2. Minimal Subscriber (`minimal-subscriber.py`)

**What it demonstrates**:
- Creating a ROS 2 node
- Creating a subscription to a topic
- Receiving messages as they arrive
- Measuring message frequency

**How to run** (with publisher running):
```bash
python3 minimal-subscriber.py
```

**Expected output**:
```
Waiting for messages on /joint_states topic...
Received 10 messages (frequency: 10.2 Hz) - Joints: ['joint_1', 'joint_2', 'joint_3']
Received 20 messages (frequency: 10.1 Hz) - Joints: ['joint_1', 'joint_2', 'joint_3']
...
```

**Explanation**:
- Creates a node called `minimal_subscriber`
- Subscribes to the `/joint_states` topic (same as publisher)
- Uses a callback function to process arriving messages
- Calculates actual message frequency to verify 10 Hz delivery

**Key learning**:
This shows how publishers and subscribers are **decoupled**. The subscriber doesn't know about the publisher; it just listens to the topic. You can start the subscriber before or after the publisher—it doesn't matter.

---

### 3. QoS Settings (`qos-settings.py`)

**What it demonstrates**:
- How QoS (Quality of Service) settings affect message delivery
- BEST_EFFORT vs. RELIABLE reliability
- KEEP_LAST(N) history policies
- Real-world impact for different node types

**How to run** (with different terminal arguments):
```bash
# Terminal 1: Publisher with RELIABLE QoS
python3 qos-settings.py publisher-reliable

# Terminal 2: Publisher with BEST_EFFORT QoS
python3 qos-settings.py publisher-best-effort

# Terminal 3: Subscriber for RELIABLE topic
python3 qos-settings.py subscriber-reliable

# Terminal 4: Subscriber for BEST_EFFORT topic
python3 qos-settings.py subscriber-best-effort
```

**Expected output** (from subscribers):
```
/joint_states_reliable (RELIABLE+KEEP_LAST(1)): Received 100 messages
/joint_states_best_effort (BEST_EFFORT+KEEP_LAST(1)): Received 100 messages
```

**Explanation**:
- **RELIABLE**: Guarantees every message is delivered. Slower but safer for critical data (motor commands)
- **BEST_EFFORT**: Delivers messages on a "best effort" basis. Faster and lower latency, acceptable for high-frequency sensor data where losing 1 frame of 100 is fine
- **KEEP_LAST(1)**: Only keep the most recent message. Perfect for sensor data where old data is stale

**Real-world humanoid examples**:
- **IMU at 100 Hz**: Use BEST_EFFORT + KEEP_LAST(1) (fast, fresh data)
- **Motor commands**: Use RELIABLE + KEEP_LAST(1) (ensure commands arrive)
- **Logging/diagnostics**: Can use either (not time-critical)

**Key learning**:
QoS is not one-size-fits-all. Choose settings based on your data type and requirements.

---

### 4. Node Lifecycle (`node-lifecycle.py`)

**What it demonstrates**:
- The three phases of a ROS 2 node:
  1. **Initialization**: Create resources (publishers, subscribers, timers)
  2. **Spinning**: Event loop runs, callbacks are called
  3. **Shutdown**: Clean up resources

**How to run**:
```bash
python3 node-lifecycle.py
```

**Expected output**:
```
>>> PHASE 1: INITIALIZATION
    Creating node...
    Creating publisher...
    Creating subscription...
    Creating timer...
>>> INITIALIZATION COMPLETE

>>> PHASE 2: SPINNING
    Event loop is running...
    Press Ctrl+C to proceed to shutdown phase

>>> PHASE 3: SHUTDOWN
    Node ran for XX.XX seconds
    Published: XXX messages
    Received: XXX messages
    Destroying node...
>>> SHUTDOWN COMPLETE
```

**Explanation**:
- Every ROS 2 node has three phases
- During initialization, all resources are created
- During spinning, the event loop runs, calling your callbacks (timer_callback, listener_callback, etc.)
- During shutdown, resources are cleaned up and the node exits gracefully

**Key learning**:
Understanding the lifecycle helps you debug. If your node never publishes, check: Did it reach the SPINNING phase? Is the timer callback being called?

---

### 5. Humanoid IMU Sensor Example (`humanoid-imu-example.py`)

**What it demonstrates**:
- A realistic humanoid robot architecture
- IMU sensor publishing orientation data at 100 Hz
- **Three independent subscribers** reacting to the same sensor data:
  1. **Balance Planner**: Monitors tilt angle for loss-of-balance detection
  2. **Stabilization Controller**: Real-time motor commands to maintain balance
  3. **Diagnostics Logger**: Long-term health monitoring
- The **decoupled pub-sub pattern** in action

**How to run**:
```bash
python3 humanoid-imu-example.py
```

**Expected output**:
```
=== Humanoid IMU Sensor Example ===

IMU Sensor ready on /imu/data (100 Hz)
Balance Planner ready, monitoring /imu/data
Stabilization Controller ready
Diagnostics Logging ready
All nodes running. Press Ctrl+C to stop.

[After ~1 second]
Published 100 IMU messages
Published 100 motor commands
Balance stable, tilt: -2.5°
Diagnostics: Accel=(...) Angular=(...)
```

**Explanation**:
- The IMU sensor publishes `sensor_msgs/Imu` messages at 100 Hz
- Three subscribers react **independently** and at different rates:
  - **Balance Planner**: Reads every message, checks for tilt > 10°
  - **Stabilization Controller**: Reads every message, commands motors at 100 Hz
  - **Diagnostics Logger**: Reads every message, but only logs every 10 seconds
- None of these subscribers know about each other
- If you wanted to add a 4th subscriber, you wouldn't modify the others
- This is the **power of decoupling**: subscribers are independent modules

**Why it matters for humanoids**:
Real humanoid robots have many independent control loops:
- Balance stabilization (legs)
- Arm coordination (shoulders, elbows, wrists)
- Vision processing (cameras)
- Force feedback (pressure sensors)

All of these can subscribe to the same sensors and act independently. If one node crashes, others keep running.

**Key learning**:
The pub-sub pattern enables **distributed, loosely-coupled robot control**. Each component is independent, making the system robust and maintainable.

---

## Running Multiple Examples Together

To see multiple examples interacting:

```bash
# Terminal 1: IMU sensor (publishes at 100 Hz)
python3 humanoid-imu-example.py

# Terminal 2: Monitor IMU topic
ros2 topic echo /imu/data

# Terminal 3: Measure frequency
ros2 topic hz /imu/data

# Terminal 4: Monitor motor commands (output of stabilization controller)
ros2 topic echo /motor_command_ankles
```

---

## Verification Checklist

Before considering examples complete, verify:

- [ ] All examples run without syntax errors
- [ ] Publisher output shows message count increasing
- [ ] Subscriber output shows received message count increasing
- [ ] Subscriber frequency matches publisher frequency (±5%)
- [ ] QoS example shows differences between RELIABLE and BEST_EFFORT
- [ ] Node lifecycle example shows all three phases
- [ ] Humanoid IMU example runs with 4 nodes simultaneously
- [ ] All examples cite official ROS 2 documentation
- [ ] Ctrl+C gracefully shuts down all examples

---

## Common Issues & Solutions

### Issue: "ModuleNotFoundError: No module named 'rclpy'"

**Solution**: Source ROS 2 before running:
```bash
source /opt/ros/humble/setup.bash
python3 example.py
```

### Issue: Publisher shows "Topic '/joint_states' has no subscribers"

**Solution**: This is expected! It's just a warning. Start a subscriber:
```bash
# Terminal 1
python3 minimal-publisher.py

# Terminal 2
python3 minimal-subscriber.py
```

The warning disappears once a subscriber connects.

### Issue: "Address already in use" error

**Solution**: Another ROS 2 process is using the same port. Either:
1. Stop other ROS 2 examples: `pkill -f "python3"`
2. Or use a different ROS_DOMAIN_ID: `export ROS_DOMAIN_ID=1`

### Issue: Examples freeze or hang

**Solution**: Ensure you have at least one publisher and one subscriber for pub-sub to work. If running only publisher or only subscriber, you need both for messages to flow.

---

## Understanding ROS 2 Topics with `ros2 cli`

While examples are running, inspect them:

```bash
# List all topics
ros2 topic list

# Show topic type
ros2 topic type /joint_states

# Echo messages from a topic
ros2 topic echo /joint_states

# Measure publish frequency
ros2 topic hz /joint_states

# Get topic info
ros2 topic info /joint_states
```

---

## Next Steps

After completing these examples:

1. **Modify examples**: Change message types, frequencies, QoS settings. See what breaks.
2. **Create your own publisher**: Try publishing custom data at different frequencies.
3. **Combine multiple publishers**: What happens when two nodes publish to the same topic?
4. **Chapter 2**: Learn about Services (request-reply) and Actions (long-running tasks)
5. **Chapter 3**: Control a humanoid robot URDF in RViz and Gazebo using these pub-sub patterns

---

## References

Official ROS 2 Documentation:
- **Concepts**: https://docs.ros.org/en/humble/Concepts.html
- **Topics**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Topics.html
- **QoS**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service.html
- **rclpy API**: https://docs.ros2.org/latest/api/rclpy/
- **Message Types**: https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html

---

**Book**: *ROS 2 as the Robotic Nervous System for Humanoid Robots*
**Chapter**: 1 - ROS 2 Fundamentals and DDS
**Updated**: 2026-01-08
