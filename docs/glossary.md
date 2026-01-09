---
sidebar_position: 202
---

# Glossary

This glossary defines key terms used throughout the book.

## ROS 2 Concepts

### Action
A ROS 2 communication pattern for long-running asynchronous tasks. A client sends a goal, the server executes the goal and provides periodic feedback, and finally returns a result. Used for tasks like "move to pose" or "reach target."

**Example**: Plan sends goal → Controller provides feedback (progress) → Controller returns result (success/failure)

### DDS (Data Distribution Service)
The underlying middleware protocol that powers ROS 2 communication. An open standard from the OMG that provides publish-subscribe, quality of service, and network abstraction.

### Executor
The ROS 2 event loop that processes callbacks from subscriptions, timers, and services. In rclpy, `rclpy.spin()` starts the executor.

### Message
A serializable data structure sent over ROS 2 topics. Examples: `JointState`, `Pose`, `Image`, `Twist`.

### Middleware
Software that sits between applications and hardware/network, abstracting low-level details. ROS 2 is middleware for robotics.

### Node
An independent ROS 2 process that publishes/subscribes to topics, offers/calls services, or offers/sends actions. Named and reusable across different systems.

### Publish-Subscribe (Pub-Sub)
An asynchronous communication pattern where publishers send messages to topics and subscribers receive them. Publishers and subscribers are decoupled (don't know about each other).

### QoS (Quality of Service)
Settings that control message delivery behavior:
- **Reliability**: RELIABLE (guaranteed delivery) or BEST_EFFORT (best effort, may drop)
- **History**: Keep last N messages or all messages
- **Deadline**: Messages must arrive within X milliseconds
- **Liveliness**: How often a node must publish to be considered "alive"

### Service
A synchronous request-reply communication pattern. A client sends a request and waits for a response from a server.

**Example**: Client requests "set motor speed to 50 RPM" → Server sets motor → Server responds "success"

### Topic
A named channel for publish-subscribe communication. Publishers send messages to topics; subscribers receive from topics.

**Example Topic**: `/joint_states` (standard topic for joint sensor data in humanoid robots)

## Robotics Concepts

### Actuator
A motor or device that produces motion. In humanoid robots, actuators move joints (shoulders, elbows, hips, knees).

### Encoder
A sensor that measures joint angle or position. Used to provide feedback on robot state.

### Humanoid Robot
A robot designed to resemble human form and movement. Typically with:
- 2 arms (with shoulders, elbows, wrists)
- 2 legs (with hips, knees, ankles)
- 1 torso
- 1 head

### IMU (Inertial Measurement Unit)
A sensor that measures acceleration, rotational velocity, and magnetic field. Used for:
- Detecting orientation (which way is "up")
- Detecting motion and vibration
- Balance feedback in humanoid robots

### Joint
A connection between two robot parts (links) that allows relative movement. Can be:
- **Revolute**: Rotational motion (like an elbow)
- **Prismatic**: Linear motion (like a sliding door)

### Kinematics
The study of motion without considering forces. Forward kinematics: given joint angles, what is the end-effector position? Inverse kinematics: given desired end-effector position, what joint angles are needed?

### Link
A rigid body (unchanging shape/size) in a robot structure. In humanoid robots: upper arm, forearm, torso, leg, etc.

### URDF (Unified Robot Description Format)
An XML file format that describes robot structure:
- Links (rigid bodies)
- Joints (connections)
- Inertia (mass, moment of inertia)
- Sensors and actuators
Used by ROS 2 tools (RViz, Gazebo, planners) to understand robot structure.

## Tools

### Gazebo
A physics simulation engine that simulates robot dynamics, sensor behavior, and physics constraints. Integrates with ROS 2 for sim-to-real validation.

### rclpy
The ROS 2 Python client library. Used to write ROS 2 nodes in Python.

**Import**: `import rclpy`

### ROS 2 Humble
The Long-Term Support (LTS) release of ROS 2 used in this book. Supported until May 2027.

### RViz (RViz2)
ROS 2's 3D visualization tool. Displays:
- Robot structure (URDF)
- Sensor data overlays (Point clouds, arrows, text)
- Transforms between coordinate frames

## General Programming Terms

### Callback
A function that is called automatically when an event occurs. In ROS 2:
- Subscription callback: called when a topic message arrives
- Timer callback: called when a timer fires
- Service callback: called when a service request arrives

### Decoupling
Reducing dependencies between components. In ROS 2, publishers and subscribers are decoupled—they don't know about each other, enabling independent development and testing.

### Event Loop
A program that continuously waits for and processes events (messages, timers, etc.). `rclpy.spin()` starts the event loop.

### Loop Rate / Frequency
How often something happens:
- **10 Hz**: Every 0.1 seconds (10 times per second)
- **100 Hz**: Every 0.01 seconds (100 times per second)
- Used for publishing sensor data, control loops, etc.

### Logging
Writing informational messages, warnings, or errors to output. In ROS 2:
- `self.get_logger().info("message")`
- `self.get_logger().warn("warning")`
- `self.get_logger().error("error")`

### Message Format
The structure of data sent over ROS 2 topics. Defined in `.msg` files. Examples: `sensor_msgs/JointState`, `geometry_msgs/Twist`.

## Abbreviations

- **API**: Application Programming Interface (functions/methods available to use)
- **DDS**: Data Distribution Service
- **Hz**: Hertz (cycles per second)
- **IMU**: Inertial Measurement Unit
- **LTS**: Long-Term Support
- **ms**: milliseconds (0.001 seconds)
- **QoS**: Quality of Service
- **URDF**: Unified Robot Description Format
- **XML**: Extensible Markup Language (text format for data)

---

**Need help?** Check the [References](./references.md) for links to official documentation.
