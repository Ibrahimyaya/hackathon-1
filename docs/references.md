---
sidebar_position: 203
---

# References

This page lists all official documentation cited throughout the book.

## Core ROS 2 Documentation

### Foundation
- **ROS 2 Documentation (Humble)**: https://docs.ros.org/en/humble/
- **ROS 2 Concepts**: https://docs.ros.org/en/humble/Concepts.html
- **ROS 2 Installation (Ubuntu)**: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### Nodes and Executors
- **About Nodes**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Nodes.html
- **About Executors**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html
- **Node Lifecycle**: https://design.ros2.org/articles/node_lifecycle.html

### Communication Patterns
- **About Topics**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Topics.html
- **About Services**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Services.html
- **About Actions**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Actions.html
- **Quality of Service**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service.html

### Python (rclpy)
- **rclpy Library**: https://docs.ros2.org/latest/api/rclpy/
- **rclpy Node Class**: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node
- **rclpy Publisher**: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_publisher
- **rclpy Subscription**: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_subscription
- **rclpy Service Server**: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_service
- **rclpy Service Client**: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_client
- **rclpy Timer**: https://docs.ros2.org/latest/api/rclpy/api/node.html#rclpy.node.Node.create_timer

## Message Types

### Standard Messages
- **sensor_msgs/JointState**: https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html
- **geometry_msgs/Pose**: https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html
- **geometry_msgs/Twist**: https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html
- **sensor_msgs/Imu**: https://docs.ros2.org/latest/api/sensor_msgs/msg/Imu.html
- **std_msgs/Float64**: https://docs.ros2.org/latest/api/std_msgs/msg/Float64.html

## DDS (Data Distribution Service)

### Specification
- **OMG DDS Standard**: https://www.omg.org/spec/DDS/
- **DDS Documentation**: https://github.com/OMG-DDS/dds-rtps

### DDS Implementations
- **Fast DDS (Default)**: https://fast-dds.docs.eprosima.com/
- **Cyclone DDS**: https://cyclonedds.io/
- **Connext DDS**: https://www.rti.com/products/real-time-dds-middleware

## Visualization and Simulation

### RViz2
- **RViz2 User Guide**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-RViz2.html
- **RViz2 Documentation**: https://github.com/ros2/rviz

### Gazebo
- **Gazebo Main Site**: https://gazebosim.org/
- **Gazebo + ROS 2 Integration**: https://gazebosim.org/docs/latest/ros2/
- **Gazebo Tutorials**: https://gazebosim.org/docs/latest/tutorials/

## Robot Description

### URDF
- **URDF XML Format**: https://wiki.ros.org/urdf/XML
- **Official URDF Documentation**: https://docs.ros.org/en/humble/Concepts/Intermediate/URDF/URDF-Main.html
- **URDF Examples**: https://github.com/ros/urdf_tutorials

### SDFormat (Alternative)
- **SDFormat Specification**: http://sdformat.org/

## Ubuntu and Linux

- **Ubuntu 22.04 LTS**: https://releases.ubuntu.com/jammy/
- **Ubuntu Package Manager (apt)**: https://ubuntu.com/server/docs/package-management

## Python

- **Python Official Docs**: https://docs.python.org/3/
- **Python 3.10+**: https://docs.python.org/3.10/

## Related Learning Resources

- **ROS 2 Discourse (Q&A)**: https://discourse.ros.org/
- **ROS Index (Package Browser)**: https://index.ros.org/
- **ROS 2 GitHub Organization**: https://github.com/ros2
- **ROS 2 Humble Release Notes**: https://docs.ros.org/en/humble/Releases/Humble-Hawksbill-Release-Notes.html

## Book Structure

This book is structured around three parts:

1. **Part 1: Foundations** - ROS 2 concepts, DDS, pub-sub architecture
2. **Part 2: Communication Patterns** - Nodes, Topics, Services, Actions
3. **Part 3: Robot Structure** - URDF, visualization, simulation

Each part builds on prior knowledge. Start with Part 1 and progress sequentially for best learning.

---

**All documentation links are current as of ROS 2 Humble (2022.12 release).**

See [Known Issues](./known-issues.md) for troubleshooting, or [Glossary](./glossary.md) for term definitions.
