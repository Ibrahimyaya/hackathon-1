---
sidebar_position: 4
title: ROS 2 Nodes and Lifecycle
---

# ROS 2 Nodes and Their Lifecycle

> **Learning Goal**: Understand ROS 2 nodes as independent executable processes and their lifecycle (initialization, spinning, shutdown).

## What is a Node?

A **ROS 2 node** is an independent executable process that:
- Is initialized with a unique name (e.g., "motor_controller", "sensor_driver")
- Creates publishers, subscribers, services, and actions
- Runs an event loop ("spinning") to receive and send messages
- Gracefully shuts down when finished

## Node Lifecycle

Every ROS 2 node goes through a lifecycle:

```
1. Initialize
   ↓
2. Create publishers, subscribers, services
   ↓
3. Spin (event loop: listen for messages, publish data)
   ↓
4. Shutdown (cleanup, destroy node)
```

*This section is an outline. Full content to be added: Node implementation in rclpy, lifecycle states, executor concepts, example code.*

---

See: [Official Node Documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Nodes.html)
