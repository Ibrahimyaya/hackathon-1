---
sidebar_position: 1
title: What is ROS 2?
---

# What is ROS 2?

> **Learning Goal**: Understand ROS 2 as middleware, why distributed systems matter for robotics, and how ROS 2 enables decoupled, scalable humanoid robot control.

## Overview

ROS 2 (Robot Operating System 2) is **middleware** for robotics. It sits between your robot's hardware (motors, sensors) and your application software (planning, control), enabling them to communicate reliably and efficiently.

Think of ROS 2 as the **nervous system** of your robot. Just as your body's nerves carry signals between different organs without them being directly connected, ROS 2 carries data between your robot's components without them knowing about each other.

### What is Middleware?

**Middleware** is software that:
- Sits between hardware and applications
- Handles communication between different programs
- Abstracts away low-level details (network protocols, message formats)
- Enables loose coupling (programs don't need to know about each other)

**Example**: Without middleware, your motor control code would need to know:
- How to talk to the IMU sensor driver
- What format IMU data arrives in
- When data arrives (synchronous vs. asynchronous)
- How to handle network communication

**With ROS 2**: Motor control code simply subscribes to the `/imu` topic and receives standardized messages. The sensor driver, the network, and other details are abstracted away.

## Why Distributed Systems?

### The Monolithic Approach (❌ Not Recommended)

All logic in one program:

```
┌─────────────────────────────────────────────┐
│  Single Robot Controller Program            │
│  ─────────────────────────────────────────  │
│  ├─ Motor Control                          │
│  ├─ Sensor Reading                         │
│  ├─ Path Planning                          │
│  ├─ Vision Processing                      │
│  └─ State Estimation                       │
└─────────────────────────────────────────────┘
```

**Problems**:
- If planning crashes, everything crashes
- Hard to test individual components
- Cannot reuse motor control code in another project
- Scaling to multiple robots or computers is difficult
- One slow component (vision processing) slows everything down

### The Distributed Approach (✅ ROS 2)

Multiple independent programs communicating via middleware:

```
┌─────────────┐  ┌──────────────────┐  ┌────────────────┐
│ Motor       │  │  Sensor Driver   │  │ Path Planning  │
│ Control     ├──┤  (IMU, Encoders) ├──┤ (A*, RRT, etc) │
│ Node        │  │  Node            │  │ Node           │
└──────┬──────┘  └────────┬─────────┘  └────────┬───────┘
       │                  │                     │
       └──────────────────┼─────────────────────┘
                    ROS 2 Middleware
                 (DDS, Topics, Services)
       ┌──────────────────┼─────────────────────┐
       │                  │                     │
┌──────▼──────┐  ┌───────▼────────┐  ┌────────▼────────┐
│ Vision      │  │ State Estimation│  │ Logging Node   │
│ Processing  │  │ (Kalman Filter) │  │ (Record Data)  │
│ Node        │  │ Node            │  │ Node           │
└─────────────┘  └────────────────┘  └────────────────┘
```

**Benefits**:
- Each component is independent and reusable
- If one crashes, others keep running
- Easy to test each component separately
- Easy to add/remove components
- Can run on multiple computers via network
- Each component can be written in different languages (Python, C++, etc.)

## Why ROS 2 for Humanoids?

### Complexity of Humanoid Robots

A humanoid robot has:
- **12-14 joints** (arms, legs, torso, head)
- **Multiple sensors**: IMU (inertial measurement unit), force sensors, encoders, cameras
- **Multiple controllers**: balance, motion planning, grasp control
- **Real-time constraints**: must respond to falling, obstacles within milliseconds

**Distributed architecture enables**:
- **Specialization**: Each node does one thing well
- **Parallelization**: Control and planning run simultaneously
- **Fault tolerance**: If planning fails, control still works
- **Hardware abstraction**: Switch from simulation to real hardware by swapping drivers

### Example: Humanoid Walking

```
Sensor Drivers                Control                Planning
─────────────────────────────────────────────────────────────
IMU publishes                 Balance Node        Walking
body motion                   subscribes to       Planner
   ↓                          IMU, adjusts        calculates
Encoders publish              joint angles        next step
joint angles                     ↓                   ↓
   ↓                          Motor Control       sends goal
Force sensors                 Node publishes      to Goal
publish ground                 motor commands     Tracker
contact                          ↓
   ↓                          Motors move      ← Real Hardware
All to ROS 2 Topics           ← To Simulation
                                or Real Hardware
```

Without ROS 2: The walking planner would need to know about:
- Which encoder gives which joint angle
- How fast sensors update
- How to convert desired motion to motor commands
- Network communication

**With ROS 2**: The planner simply:
1. Subscribes to `/imu` (sensor data)
2. Subscribes to `/joint_states` (current angles)
3. Publishes to `/goal_pose` (desired position)

The planner doesn't care if data comes from simulation or real hardware. Other nodes handle those details.

## ROS 2 Architecture

ROS 2 is built on three key pillars:

### 1. **DDS (Data Distribution Service)**

The underlying **communication protocol** that ensures reliable, efficient message delivery between nodes.

**Key features**:
- **Decoupled**: Publishers and subscribers don't need to know about each other
- **Flexible**: Quality of Service (QoS) settings for different use cases
- **Scalable**: Works on single machine or entire network
- **Standard**: Open specification, multiple implementations (Fast-RTPS, CycloneDDS, etc.)

### 2. **Nodes and Topics**

**Nodes** are independent programs. They communicate via **Topics**.

```
Publisher Node          Topic              Subscriber Node
(Motor Control)    ─→  /joint_commands  ──→ (Motor Driver)
                       (ROS 2 Channel)
```

Think of topics as bulletin boards:
- Publisher posts messages to a bulletin board (topic)
- Subscribers read messages from the bulletin board
- Publisher and subscriber never talk directly

### 3. **Services and Actions**

For request-reply communication:

**Service**: Synchronous request-reply (waits for answer)
```
Client Node              Service              Server Node
(Planner)      ─→ /set_motor_speed ──→ (Motor Control)
               ←─ (response)          ←─
```

**Action**: Asynchronous task (long-running, with feedback)
```
Goal Sender             Action              Goal Executor
(Planner)     ─→ /move_to_pose ──→ (Motion Controller)
              ← (feedback: progress)
              ← (result: success/failure)
```

## Key Concepts

### Loose Coupling

Nodes don't know about each other. They only know:
- What topic to publish/subscribe to
- What message format to use

**Benefit**: Change one node without affecting others. Swap sensor drivers without changing control code.

### Flexibility

ROS 2 adapts to different scenarios:
- **Single machine**: All nodes on one computer
- **Multiple machines**: Nodes communicate over network
- **Simulation**: Swap hardware drivers for simulator drivers
- **Different languages**: Mix Python, C++, Rust nodes

### Real-Time Capability

For humanoids, some tasks need real-time guarantees (balance, falling):
- **QoS settings** ensure messages arrive reliably
- **Deterministic execution** is possible (Real-Time Linux, etc.)
- **Middleware** is optimized for low-latency communication

## Where ROS 2 Fits in Your System

```
┌────────────────────────────────────────────┐
│  Your Application Layer                    │
│  (Path planning, Decision-making, etc.)    │
│                                            │
│  Written in Python, C++, or other langs    │
└──────────────┬─────────────────────────────┘
               │
      ROS 2 Middleware Layer (This Book)
      ├─ Topics (pub-sub)
      ├─ Services (request-reply)
      ├─ Actions (async tasks)
      └─ QoS (quality of service)
               │
┌──────────────▼─────────────────────────────┐
│  Hardware Abstraction Layer                │
│  ├─ Motor drivers                          │
│  ├─ Sensor drivers (IMU, camera, etc.)    │
│  └─ Network communication                  │
└──────────────┬─────────────────────────────┘
               │
┌──────────────▼─────────────────────────────┐
│  Hardware                                  │
│  ├─ Motors                                 │
│  ├─ Sensors                                │
│  └─ Network                                │
└────────────────────────────────────────────┘
```

ROS 2 provides the middleware layer, abstracting hardware details from your application logic.

## What We'll Learn in This Chapter

In this chapter (Part 1: Foundations), you'll learn:

1. **ROS 2 Concepts**: Nodes, topics, services, actions
2. **DDS Middleware**: Quality of Service settings, publish-subscribe patterns
3. **Humanoid Examples**: Real scenarios for distributed control
4. **Why Loose Coupling**: Benefits for complex systems like humanoids
5. **Getting Data to Flow**: How topics, messages, and subscriptions work

## Next Steps

Ready to dive deeper? Read on:

1. **[Next: DDS Concepts](./02-dds-concepts.md)** - Understand the communication backbone
2. **Or jump ahead**: If you're already familiar with DDS, read [Why Humanoids?](./03-why-humanoids.md)

---

## Further Reading

- **Official ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ROS 2 Concepts**: https://docs.ros.org/en/humble/Concepts.html
- **ROS 2 About ROS**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-ROS-2.html
- **DDS Standard**: https://www.omg.org/spec/DDS/

---

**Questions?** See the [Glossary](../glossary.md) or [Known Issues](../known-issues.md).
