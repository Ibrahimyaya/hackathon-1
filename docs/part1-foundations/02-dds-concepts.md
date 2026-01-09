---
sidebar_position: 2
title: DDS Middleware Concepts
---

# DDS: Data Distribution Service

> **Learning Goal**: Understand DDS as the communication backbone of ROS 2, learn Quality of Service (QoS) settings, and see why loose coupling is critical for humanoid robots.

## DDS Overview

**DDS (Data Distribution Service)** is the middleware technology that powers ROS 2. It's a standard specification maintained by the OMG (Object Management Group) and used in industries ranging from aerospace to robotics.

### What Problem Does DDS Solve?

In traditional point-to-point communication, each component connects directly to others:

```
Motor ↔ Sensor Driver
  ↕       ↕
  Planner ← Balance Controller
```

This becomes unmaintainable with many components:
- How do Motor and Planner connect? Network? Shared memory?
- What if Sensor Driver crashes? Does Motor still work?
- How do we add a new Logging component?

**DDS abstracts this away** with a publish-subscribe model:

```
All nodes publish/subscribe to a shared message bus.
No direct connections needed.
Nodes are loosely coupled.
```

*This section is an outline. Full content to be added: DDS fundamentals, QoS settings, reliability policies, history settings, humanoid examples.*

---

See: [Official DDS Specification](https://www.omg.org/spec/DDS/)
