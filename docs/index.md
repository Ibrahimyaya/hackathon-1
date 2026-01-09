---
slug: /
sidebar_position: 1
---

# ROS 2 as the Robotic Nervous System for Humanoid Robots

Welcome to a comprehensive guide to ROS 2 for humanoid robotics. This book teaches you how to build distributed, decoupled control systems for humanoid robots using ROS 2 middleware.

## What You'll Learn

This book is organized into three progressive parts that build upon each other:

### **Part 1: Foundations** 📚
Understand why ROS 2 is essential for humanoid robots and how DDS (Data Distribution Service) middleware enables reliable, distributed communication.

- What ROS 2 is and its role as middleware
- DDS concepts and Quality of Service (QoS) settings
- Why loose coupling benefits distributed robotic systems
- Real humanoid examples (IMU sensors, motor control)

**Duration**: ~2 hours | **Outcomes**: Understand pub-sub architecture, node lifecycle, QoS effects

---

### **Part 2: Communication Patterns** 🔌
Master the core ROS 2 communication patterns (Topics, Services, Actions) through hands-on examples. Learn the agent/controller architecture for building humanoid controllers.

- ROS 2 Nodes and their lifecycle
- Topics for asynchronous publish-subscribe
- Services for synchronous request-reply
- Actions for long-running async tasks
- Agent/controller pattern for humanoid control

**Duration**: ~4 hours | **Outcomes**: Build working multi-node systems, implement realistic humanoid control flows

---

### **Part 3: Robot Structure** 🤖
Learn URDF (Unified Robot Description Format) as the standard way to describe robot structure. Write a complete humanoid URDF, load it in RViz and Gazebo, and simulate control.

- URDF XML format and robot structure
- Complete humanoid URDF with realistic joints and masses
- Loading robots in RViz for visualization
- Simulating control in Gazebo physics engine
- Adapting examples for real hardware vs. simulation

**Duration**: ~3 hours | **Outcomes**: Describe complete humanoid robot, simulate control flows, understand real hardware constraints

---

## Target Audience

- **AI students and developers** entering humanoid robotics
- Developers with **intermediate Python** experience
- Those comfortable with **object-oriented programming**
- Readers ready to build **real working systems**

This book assumes you understand Python; it does not teach Python syntax. It assumes you understand distributed systems concepts; it explains them in the ROS 2 context.

---

## How to Use This Book

### **Option 1: Learn from Start to Finish** ✅ Recommended
1. Start with **Part 1: Foundations** to understand ROS 2 concepts
2. Work through **Part 2: Communication Patterns** to build hands-on examples
3. Complete **Part 3: Robot Structure** to simulate complete humanoid systems

Each chapter is self-contained and includes working code examples you can run on your Ubuntu 22.04 LTS system. Every example is reproducible from start to finish.

### **Option 2: Jump to a Specific Topic**
- If you already understand ROS 2 basics, start with **Part 2**
- If you only need URDF and simulation, start with **Part 3**
- Each part includes setup instructions and references to foundational concepts

### **Option 3: Run Examples in Parallel**
Each chapter's code examples are independent and can be run on separate terminal windows. Follow the example READMEs in `docs/examples/` for step-by-step instructions.

---

## Getting Started (30 minutes)

### Prerequisites
- Ubuntu 22.04 LTS (or later)
- Basic terminal/shell experience
- Python 3.10 or later
- Intermediate Python programming knowledge

### Quick Start
1. **Follow the [Setup Guide](./setup-guide.md)** to install ROS 2 Humble and dependencies (~15 minutes)
2. **Try the [Quickstart Example](./quickstart.md)** - write and run your first ROS 2 publisher node (~10 minutes)
3. **Explore Part 1** to understand concepts behind what you just built

---

## Key Features of This Book

✅ **100% Reproducible** — Every code example runs on a clean Ubuntu 22.04 system without modification

✅ **Officially Sourced** — All technical claims cite official ROS 2 documentation and DDS specifications

✅ **Hands-On Learning** — 30+ working Python examples demonstrate every concept

✅ **Humanoid-Focused** — Examples use humanoid robot scenarios: joint states, motor control, humanoid URDF

✅ **Clear Progression** — Concepts → Hands-on Patterns → Complete Systems (Part 1 → 2 → 3)

✅ **Real Hardware Ready** — Examples distinguish simulation-only features from real hardware

---

## Technical Stack

This book uses **official ROS 2 tools and libraries** to ensure long-term relevance:

| Component | Tool | Version |
|-----------|------|---------|
| **ROS 2 Distribution** | Humble (LTS) | 2022.12+ |
| **Python Client** | rclpy | 3.10+ |
| **Robot Description** | URDF | Standard ROS 2 format |
| **Visualization** | RViz2 | ROS 2 standard |
| **Simulation** | Gazebo | ROS 2 compatible |
| **OS** | Ubuntu | 22.04 LTS |

---

## What This Book Does NOT Cover

- ❌ Advanced motion planning (MoveIt)
- ❌ Machine learning and perception
- ❌ Hardware driver development
- ❌ ROS 1 (previous version)
- ❌ Non-humanoid robots

These topics are out of scope but are documented with references to official resources.

---

## Success Criteria

By the end of this book, you will be able to:

✅ Explain ROS 2 as middleware and understand why DDS enables distributed robotics

✅ Build multi-node ROS 2 systems with Topics, Services, and Actions

✅ Implement agent/controller patterns for humanoid control flows

✅ Write complete URDF descriptions of humanoid robots

✅ Load and control robots in RViz and Gazebo simulation

✅ Understand how to adapt examples from simulation to real hardware

---

## How to Navigate

- **[Setup Guide](./setup-guide.md)** — Install ROS 2 Humble and dependencies
- **[Quickstart](./quickstart.md)** — Run your first ROS 2 example in 30 minutes
- **[Part 1: Foundations](./part1-foundations/01-ros2-overview)** — Learn ROS 2 and DDS concepts
- **[Part 2: Communication](./part2-communication/04-nodes-and-lifecycle)** — Master communication patterns
- **[Part 3: Robot Structure](./part3-robot-structure/09-urdf-fundamentals)** — Build humanoid robots
- **[Glossary](./glossary.md)** — Terms and definitions
- **[References](./references.md)** — Official documentation links
- **[Known Issues](./known-issues.md)** — Debugging tips and workarounds

---

## About This Book

This book is written by Anthropic for AI students and developers learning humanoid robotics. It follows official ROS 2 documentation (Humble/Jazzy LTS releases) and is maintained on [GitHub](https://github.com/anthropics/ros2-humanoid-book).

**Philosophy**: Start with concepts, practice with hands-on examples, integrate into complete systems. Every claim is traceable to official sources. Every example runs on clean systems without external services or proprietary tools.

---

## Get Help

- **Official ROS 2 Docs**: https://docs.ros.org/en/humble/
- **ROS 2 Discourse**: https://discourse.ros.org/
- **This Book's Issues**: https://github.com/anthropics/ros2-humanoid-book/issues
- **Glossary & References**: See links in sidebar

---

Ready? Start with [Setup Guide](./setup-guide.md) or jump to [Part 1: Foundations](./part1-foundations/01-ros2-overview).
