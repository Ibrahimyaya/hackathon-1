# Module 2 Implementation Plan: Gazebo and Unity Simulations for Humanoid Robotics

**Date**: 2026-01-08
**Status**: Planning
**Scope**: Comprehensive Gazebo and Unity simulation chapters with physics, environment, and sensor integration
**Target**: Follow existing Spec-Driven Development (SDD) framework from Module 1

---

## Executive Summary

Module 2 expands the ROS 2 humanoid robotics book with **two major simulation frameworks** for humanoid robot development and testing. This module provides hands-on, reproducible chapters covering:

1. **Gazebo Simulation** (3 chapters): Physics engine, environment setup, sensor integration
2. **Unity Simulation** (3 chapters): Physics configuration, scene building, sensor streaming to ROS 2

All chapters will be written as structured Markdown files, organized per the existing Docusaurus framework. Code examples will be reproducible on Ubuntu 22.04 (Gazebo) and cross-platform (Unity).

---

## Module 2 High-Level Structure

```
docs/
├── part4-gazebo-simulation/              # Module 2A: Gazebo Simulations
│   ├── 12-gazebo-fundamentals.md         # Chapter 12: Gazebo basics & ROS 2 integration
│   ├── 13-physics-simulation.md          # Chapter 13: Physics engine, joint dynamics, constraints
│   ├── 14-humanoid-gazebo-world.md       # Chapter 14: Building a complete humanoid simulation environment
│   ├── 15-gazebo-sensors.md              # Chapter 15: Sensor plugins (IMU, camera, laser, joint state)
│   ├── 16-sensor-streaming-ros2.md       # Chapter 16: Publishing sensor data to ROS 2 topics
│   └── 17-gazebo-advanced-physics.md     # Chapter 17: Advanced physics, custom plugins, debugging
│
├── part5-unity-simulation/               # Module 2B: Unity Simulations
│   ├── 18-unity-fundamentals.md          # Chapter 18: Unity for robotics, scene setup, humanoid import
│   ├── 19-unity-physics-setup.md         # Chapter 19: PhysX configuration, joint limits, constraints
│   ├── 20-unity-humanoid-scene.md        # Chapter 20: Building a complete humanoid scene with environment
│   ├── 21-unity-sensor-simulation.md     # Chapter 21: Sensor simulation (IMU, camera, proximity sensors)
│   ├── 22-unity-ros2-bridge.md           # Chapter 22: Connecting Unity to ROS 2 via TCP/UDP bridge
│   └── 23-unity-advanced-features.md     # Chapter 23: Advanced rendering, multi-robot scenes, deployment
│
├── examples/
│   ├── ch4-gazebo-simulation/
│   │   ├── gazebo-launch-file.launch.xml        # Example launch file
│   │   ├── humanoid-gazebo-world.world         # Complete Gazebo world file
│   │   ├── humanoid-gazebo-plugin.cpp          # Custom Gazebo plugin example
│   │   ├── sensor-publisher.py                 # ROS 2 sensor data publisher
│   │   ├── physics-tuning-guide.md             # Physics parameter documentation
│   │   └── README.md                           # Gazebo examples guide
│   │
│   └── ch5-unity-simulation/
│       ├── humanoid-unity-scene.md             # Scene setup instructions
│       ├── unity-ros2-bridge-csharp.cs         # C# ROS 2 bridge script
│       ├── sensor-simulation-script.cs         # Sensor simulation component
│       ├── physics-configuration.md            # PhysX setup guide
│       └── README.md                           # Unity examples guide
│
└── reference/
    ├── gazebo-ros2-integration.md              # Integration best practices
    ├── unity-ros2-comparison.md                # Gazebo vs. Unity analysis
    └── sensor-data-formats.md                  # Standardized sensor data structures
```

---

## Chapter Breakdown

### PART 4: GAZEBO SIMULATION (Chapters 12–17)

#### Chapter 12: Gazebo Fundamentals & ROS 2 Integration

**Learning Outcomes**:
- Understand Gazebo as a physics simulation engine for ROS 2
- Set up Gazebo on Ubuntu 22.04 with ROS 2 Humble/Jazzy
- Load and manipulate a simple humanoid model in Gazebo
- Understand the Gazebo/ROS 2 plugin architecture

**Key Sections**:
1. What is Gazebo? (history, versions, ROS 2 integration)
2. Gazebo Installation & Verification
3. Gazebo GUI Basics (menus, scene tree, properties)
4. Loading a URDF Model in Gazebo
5. Gazebo Launch Files (XML structure, parameter passing)
6. Gazebo Plugins Architecture (sensors, actuators, world plugins)
7. Gazebo ROS 2 Bridge (gz_ros2_control, ros2_control)

**Acceptance Criteria**:
- ✅ Readers can launch Gazebo and load the humanoid URDF from Module 1
- ✅ Readers understand the flow: URDF → Gazebo world → ROS 2 nodes
- ✅ A sample launch file successfully spawns a humanoid in Gazebo
- ✅ Gazebo window shows correct joint hierarchy and link collisions

**Code Examples** (4-5):
- `gazebo-launch-humanoid.launch.xml` — Launch file for humanoid
- `gazebo-empty-world.launch.xml` — Minimal world setup
- `spawn-model-script.py` — Python script to spawn URDF at runtime
- `verify-gazebo-setup.sh` — Verification script for installation

---

#### Chapter 13: Physics Simulation & Dynamics

**Learning Outcomes**:
- Configure physics engine (ODE, Bullet, DART, SIMBODY) for humanoid
- Understand joint dynamics, constraints, and contact modeling
- Tune friction, damping, and mass properties for realistic motion
- Debug physics instability and collision issues

**Key Sections**:
1. Physics Engine Selection (ODE, Bullet, DART; pros/cons)
2. Gazebo Physics Parameters (gravity, step size, contact parameters)
3. Joint Types & Dynamics (revolute, prismatic, constraints)
4. Mass & Inertia Configuration (link properties, COG placement)
5. Friction & Contact Modeling (mu, surface contact dynamics)
6. Joint Limits & Damping (position/velocity limits, friction)
7. Physics Debugging Tools (visualize contacts, forces, velocities)
8. Performance Tuning (step size, solver iterations, collision detection)

**Acceptance Criteria**:
- ✅ Humanoid can be dropped in gravity and settles stably without jitter
- ✅ Joint limits prevent unrealistic motion
- ✅ Friction parameters make contact realistic (no sliding when stationary)
- ✅ Motor torques produce smooth, controllable motion

**Code Examples** (5-6):
- `world-physics-config.world` — Gazebo world with physics settings
- `humanoid-with-physics.urdf` — URDF with realistic inertia/damping
- `physics-tuning-script.py` — Interactive tool to adjust parameters
- `contact-visualizer.py` — Visualize contact forces in RViz
- `joint-dynamics-example.py` — Control motor torques, log joint states
- `stability-test.py` — Automated physics stability test

---

#### Chapter 14: Humanoid Gazebo World Environment

**Learning Outcomes**:
- Design realistic environments for humanoid simulation
- Add terrain, obstacles, and interactive objects
- Set up lighting, camera views, and scene composition
- Create reproducible simulation scenarios

**Key Sections**:
1. Gazebo World File Format (SDF structure)
2. Ground Plane & Terrain (flat, heightmaps, friction)
3. Adding Static Objects (walls, boxes, tables)
4. Dynamic Objects (balls, blocks, movable furniture)
5. Lighting & Camera Setup (ambient light, point lights, default camera)
6. Environmental Effects (wind, gravity variations)
7. Scene Visualization (default camera angle, plugins)
8. Multi-Robot Worlds (multiple humanoids, cooperative scenarios)

**Acceptance Criteria**:
- ✅ Complete humanoid can walk/move in the world without falling through ground
- ✅ World file is valid SDF and loads without errors
- ✅ Objects are correctly placed, textured, and interactive
- ✅ Scene is visually clear and photorealistic (with proper lighting)

**Code Examples** (5-6):
- `humanoid-empty-world.world` — Minimal world file
- `humanoid-obstacle-course.world` — Complex world with obstacles
- `humanoid-outdoor-world.world` — Terrain/ground variation
- `add-dynamic-objects.py` — Script to insert objects at runtime
- `multi-robot-scenario.world` — Two humanoids in same world
- `world-template.world` — Reusable world template

---

#### Chapter 15: Gazebo Sensor Plugins & Integration

**Learning Outcomes**:
- Configure common sensors in Gazebo (IMU, camera, lidar, joint state)
- Understand sensor simulation accuracy and limitations
- Add custom sensors and plugins
- Verify sensor data in Gazebo and ROS 2

**Key Sections**:
1. Gazebo Sensor Plugin Architecture (SensorPlugin, ModelPlugin)
2. IMU Sensor (accelerometer, gyroscope, magnetometer simulation)
3. Camera Sensor (RGB, depth, intrinsic calibration)
4. Lidar/Laser Sensor (2D/3D point clouds, beam simulation)
5. Contact Sensor (bumper simulation, collision detection)
6. Force/Torque Sensor (6-axis force-torque sensing)
7. Proprioceptive Sensors (joint state, joint effort)
8. Sensor Noise Modeling (Gaussian noise, bias, quantization)

**Acceptance Criteria**:
- ✅ All configured sensors appear in Gazebo GUI and can be toggled
- ✅ Sensor data is published to ROS 2 topics (verified with `ros2 topic echo`)
- ✅ IMU shows realistic gravity orientation when humanoid is tilted
- ✅ Camera publishes valid image frames (rect with compression)
- ✅ Lidar publishes point cloud with correct geometry

**Code Examples** (6-7):
- `humanoid-with-sensors.urdf` — URDF with sensor definitions
- `gazebo-sensor-plugin.cpp` — Custom C++ sensor plugin
- `sensor-data-logger.py` — ROS 2 node to log all sensor streams
- `imu-verification.py` — Verify IMU calibration and noise
- `camera-viewer.py` — Subscribe to camera and display frames
- `lidar-visualization.py` — RViz visualization of point cloud
- `sensor-noise-characterization.md` — Documentation on sensor accuracy

---

#### Chapter 16: Sensor Data Streaming to ROS 2 Topics

**Learning Outcomes**:
- Configure Gazebo plugins to publish to ROS 2 topics
- Understand message types for common sensors
- Stream sensor data in real time with configurable rates
- Synchronize multi-sensor streams for consistent timestamps

**Key Sections**:
1. ROS 2 Control Integration (ros2_control, transmissions)
2. Standard ROS 2 Sensor Messages (`sensor_msgs/Imu`, `sensor_msgs/Image`, `sensor_msgs/LaserScan`, `sensor_msgs/JointState`)
3. Topic Naming Conventions (namespace hierarchies for multiple robots/sensors)
4. Publishing Rates & Synchronization (message timestamps, frame rates)
5. Gazebo-ROS 2 Plugin Binding (gz_ros2_control, custom plugins)
6. Transform Broadcasting (tf2 for sensor frames)
7. Debugging Sensor Streams (rostopic commands, filtering)

**Acceptance Criteria**:
- ✅ `ros2 topic list` shows all configured sensor topics
- ✅ Each sensor publishes at expected frequency (100+ Hz for IMU/joints, 30+ Hz for camera)
- ✅ Messages are valid, non-null, and timestamped consistently
- ✅ Multiple sensors can stream simultaneously without data loss
- ✅ Sensor frames are correctly registered in TF tree

**Code Examples** (5-6):
- `ros2-sensor-config.yaml` — ROS 2 control configuration
- `sensor-subscriber.py` — Multi-sensor subscriber node
- `sensor-synchronizer.py` — Combine multiple sensor streams with timestamp alignment
- `tf2-broadcaster.py` — Publish sensor frame transforms
- `topic-monitor.py` — Real-time visualization of all topics
- `data-recorder.rosbag` — Record all sensor streams to bag file

---

#### Chapter 17: Advanced Physics & Custom Plugins

**Learning Outcomes**:
- Write custom Gazebo plugins in C++
- Implement advanced physics (contact dynamics, fluid drag)
- Optimize performance for real-time simulation
- Integrate third-party physics libraries

**Key Sections**:
1. Gazebo Plugin Development (WorldPlugin, ModelPlugin, SensorPlugin, VisualsPlugin)
2. Physics Callback Hooks (OnUpdate, PrePhysics, PostPhysics)
3. Custom Contact Handling (contact events, friction rules)
4. Aerodynamic Effects (wind simulation, drag modeling)
5. Deformable Bodies & Soft Contacts (optional advanced topics)
6. Performance Optimization (computational budget, threading)
7. Plugin Debugging (logging, Gazebo introspection)
8. Integration with External Simulators (interfacing physics)

**Acceptance Criteria**:
- ✅ Custom plugin compiles without warnings/errors
- ✅ Custom physics behaviors (e.g., wind, contact events) work as specified
- ✅ Simulation runs in real time (wall clock ≈ sim time)
- ✅ Plugin can be toggled on/off without breaking simulation

**Code Examples** (4-5):
- `custom-physics-plugin.cpp` — Example WorldPlugin for custom physics
- `contact-callback-plugin.cpp` — Monitor and respond to contacts
- `wind-plugin.cpp` — Environmental effects plugin
- `plugin-performance-benchmark.cpp` — Performance testing framework
- `plugin-CMakeLists.txt` — Build configuration for custom plugins

---

### PART 5: UNITY SIMULATION (Chapters 18–23)

#### Chapter 18: Unity Fundamentals for Robotics

**Learning Outcomes**:
- Set up Unity for robotics simulation
- Import humanoid models (URDF, FBX)
- Understand PhysX physics engine in Unity
- Configure basic scene with humanoid robot

**Key Sections**:
1. Unity Installation & Setup (LTS versions, robotics packages)
2. Project Structure for Robotics (prefabs, scenes, scripts)
3. Importing Humanoid Models (URDF to FBX conversion, rigging)
4. Humanoid Avatar Setup (Mecanim, joint mapping)
5. PhysX Physics Configuration (gravity, default material)
6. Scene Hierarchy & Inspector Workflow
7. Physics Debugging (visual debugging, gizmos)
8. Performance Profiler (CPU/GPU usage, frame rate)

**Acceptance Criteria**:
- ✅ Unity project loads without errors
- ✅ Humanoid model loads and displays correctly in scene
- ✅ Humanoid can be manipulated in editor and play mode
- ✅ Physics simulation runs smoothly (60+ FPS)

**Code Examples** (4-5):
- `ProjectSetup.md` — Step-by-step project initialization
- `HumanoidImporter.cs` — Script to import URDF models
- `PhysicsDebugger.cs` — Visualization of physics bounds and forces
- `PerformanceMonitor.cs` — Real-time FPS and memory tracking
- `unity-project-structure.md` — Directory organization best practices

---

#### Chapter 19: Unity Physics Configuration & Joint Setup

**Learning Outcomes**:
- Configure PhysX physics for humanoid dynamics
- Set up joint constraints (revolute, fixed, ball-socket)
- Define joint limits, damping, and spring forces
- Tune physics for realistic humanoid motion

**Key Sections**:
1. PhysX Physics Engine (rigid bodies, colliders, constraints)
2. Rigidbody Configuration (mass, drag, constraints)
3. Joint Components in Unity (HingeJoint, ConfigurableJoint)
4. Joint Limits & Drives (position/velocity targets, springs)
5. Motor Torque Control (joint motors, force limits)
6. Collision Detection (convex meshes, collision layers)
7. Material Properties (friction, bounciness, surface properties)
8. Multi-Constraint Scenarios (kinematic chains, closed loops)

**Acceptance Criteria**:
- ✅ All humanoid joints have correct limits and can move smoothly
- ✅ Humanoid dropped from height settles stably on ground
- ✅ Gravity and inertia create realistic motion dynamics
- ✅ Motor commands produce proportional joint motion
- ✅ Collision detection prevents interpenetration

**Code Examples** (5-6):
- `HumanoidPhysicsSetup.cs` — Script to configure all joints
- `JointLimitConfiguration.cs` — Define limits for each joint
- `MotorController.cs` — Control joint motors with target angles/velocities
- `PhysicsMaterial.asset` — Pre-configured physics materials
- `JointTuningTool.cs` — Interactive UI to tune parameters in real-time
- `physics-parameters.json` — Configuration file for all physics settings

---

#### Chapter 20: Building Complete Humanoid Scene in Unity

**Learning Outcomes**:
- Design complex environments in Unity
- Populate scenes with terrain, objects, lighting
- Create realistic backgrounds and visual elements
- Organize scene for simulation and deployment

**Key Sections**:
1. Unity Scene Composition (terrain, skybox, lighting)
2. Terrain Tools (heightmaps, vegetation, texturing)
3. Adding Static Objects (walls, furniture, props)
4. Dynamic Objects (balls, movable blocks, interactive elements)
5. Lighting Setup (directional, point, spotlights; shadows)
6. Material & Shader Configuration (textures, normal maps, metallic)
7. Camera Setup (third-person, first-person, debug views)
8. Scene Optimization (LOD, culling, batching)

**Acceptance Criteria**:
- ✅ Complete scene loads without errors
- ✅ All objects are visible and correctly lit
- ✅ Scene runs at target frame rate (60+ FPS)
- ✅ Humanoid can interact naturally with environment (no clipping, realistic collisions)
- ✅ Multiple camera angles provide useful debugging views

**Code Examples** (5-6):
- `SceneSetup.md` — Step-by-step scene construction guide
- `TerrainBuilder.cs` — Procedural terrain generation
- `EnvironmentBuilder.cs` — Populate scene with objects
- `LightingConfiguration.cs` — Advanced lighting setup
- `CameraController.cs` — Multiple camera modes and transitions
- `scene-prefabs.md` — Reusable scene components

---

#### Chapter 21: Sensor Simulation in Unity

**Learning Outcomes**:
- Simulate common sensors in Unity (camera, IMU, lidar)
- Generate realistic sensor data with noise
- Configure sensor update rates and accuracy
- Stream sensor data via ROS 2 bridge

**Key Sections**:
1. Camera Simulation (RGB rendering, depth maps, image formats)
2. IMU Simulation (accelerometer, gyroscope data)
3. Lidar/Point Cloud Simulation (raycast-based point generation)
4. Proximity Sensors (distance measurement, obstacle detection)
5. Contact & Pressure Sensors (collision callbacks, force measurement)
6. Sensor Noise Modeling (Gaussian noise, quantization, bias)
7. Sensor Fusion (combining multiple sensors)
8. Recording & Playback (sensor data logging for analysis)

**Acceptance Criteria**:
- ✅ All sensors produce realistic data (verified against physical reference)
- ✅ Sensors update at configurable rates (100+ Hz for proprioceptive)
- ✅ Noise models match hardware specifications
- ✅ Multi-sensor data can be synchronized with consistent timestamps
- ✅ Sensor data is valid and non-null in all conditions

**Code Examples** (6-7):
- `CameraSimulator.cs` — Render and publish camera data
- `IMUSensor.cs` — Simulate IMU with realistic noise
- `LidarSimulator.cs` — Raycast-based point cloud generation
- `SensorNoise.cs` — Configurable noise injection
- `SensorManager.cs` — Coordinate multiple sensors
- `sensor-config.json` — Per-sensor calibration data
- `sensor-verification.cs` — Automated testing of sensor accuracy

---

#### Chapter 22: Connecting Unity to ROS 2 (TCP/UDP Bridge)

**Learning Outcomes**:
- Implement ROS 2 integration in Unity via TCP/UDP bridge
- Publish sensor data to ROS 2 topics
- Subscribe to control commands from ROS 2 nodes
- Handle network communication robustly

**Key Sections**:
1. ROS 2 Communication Protocols (DDS, TCP/UDP fallback)
2. Unity Networking (WebSockets, TCP sockets, UDP)
3. Message Serialization (JSON, binary, Protocol Buffers)
4. Custom ROS 2 Bridge (connecting Unity to ros2_bridge_cpp)
5. Topic Publishing (sensor streams, ground truth)
6. Topic Subscription (control commands, target states)
7. Service Calls (query world state, reset simulation)
8. Network Debugging (packet inspection, latency monitoring)
9. Failsafe & Timeout Handling (reconnection, watchdog)

**Acceptance Criteria**:
- ✅ Unity application can connect to ROS 2 master/agent
- ✅ Sensor topics are published and visible in ROS 2 (`ros2 topic list/echo`)
- ✅ ROS 2 control commands are received and applied to humanoid joints
- ✅ Network latency is acceptable (<100 ms for control loops)
- ✅ Bridge gracefully handles connection drops and restarts

**Code Examples** (6-7):
- `ROS2Bridge.cs` — TCP/UDP bridge to ROS 2
- `TopicPublisher.cs` — Publish sensor data as ROS 2 messages
- `TopicSubscriber.cs` — Subscribe and apply control commands
- `MessageSerialization.cs` — Convert C# objects to ROS 2 message format
- `NetworkMonitor.cs` — Monitor connection health and latency
- `SimulationController.py` — ROS 2 node to control Unity simulation
- `bridge-config.yaml` — Configuration for topics and message mapping

---

#### Chapter 23: Advanced Unity Features & Deployment

**Learning Outcomes**:
- Implement advanced rendering and simulation features
- Support multi-robot scenarios and complex interactions
- Deploy Unity simulation for distributed use
- Optimize for real-time performance

**Key Sections**:
1. Advanced Rendering (ray tracing, post-processing, particle effects)
2. Multi-Robot Simulation (spawning, managing multiple humanoids)
3. AI & Behavior Trees (non-player characters, obstacle avoidance)
4. Recorded Playback (replay scenarios, data analysis)
5. Build & Deployment (Windows, Linux, macOS, headless mode)
6. Cloud Simulation (remote rendering, distributed physics)
7. Performance Profiling (CPU/GPU optimization, bottleneck analysis)
8. Integration with Reinforcement Learning (collecting training data)

**Acceptance Criteria**:
- ✅ Multi-robot scene runs at target performance (30+ FPS with 3 robots)
- ✅ Advanced rendering can be toggled without affecting physics accuracy
- ✅ Build produces standalone executable for target platform
- ✅ Headless mode works for automated simulation/testing
- ✅ RL training data can be extracted and formatted correctly

**Code Examples** (5-6):
- `MultiRobotManager.cs` — Spawn and manage multiple instances
- `BehaviorTree.cs` — AI decision framework
- `RecordingSystem.cs` — Capture and replay scenarios
- `BuildConfiguration.cs` — Deployment settings and optimization
- `HeadlessRunner.cs` — Headless/server mode for remote simulation
- `RLDataCollector.cs` — Extract training data for machine learning

---

## Cross-Module Content

### Reference Section Additions

**New Reference Files** (to be added under `docs/reference/`):

1. **`gazebo-ros2-integration.md`** (5000 words)
   - Best practices for Gazebo/ROS 2 integration
   - Plugin architecture and debugging
   - Network configuration for multi-machine simulation
   - Performance benchmarks and optimization

2. **`unity-ros2-bridge.md`** (4000 words)
   - Detailed guide to TCP/UDP communication
   - Message serialization strategies
   - Network reliability and error handling
   - Comparison of different bridge implementations

3. **`gazebo-vs-unity-comparison.md`** (3000 words)
   - Feature comparison (physics, rendering, extensibility)
   - When to use each simulator
   - Workflow recommendations (Gazebo for ROS 2 native, Unity for VR/visualization)
   - Cost and resource considerations

4. **`sensor-data-formats.md`** (2500 words)
   - Standard ROS 2 message types
   - Customization for non-standard sensors
   - Data format specifications and examples
   - Validation and verification procedures

5. **`simulation-debugging-guide.md`** (4000 words)
   - Common issues in Gazebo and Unity
   - Debugging techniques (visualization, logging, inspection)
   - Performance profiling and optimization
   - Reproducing physics bugs

---

## Module 2 Example Repository Structure

```
docs/examples/
├── ch4-gazebo-simulation/
│   ├── README.md
│   ├── 01-basic-launch.launch.xml
│   ├── 02-physics-tuning.world
│   ├── 03-humanoid-with-sensors.urdf (symlink to Part 1)
│   ├── 04-sensor-publisher.py
│   ├── 05-joint-controller.py
│   ├── 06-contact-monitor.py
│   ├── 07-physics-stability-test.py
│   ├── gazebo-plugins/
│   │   ├── CMakeLists.txt
│   │   ├── custom-physics-plugin.cpp
│   │   ├── contact-callback-plugin.cpp
│   │   └── wind-plugin.cpp
│   └── verification/
│       ├── gazebo-installation-test.sh
│       ├── physics-stability-test.py
│       └── sensor-stream-test.py
│
└── ch5-unity-simulation/
    ├── README.md
    ├── Assets/
    │   ├── Scenes/
    │   │   ├── RobotSimulation.unity
    │   │   ├── MultiRobotScene.unity
    │   │   └── HumanoidObstacle.unity
    │   ├── Scripts/
    │   │   ├── HumanoidPhysicsSetup.cs
    │   │   ├── ROS2Bridge.cs
    │   │   ├── SensorSimulator.cs
    │   │   └── EnvironmentBuilder.cs
    │   ├── Prefabs/
    │   │   ├── Humanoid.prefab
    │   │   ├── EnvironmentObject.prefab
    │   │   └── Sensor.prefab
    │   └── Materials/
    │       ├── Ground.mat
    │       ├── Humanoid.mat
    │       └── Objects.mat
    ├── ProjectSettings/
    │   └── (Unity auto-generated)
    └── verification/
        ├── unity-setup-test.md
        ├── physics-verification.cs
        └── ros2-bridge-test.py
```

---

## Implementation Phases

### Phase 1: Gazebo Foundation (Chapters 12-13)
- Set up Gazebo integration examples
- Document physics engine configuration
- Create reproducible physics test cases
- Deliverable: `docs/part4-gazebo-simulation/` chapters 12-13 complete

### Phase 2: Gazebo Integration (Chapters 14-16)
- Build complete world environments
- Implement sensor plugins and data streaming
- Create end-to-end ROS 2 sensor examples
- Deliverable: Chapters 14-16, sensor integration working

### Phase 3: Gazebo Advanced (Chapter 17)
- Custom plugin development examples
- Advanced physics implementation
- Performance optimization guide
- Deliverable: Chapter 17, plugin framework documented

### Phase 4: Unity Foundation (Chapters 18-20)
- Unity project setup and configuration
- Physics tuning for humanoids
- Scene building and environment design
- Deliverable: `docs/part5-unity-simulation/` chapters 18-20 complete

### Phase 5: Unity Integration (Chapters 21-22)
- Sensor simulation in Unity
- ROS 2 bridge implementation
- End-to-end communication example
- Deliverable: Chapters 21-22, working Unity-ROS 2 pipeline

### Phase 6: Unity Advanced & Reference (Chapter 23 + Reference)
- Advanced features (multi-robot, RL integration)
- Complete reference documentation
- Comparative guides and best practices
- Deliverable: Full Module 2 documentation, all examples working

---

## Acceptance Criteria (Module 2)

### Code Quality
- ✅ 100% code examples run without modification on target platforms
- ✅ All examples include complete imports, setup, and usage instructions
- ✅ Code is documented with comments explaining non-obvious behavior
- ✅ Examples follow project naming conventions and patterns

### Documentation Quality
- ✅ All chapters follow 3-part structure: Introduction → Core Concepts → Hands-On Examples
- ✅ 90%+ of technical claims cite official documentation (Gazebo, Unity, ROS 2 sources)
- ✅ Cross-references between chapters are accurate and up-to-date
- ✅ Glossary is updated with all new terms introduced

### Functionality
- ✅ Gazebo examples load humanoid and run physics simulation at real-time speed
- ✅ All sensors publish valid data to ROS 2 topics at specified frequencies
- ✅ Unity scenes load without errors and maintain 60+ FPS
- ✅ ROS 2 bridge successfully exchanges messages between Unity and ROS 2 nodes

### Reproducibility
- ✅ Setup instructions complete in ≤60 minutes for both Gazebo and Unity
- ✅ All code examples produce expected output on clean systems
- ✅ Verification scripts confirm working installations
- ✅ Troubleshooting guide covers ≥90% of common issues

---

## Success Metrics (Module 2)

| Metric | Target | Verification |
|--------|--------|--------------|
| Code Examples Reproducible | 100% | Test on 2 clean systems each (Ubuntu for Gazebo, 1 Windows + 1 Linux for Unity) |
| Documentation Accuracy | 100% | Cross-check all claims against official sources |
| Chapter Completeness | 100% | All 12 chapters written and peer-reviewed |
| Example Quality | 90%+ with comments | Automated code review for clarity and documentation |
| Setup Time | ≤60 min | Benchmark on clean systems |
| Gazebo Physics Stability | No jitter/clipping | Visual inspection and automated contact tests |
| Sensor Data Quality | 100% valid | Automated validation of message structure and rate |
| ROS 2 Integration | 100% working | End-to-end test of all topics/services |
| Performance | Gazebo ≥1.0x RT, Unity ≥30 FPS | Profiling and optimization validation |

---

## Dependencies & Constraints

### Software Requirements

**Gazebo Track**:
- ROS 2 Humble/Jazzy (same as Module 1)
- Gazebo (gz-sim version matching ROS 2 release)
- Ubuntu 22.04 LTS (or 24.04 LTS for Jazzy)
- Gazebo ROS 2 control plugin
- Python 3.10+

**Unity Track**:
- Unity 2022 LTS or later
- Visual Studio or Rider (C# IDE)
- PhysX physics engine (bundled with Unity)
- Optional: Blender for model conversion (URDF → FBX)
- Windows/Mac/Linux development platform

### Scope Boundaries

**In Scope** (Module 2):
- Gazebo simulation with ROS 2 integration
- Unity simulation with ROS 2 bridge via TCP/UDP
- Humanoid-specific physics and sensor configurations
- Standard ROS 2 sensors (IMU, camera, lidar, joint state)
- Reference documentation and comparisons

**Out of Scope**:
- Advanced motion planning (MoveIt) integration
- Real hardware deployment
- Reinforcement Learning training loops (only data collection setup)
- Custom physics libraries beyond PhysX/ODE
- Commercial robot integrations (proprietary URDFs)
- VR/AR interaction frameworks

---

## Next Steps

1. **Create Spec Document**: Generate `specs/001-ros2-humanoid-book/module-2-spec.md` with formal requirements
2. **Create Tasks**: Use `/sp.tasks` to generate implementation tasks for each phase
3. **Establish Examples**: Set up example directory structure and placeholder files
4. **Phase 1 Implementation**: Begin with Chapter 12-13 (Gazebo fundamentals)
5. **Iterate**: Complete phases sequentially with verification at each milestone

---

## Appendix: Markdown File Templates

### Chapter Template (for all 12 chapters)

```markdown
# Chapter [N]: [Title]

**Learning Outcomes** (bullet list, 3-5 items)

**Prerequisites** (knowledge/setup required)

## Section 1: [Topic]

### Key Concepts
- Concept A
- Concept B
- Concept C

### Example Code
\`\`\`python
# Code example
\`\`\`

## Section 2: [Topic]

... (repeat structure)

## Summary & Next Steps

## Further Reading
- Official documentation link
- Related chapter reference
```

### Example Code Template

```markdown
# Example: [Title]

**Difficulty**: Beginner/Intermediate/Advanced
**Time to Complete**: 15-30 minutes
**Dependencies**: ROS 2, Gazebo (or Unity)

## What You'll Learn
- Learning outcome 1
- Learning outcome 2

## Prerequisites
- Step 1: Install/setup requirement
- Step 2: Verify installation

## Complete Code

\`\`\`cpp
// or .py, .cs, .xml depending on context
// Full, copy-paste ready code
\`\`\`

## Expected Output
\`\`\`
[Example output]
\`\`\`

## Explanation

### Key Points
- Point 1
- Point 2

### Common Errors & Fixes
- Error: [issue]
  Solution: [fix]

## Next Steps
- Try modifying X to do Y
- Explore related example [link]

## References
- [Official docs](url)
- [Related chapter](link)
```

---

**Module 2 Plan Status**: ✅ Complete and Ready for Specification Phase
