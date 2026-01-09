# Module 2 Directory Structure & File Organization

**Created**: 2026-01-08
**Purpose**: Detailed breakdown of all directories, files, and naming conventions for Module 2 content

---

## Complete Directory Tree

```
docs/
│
├── part4-gazebo-simulation/                    # Module 2A: Gazebo Simulations
│   │
│   ├── 12-gazebo-fundamentals.md               # Chapter 12: Gazebo basics & ROS 2 integration
│   │   ├── Introduction to Gazebo
│   │   ├── Gazebo Installation on Ubuntu 22.04
│   │   ├── Gazebo GUI Overview
│   │   ├── Loading URDF Models
│   │   ├── Launch Files (XML format & structure)
│   │   ├── Gazebo Plugin Architecture
│   │   ├── ROS 2 Integration via plugins
│   │   └── Summary & Exercises
│   │
│   ├── 13-physics-simulation.md                # Chapter 13: Physics engine & dynamics
│   │   ├── Physics Engine Selection (ODE, Bullet, DART)
│   │   ├── Physics Parameters Configuration
│   │   ├── Joint Types & Dynamics
│   │   ├── Mass & Inertia Configuration
│   │   ├── Friction & Contact Modeling
│   │   ├── Joint Limits & Damping
│   │   ├── Physics Debugging Tools
│   │   ├── Performance Tuning
│   │   └── Summary & Exercises
│   │
│   ├── 14-humanoid-gazebo-world.md             # Chapter 14: Environment & world building
│   │   ├── Gazebo World File Format (SDF)
│   │   ├── Ground Plane & Terrain
│   │   ├── Static Objects (walls, boxes)
│   │   ├── Dynamic Objects (balls, blocks)
│   │   ├── Lighting & Camera Setup
│   │   ├── Environmental Effects
│   │   ├── Scene Visualization
│   │   ├── Multi-Robot Worlds
│   │   └── Summary & Exercises
│   │
│   ├── 15-gazebo-sensors.md                    # Chapter 15: Sensor plugins & simulation
│   │   ├── Gazebo Sensor Plugin Architecture
│   │   ├── IMU Sensor (accelerometer, gyro, magnetometer)
│   │   ├── Camera Sensor (RGB, depth)
│   │   ├── Lidar/Laser Sensor (2D/3D point clouds)
│   │   ├── Contact Sensor (bumper, collision)
│   │   ├── Force/Torque Sensor (6-axis)
│   │   ├── Proprioceptive Sensors (joint state, effort)
│   │   ├── Sensor Noise Modeling
│   │   └── Summary & Exercises
│   │
│   ├── 16-sensor-streaming-ros2.md             # Chapter 16: Publishing to ROS 2 topics
│   │   ├── ROS 2 Control Integration
│   │   ├── Standard ROS 2 Message Types
│   │   ├── Topic Naming Conventions
│   │   ├── Publishing Rates & Synchronization
│   │   ├── Gazebo-ROS 2 Plugin Binding
│   │   ├── Transform Broadcasting (tf2)
│   │   ├── Debugging Sensor Streams
│   │   └── Summary & Exercises
│   │
│   └── 17-gazebo-advanced-physics.md           # Chapter 17: Advanced & custom plugins
│       ├── Gazebo Plugin Development (WorldPlugin, ModelPlugin, SensorPlugin)
│       ├── Physics Callback Hooks
│       ├── Custom Contact Handling
│       ├── Aerodynamic Effects
│       ├── Deformable Bodies & Soft Contacts
│       ├── Performance Optimization
│       ├── Plugin Debugging
│       ├── Integration with External Simulators
│       └── Summary & Exercises
│
├── part5-unity-simulation/                     # Module 2B: Unity Simulations
│   │
│   ├── 18-unity-fundamentals.md                # Chapter 18: Unity basics & robot import
│   │   ├── Unity Installation & Setup (LTS versions)
│   │   ├── Project Structure for Robotics
│   │   ├── Importing Humanoid Models (URDF, FBX)
│   │   ├── Humanoid Avatar Setup (Mecanim)
│   │   ├── PhysX Physics Configuration
│   │   ├── Scene Hierarchy & Inspector
│   │   ├── Physics Debugging (visual, gizmos)
│   │   ├── Performance Profiler
│   │   └── Summary & Exercises
│   │
│   ├── 19-unity-physics-setup.md               # Chapter 19: Physics configuration & joints
│   │   ├── PhysX Physics Engine Fundamentals
│   │   ├── Rigidbody Configuration
│   │   ├── Joint Components (HingeJoint, ConfigurableJoint)
│   │   ├── Joint Limits & Drives
│   │   ├── Motor Torque Control
│   │   ├── Collision Detection & Layers
│   │   ├── Material Properties (friction, bounce)
│   │   ├── Multi-Constraint Scenarios
│   │   └── Summary & Exercises
│   │
│   ├── 20-unity-humanoid-scene.md              # Chapter 20: Environment & scene building
│   │   ├── Scene Composition (terrain, skybox, lighting)
│   │   ├── Terrain Tools (heightmaps, vegetation)
│   │   ├── Static Objects (walls, furniture, props)
│   │   ├── Dynamic Objects (balls, movable blocks)
│   │   ├── Lighting Setup (directional, point, spot)
│   │   ├── Material & Shader Configuration
│   │   ├── Camera Setup (3rd person, 1st person, debug)
│   │   ├── Scene Optimization
│   │   └── Summary & Exercises
│   │
│   ├── 21-unity-sensor-simulation.md           # Chapter 21: Sensor simulation & data generation
│   │   ├── Camera Simulation (RGB, depth, intrinsic calibration)
│   │   ├── IMU Simulation (accelerometer, gyroscope)
│   │   ├── Lidar/Point Cloud Simulation (raycast-based)
│   │   ├── Proximity Sensors (distance, obstacle detection)
│   │   ├── Contact & Pressure Sensors
│   │   ├── Sensor Noise Modeling
│   │   ├── Sensor Fusion (combining multiple sensors)
│   │   ├── Recording & Playback
│   │   └── Summary & Exercises
│   │
│   ├── 22-unity-ros2-bridge.md                 # Chapter 22: ROS 2 integration & bridge
│   │   ├── ROS 2 Communication Protocols
│   │   ├── Unity Networking (WebSockets, TCP, UDP)
│   │   ├── Message Serialization (JSON, binary, Protobuf)
│   │   ├── Custom ROS 2 Bridge Implementation
│   │   ├── Topic Publishing (sensor streams)
│   │   ├── Topic Subscription (control commands)
│   │   ├── Service Calls (query, reset)
│   │   ├── Network Debugging & Monitoring
│   │   ├── Failsafe & Timeout Handling
│   │   └── Summary & Exercises
│   │
│   └── 23-unity-advanced-features.md           # Chapter 23: Advanced features & deployment
│       ├── Advanced Rendering (ray tracing, post-processing)
│       ├── Multi-Robot Simulation
│       ├── AI & Behavior Trees
│       ├── Recorded Playback & Analysis
│       ├── Build & Deployment (Windows, Linux, Mac, headless)
│       ├── Cloud Simulation (remote rendering, distributed physics)
│       ├── Performance Profiling & Optimization
│       ├── RL Integration (training data collection)
│       └── Summary & Exercises
│
├── examples/
│   │
│   ├── ch4-gazebo-simulation/                  # All Gazebo examples
│   │   │
│   │   ├── README.md                           # Overview & how to run examples
│   │   │
│   │   ├── 01-basic-launch/
│   │   │   ├── gazebo-empty-world.launch.xml   # Minimal launch file
│   │   │   ├── gazebo-humanoid.launch.xml      # Launch with humanoid
│   │   │   └── spawn-model.py                  # Runtime model spawning script
│   │   │
│   │   ├── 02-physics-tuning/
│   │   │   ├── humanoid-physics.world          # World file with physics config
│   │   │   ├── physics-config.yaml             # Physics parameters
│   │   │   ├── physics-tuning-tool.py          # Interactive tuning script
│   │   │   └── verify-stability.py             # Stability test script
│   │   │
│   │   ├── 03-environment/
│   │   │   ├── humanoid-with-obstacles.world   # Complex world with objects
│   │   │   ├── humanoid-outdoor.world          # Terrain and outdoor environment
│   │   │   ├── multi-robot.world               # Two humanoids in same world
│   │   │   └── world-builder.py                # Script to generate worlds
│   │   │
│   │   ├── 04-sensors/
│   │   │   ├── humanoid-with-sensors.urdf      # URDF with sensor definitions (symlink from Part 1)
│   │   │   ├── sensor-publisher.py             # ROS 2 node publishing all sensor data
│   │   │   ├── sensor-subscriber.py            # Receive and log sensor data
│   │   │   ├── imu-calibration.py              # Verify IMU calibration
│   │   │   ├── camera-viewer.py                # Display camera images
│   │   │   ├── lidar-visualizer.py             # Visualize point clouds in RViz
│   │   │   └── sensor-sync.py                  # Synchronize multiple sensor streams
│   │   │
│   │   ├── 05-control/
│   │   │   ├── joint-controller.py             # Control humanoid joints via ROS 2
│   │   │   ├── motor-commands.py               # Publish motor commands
│   │   │   ├── trajectory-executor.py          # Execute joint trajectories
│   │   │   └── contact-monitor.py              # Monitor contact forces
│   │   │
│   │   ├── 06-plugins/
│   │   │   ├── CMakeLists.txt                  # Build configuration
│   │   │   ├── custom-physics-plugin.cpp       # Example world physics plugin
│   │   │   ├── contact-callback-plugin.cpp     # Contact event handling
│   │   │   ├── wind-plugin.cpp                 # Environmental effect plugin
│   │   │   ├── plugin-loader.cpp               # Plugin loading framework
│   │   │   └── BUILD_INSTRUCTIONS.md           # How to compile plugins
│   │   │
│   │   ├── 07-advanced/
│   │   │   ├── humanoid-locomotion.world       # Complex walking scenario
│   │   │   ├── humanoid-manipulation.world     # Humanoid picking up objects
│   │   │   ├── performance-benchmark.py        # Measure simulation speed
│   │   │   ├── physics-debugging.py            # Advanced physics inspection
│   │   │   └── gazebo-record-replay.py         # Record and replay scenarios
│   │   │
│   │   └── verification/
│   │       ├── gazebo-installation-test.sh     # Verify Gazebo installation
│   │       ├── physics-stability-test.py       # Automated stability verification
│   │       ├── sensor-stream-test.py           # Verify all sensors publish
│   │       ├── ros2-integration-test.py        # Test ROS 2 connection
│   │       └── VERIFICATION_CHECKLIST.md       # Manual QA checklist
│   │
│   └── ch5-unity-simulation/                   # All Unity examples
│       │
│       ├── README.md                           # Overview & setup instructions
│       │
│       ├── Assets/
│       │   │
│       │   ├── Scenes/                         # Unity scenes
│       │   │   ├── RobotSimulation.unity       # Basic humanoid simulation scene
│       │   │   ├── HumanoidObstacle.unity      # Humanoid with obstacles
│       │   │   ├── MultiRobotScene.unity       # Multiple robots
│       │   │   ├── IndoorEnvironment.unity     # Indoor scene (room, furniture)
│       │   │   └── OutdoorEnvironment.unity    # Outdoor scene (terrain, sky)
│       │   │
│       │   ├── Scripts/
│       │   │   │
│       │   │   ├── Core/
│       │   │   │   ├── HumanoidPhysicsSetup.cs   # Configure all humanoid joints
│       │   │   │   ├── MotorController.cs        # Control motors with target angles
│       │   │   │   ├── JointLimitConfiguration.cs # Define per-joint limits
│       │   │   │   └── RigidbodyConfiguration.cs  # Setup rigidbody properties
│       │   │   │
│       │   │   ├── Environment/
│       │   │   │   ├── TerrainBuilder.cs         # Procedural terrain generation
│       │   │   │   ├── EnvironmentBuilder.cs     # Place objects in scene
│       │   │   │   ├── LightingConfiguration.cs  # Set up lighting
│       │   │   │   ├── CameraController.cs       # Multiple camera modes
│       │   │   │   └── WeatherSimulation.cs      # Rain, wind, fog effects
│       │   │   │
│       │   │   ├── Sensors/
│       │   │   │   ├── CameraSimulator.cs        # RGB camera rendering
│       │   │   │   ├── DepthCameraSimulator.cs   # Depth sensor
│       │   │   │   ├── IMUSensor.cs              # Accelerometer + gyroscope
│       │   │   │   ├── LidarSimulator.cs         # Point cloud generation
│       │   │   │   ├── SensorNoise.cs            # Add realistic noise
│       │   │   │   └── SensorManager.cs          # Coordinate multiple sensors
│       │   │   │
│       │   │   ├── ROS2/
│       │   │   │   ├── ROS2Bridge.cs             # TCP/UDP bridge to ROS 2
│       │   │   │   ├── TopicPublisher.cs         # Publish sensor data
│       │   │   │   ├── TopicSubscriber.cs        # Subscribe to commands
│       │   │   │   ├── MessageSerializer.cs      # ROS 2 message format
│       │   │   │   ├── NetworkMonitor.cs         # Connection health check
│       │   │   │   └── ServiceHandler.cs         # ROS 2 service calls
│       │   │   │
│       │   │   ├── Advanced/
│       │   │   │   ├── MultiRobotManager.cs      # Spawn and manage multiple robots
│       │   │   │   ├── BehaviorTree.cs           # AI decision framework
│       │   │   │   ├── RecordingSystem.cs        # Record scenarios
│       │   │   │   ├── PlaybackSystem.cs         # Replay recorded data
│       │   │   │   ├── RLDataCollector.cs        # Collect training data
│       │   │   │   └── HeadlessRunner.cs         # Server/headless mode
│       │   │   │
│       │   │   └── Utilities/
│       │   │       ├── PerformanceMonitor.cs     # FPS and memory tracking
│       │   │       ├── PhysicsDebugger.cs        # Visualize physics state
│       │   │       ├── DataLogger.cs             # Log all simulation data
│       │   │       └── ConfigurationLoader.cs    # Load from JSON/YAML
│       │   │
│       │   ├── Prefabs/                         # Reusable Unity prefabs
│       │   │   ├── Humanoid.prefab              # Complete humanoid robot
│       │   │   ├── EnvironmentObject.prefab     # Generic static object
│       │   │   ├── DynamicObject.prefab         # Movable object
│       │   │   ├── Sensor.prefab                # Sensor component
│       │   │   └── Obstacle.prefab              # Obstacle for navigation
│       │   │
│       │   ├── Materials/                       # Unity materials & textures
│       │   │   ├── GroundMaterial.mat           # Ground/terrain material
│       │   │   ├── HumanoidMaterial.mat         # Robot body material
│       │   │   ├── ObjectMaterial.mat           # Generic object material
│       │   │   └── SkyboxMaterial.mat           # Sky material
│       │   │
│       │   ├── Models/                          # 3D model files
│       │   │   ├── Humanoid.fbx                 # Humanoid model (converted from URDF)
│       │   │   ├── EnvironmentObjects/
│       │   │   │   ├── Cube.fbx
│       │   │   │   ├── Sphere.fbx
│       │   │   │   └── Wall.fbx
│       │   │   └── Terrain.asset                # Unity terrain asset
│       │   │
│       │   ├── Animations/                      # Animation clips (optional)
│       │   │   ├── Walk.anim                    # Walking animation
│       │   │   └── Run.anim                     # Running animation
│       │   │
│       │   └── Resources/                       # Configuration files
│       │       ├── SensorConfig.json            # Sensor calibration data
│       │       ├── PhysicsParams.json           # Physics tuning parameters
│       │       ├── BridgeConfig.yaml            # ROS 2 bridge settings
│       │       └── SceneConfig.json             # Scene initialization data
│       │
│       ├── ProjectSettings/                     # Unity auto-generated settings
│       │   ├── ProjectVersion.txt
│       │   ├── ProjectSettings.asset
│       │   └── ...
│       │
│       ├── Packages/                            # Unity package manifests
│       │   └── manifest.json
│       │
│       └── verification/
│           ├── SETUP_INSTRUCTIONS.md            # Step-by-step Unity setup
│           ├── unity-setup-test.md              # Verify Unity installation
│           ├── physics-verification.cs          # Automated physics tests
│           ├── ros2-bridge-test.py              # ROS 2 connection test
│           ├── sensor-accuracy-test.cs          # Sensor validation
│           └── VERIFICATION_CHECKLIST.md        # Manual QA checklist
│
├── reference/                                   # Cross-module reference docs
│   ├── gazebo-ros2-integration.md               # Gazebo/ROS 2 best practices
│   ├── unity-ros2-bridge.md                     # Unity bridge implementation guide
│   ├── gazebo-vs-unity-comparison.md            # Feature & use-case comparison
│   ├── sensor-data-formats.md                   # ROS 2 message type specifications
│   ├── simulation-debugging-guide.md            # Troubleshooting & debugging
│   ├── physics-tuning-reference.md              # Physics parameter reference
│   └── simulation-performance-guide.md          # Optimization & benchmarking
│
└── (existing Part 1-3 content)
    ├── part1-foundations/
    ├── part2-communication/
    ├── part3-robot-structure/
    └── ... (all existing chapters and reference)
```

---

## File Naming Conventions

### Markdown Chapter Files

**Format**: `NN-kebab-case-title.md`

```
12-gazebo-fundamentals.md
13-physics-simulation.md
14-humanoid-gazebo-world.md
15-gazebo-sensors.md
16-sensor-streaming-ros2.md
17-gazebo-advanced-physics.md
18-unity-fundamentals.md
19-unity-physics-setup.md
20-unity-humanoid-scene.md
21-unity-sensor-simulation.md
22-unity-ros2-bridge.md
23-unity-advanced-features.md
```

### Configuration Files

**Format**: Descriptive name with appropriate extension

```
gazebo-launch-humanoid.launch.xml      # Gazebo launch file
humanoid-physics.world                  # Gazebo world file
humanoid-with-sensors.urdf              # URDF description (symlink from Part 1)
physics-config.yaml                     # Physics parameters
SensorConfig.json                        # Unity sensor configuration
PhysicsParams.json                       # Unity physics parameters
BridgeConfig.yaml                        # ROS 2 bridge configuration
```

### Code Example Files

**Format**: Descriptive name with language extension

**Python** (`*.py`):
```
sensor-publisher.py
joint-controller.py
physics-tuning-tool.py
verify-stability.py
gazebo-record-replay.py
```

**C++** (`*.cpp`, `*.h`):
```
custom-physics-plugin.cpp
contact-callback-plugin.cpp
wind-plugin.cpp
plugin-loader.cpp
CMakeLists.txt
```

**C#** (`*.cs`):
```
HumanoidPhysicsSetup.cs
MotorController.cs
ROS2Bridge.cs
CameraSimulator.cs
MultiRobotManager.cs
```

**XML** (`*.xml`):
```
gazebo-empty-world.launch.xml
gazebo-humanoid.launch.xml
```

**SDF** (Gazebo world format):
```
humanoid-physics.world
humanoid-obstacle-course.world
humanoid-outdoor-world.world
```

### Directory Organization Principles

1. **By Topic**: Each chapter has its own folder under `part4-gazebo/` or `part5-unity/`
2. **By Type**: Examples organized into subdirectories (launch, sensors, control, etc.)
3. **By Stage**: Progression from basic (01-) to advanced (07-)
4. **Consistent Naming**: All files use kebab-case or PascalCase (C# convention)
5. **Verification**: Each example folder includes verification scripts/checklists

---

## File Size Estimates

### Markdown Chapters (Module 2)

| Chapter | Est. Words | Est. KB | Content Type |
|---------|-----------|--------|--------------|
| 12-gazebo-fundamentals | 4000 | 25 | Theory + 3 examples |
| 13-physics-simulation | 5000 | 30 | Theory + 4 examples |
| 14-humanoid-gazebo-world | 3500 | 22 | Theory + 3 examples |
| 15-gazebo-sensors | 4500 | 28 | Theory + 4 examples |
| 16-sensor-streaming-ros2 | 3500 | 22 | Theory + 3 examples |
| 17-gazebo-advanced-physics | 3000 | 18 | Theory + 3 examples |
| 18-unity-fundamentals | 4000 | 25 | Theory + 3 examples |
| 19-unity-physics-setup | 4500 | 28 | Theory + 4 examples |
| 20-unity-humanoid-scene | 3500 | 22 | Theory + 3 examples |
| 21-unity-sensor-simulation | 4000 | 25 | Theory + 4 examples |
| 22-unity-ros2-bridge | 4500 | 28 | Theory + 4 examples |
| 23-unity-advanced-features | 3500 | 22 | Theory + 3 examples |
| **Subtotal** | **48000** | **295** | **12 chapters** |

### Reference Documentation

| Document | Est. Words | Est. KB |
|----------|-----------|--------|
| gazebo-ros2-integration.md | 5000 | 30 |
| unity-ros2-bridge.md | 4000 | 25 |
| gazebo-vs-unity-comparison.md | 3000 | 18 |
| sensor-data-formats.md | 2500 | 15 |
| simulation-debugging-guide.md | 4000 | 25 |
| physics-tuning-reference.md | 2500 | 15 |
| simulation-performance-guide.md | 2000 | 12 |
| **Subtotal** | **23000** | **140** | **7 reference docs** |

### Code Examples (Gazebo)

| Category | Files | Est. Lines | Est. KB |
|----------|-------|-----------|--------|
| Launch files | 3 | 150 | 10 |
| World files | 5 | 500 | 25 |
| Python scripts | 10 | 2000 | 80 |
| C++ plugins | 4 | 1500 | 60 |
| Configuration | 3 | 200 | 8 |
| **Subtotal** | **25** | **4350** | **183** |

### Code Examples (Unity)

| Category | Files | Est. Lines | Est. KB |
|----------|-------|-----------|--------|
| C# scripts | 20 | 4000 | 160 |
| Unity scenes | 5 | 2000 | 80 |
| Prefabs | 5 | 1000 | 40 |
| Materials | 4 | 200 | 8 |
| Configuration | 4 | 400 | 16 |
| **Subtotal** | **38** | **7600** | **304** |

### Total Module 2 Content

```
Documentation (chapters + reference): 295 + 140 = 435 KB (71,000 words)
Gazebo examples: 183 KB (4,350 LOC)
Unity examples: 304 KB (7,600 LOC)
────────────────────────────────
Total: ~922 KB (11,950 LOC documented examples)
```

---

## Implementation Checklist

### Directory Structure Setup

- [ ] Create `docs/part4-gazebo-simulation/` folder
- [ ] Create `docs/part5-unity-simulation/` folder
- [ ] Create `docs/examples/ch4-gazebo-simulation/` with subdirectories
- [ ] Create `docs/examples/ch5-unity-simulation/` with subdirectories
- [ ] Create `docs/reference/` subdirectories
- [ ] Create `.gitkeep` files in empty directories for Git tracking

### Chapter Files

**Gazebo** (Chapters 12-17):
- [ ] 12-gazebo-fundamentals.md
- [ ] 13-physics-simulation.md
- [ ] 14-humanoid-gazebo-world.md
- [ ] 15-gazebo-sensors.md
- [ ] 16-sensor-streaming-ros2.md
- [ ] 17-gazebo-advanced-physics.md

**Unity** (Chapters 18-23):
- [ ] 18-unity-fundamentals.md
- [ ] 19-unity-physics-setup.md
- [ ] 20-unity-humanoid-scene.md
- [ ] 21-unity-sensor-simulation.md
- [ ] 22-unity-ros2-bridge.md
- [ ] 23-unity-advanced-features.md

### Example Files

**Gazebo Examples**:
- [ ] Launch files (gazebo-launch-*.xml)
- [ ] World files (*.world)
- [ ] Python scripts (*.py)
- [ ] C++ plugins (*.cpp, CMakeLists.txt)
- [ ] Verification scripts

**Unity Examples**:
- [ ] C# scripts (*.cs)
- [ ] Scene files (*.unity)
- [ ] Prefabs (*.prefab)
- [ ] Configuration files (*.json, *.yaml)
- [ ] Verification scripts

### Reference Documentation

- [ ] gazebo-ros2-integration.md
- [ ] unity-ros2-bridge.md
- [ ] gazebo-vs-unity-comparison.md
- [ ] sensor-data-formats.md
- [ ] simulation-debugging-guide.md
- [ ] physics-tuning-reference.md
- [ ] simulation-performance-guide.md

### Docusaurus Configuration

- [ ] Update `sidebars.js` to include Module 2 chapters
- [ ] Update `docusaurus.config.js` if needed (customization)
- [ ] Create navigation labels for Part 4 & 5
- [ ] Test sidebar generation and links

### Verification & Testing

- [ ] Markdown lint all chapter files
- [ ] Test all code examples on clean systems
- [ ] Verify all cross-references are valid
- [ ] Build Docusaurus site with all new content
- [ ] Check for broken links in generated HTML

---

## Integration with Existing Project

### Docusaurus Sidebar Update

**Current** (`sidebars.js`):
```javascript
'part3-robot-structure': [
  'part3-robot-structure/09-urdf-fundamentals',
  'part3-robot-structure/10-humanoid-urdf-example',
  'part3-robot-structure/11-rviz-gazebo-integration',
],
```

**New** (with Module 2):
```javascript
'part3-robot-structure': [
  'part3-robot-structure/09-urdf-fundamentals',
  'part3-robot-structure/10-humanoid-urdf-example',
  'part3-robot-structure/11-rviz-gazebo-integration',
],
'part4-gazebo-simulation': [
  'part4-gazebo-simulation/12-gazebo-fundamentals',
  'part4-gazebo-simulation/13-physics-simulation',
  'part4-gazebo-simulation/14-humanoid-gazebo-world',
  'part4-gazebo-simulation/15-gazebo-sensors',
  'part4-gazebo-simulation/16-sensor-streaming-ros2',
  'part4-gazebo-simulation/17-gazebo-advanced-physics',
],
'part5-unity-simulation': [
  'part5-unity-simulation/18-unity-fundamentals',
  'part5-unity-simulation/19-unity-physics-setup',
  'part5-unity-simulation/20-unity-humanoid-scene',
  'part5-unity-simulation/21-unity-sensor-simulation',
  'part5-unity-simulation/22-unity-ros2-bridge',
  'part5-unity-simulation/23-unity-advanced-features',
],
```

### Example Organization in Book

All examples will be referenced in chapters with links:

```markdown
## Example: Setting Up Gazebo with Humanoid

See complete code: [`docs/examples/ch4-gazebo-simulation/01-basic-launch/`](../../examples/ch4-gazebo-simulation/01-basic-launch/)
```

---

**Status**: ✅ Directory structure and file organization complete and ready for implementation

