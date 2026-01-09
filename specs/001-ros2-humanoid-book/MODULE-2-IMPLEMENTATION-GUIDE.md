# Module 2: Implementation Quick Start Guide

**Created**: 2026-01-08
**Purpose**: Step-by-step guide to begin implementing Module 2
**Audience**: Development team ready to start writing chapters

---

## Pre-Implementation Checklist

Before you start writing content, verify these are complete:

### Prerequisites
- [ ] Read `module-2-plan.md` (executive summary + one chapter breakdown)
- [ ] Review `module-2-structure.md` (directory structure you'll be creating)
- [ ] Approved by project stakeholders (SDD governance requirement)
- [ ] Spec document created in `specs/001-ros2-humanoid-book/module-2-spec.md`
- [ ] Tasks generated via `/sp.tasks` command

### Development Environment
- [ ] Ubuntu 22.04 LTS VM or machine (for Gazebo examples)
- [ ] ROS 2 Humble/Jazzy installed & tested
- [ ] Gazebo simulation installed & verified
- [ ] Unity 2022 LTS+ with C# development setup
- [ ] Git repository cloned and branch `001-ros2-humanoid-book` active
- [ ] Local Docusaurus build tested (`npm start` works)

### Tools & Configuration
- [ ] Code editor/IDE ready (VS Code, PyCharm, Visual Studio, etc.)
- [ ] GitHub account with access to repository
- [ ] Pre-commit hooks configured (if used by project)
- [ ] Markdown linter configured (if required)

---

## Phase-by-Phase Implementation

### PHASE 1: Setup Directory Structure (1-2 hours)

**Goal**: Create all directories and placeholder files for Module 2

**Steps**:

1. **Create Gazebo chapters directory**
   ```bash
   mkdir -p docs/part4-gazebo-simulation
   touch docs/part4-gazebo-simulation/{12,13,14,15,16,17}-*.md
   ```

2. **Create Unity chapters directory**
   ```bash
   mkdir -p docs/part5-unity-simulation
   touch docs/part5-unity-simulation/{18,19,20,21,22,23}-*.md
   ```

3. **Create examples directories**
   ```bash
   mkdir -p docs/examples/ch4-gazebo-simulation/{01-basic-launch,02-physics-tuning,03-environment,04-sensors,05-control,06-plugins,07-advanced,verification}
   mkdir -p docs/examples/ch5-unity-simulation/{Assets/{Scenes,Scripts/{Core,Environment,Sensors,ROS2,Advanced,Utilities},Prefabs,Materials,Models,Animations,Resources},ProjectSettings,Packages,verification}
   ```

4. **Create reference directory**
   ```bash
   mkdir -p docs/reference
   touch docs/reference/{gazebo-ros2-integration,unity-ros2-bridge,gazebo-vs-unity-comparison,sensor-data-formats,simulation-debugging-guide,physics-tuning-reference,simulation-performance-guide}.md
   ```

5. **Initialize chapter files with templates**
   ```bash
   # For each chapter file, copy the template from MODULE-2-SUMMARY.md Appendix
   # Create placeholder content with structure
   ```

**Deliverable**: All directories exist, all markdown chapter files created with template headers

**Verification**:
```bash
find docs/part4-gazebo-simulation docs/part5-unity-simulation -type f | wc -l
# Should show 12 markdown files (6 each)

find docs/examples/ch4-gazebo-simulation docs/examples/ch5-unity-simulation -type d | wc -l
# Should show 18+ subdirectories
```

---

### PHASE 2: Write Gazebo Fundamentals (Chapters 12-13) (1-2 weeks)

**Goal**: Establish Gazebo introduction and physics foundations

#### Chapter 12: Gazebo Fundamentals (3-4 days)

**Structure**:
```
12-gazebo-fundamentals.md (4000 words)
├── Learning Outcomes
├── Prerequisites
├── What is Gazebo?
│   ├── History & context
│   ├── Versions (classic vs Gazebo sim)
│   ├── Why Gazebo for ROS 2?
├── Installation on Ubuntu 22.04
│   ├── Prerequisites
│   ├── Step-by-step installation
│   ├── Verification
├── Gazebo GUI Walkthrough
│   ├── Main window layout
│   ├── Scene tree & properties
│   ├── Visualization controls
├── Loading a URDF Model
│   ├── URDF format review
│   ├── Gazebo world files (SDF)
│   ├── Spawning models
├── Launch Files Introduction
│   ├── XML format
│   ├── ros2_launch vs legacy
│   ├── Example launch file
├── Gazebo Plugin Architecture
│   ├── Plugin types (World, Model, Sensor)
│   ├── Built-in vs custom plugins
├── ROS 2 Integration Overview
│   ├── ros2_control framework
│   ├── gz_ros2_control
│   ├── Gazebo/ROS 2 workflow
└── Summary & Exercises
```

**Code Examples** (create in `ch4-gazebo-simulation/01-basic-launch/`):
- `gazebo-empty-world.launch.xml` — Minimal launch file
- `gazebo-humanoid.launch.xml` — Launch with humanoid from Module 1
- `spawn-model.py` — Python script to spawn URDF at runtime
- `verify-gazebo.sh` — Verification script

**Acceptance Criteria**:
- ✅ Chapter follows 3-part template (intro → concepts → examples)
- ✅ All code examples tested on clean Ubuntu 22.04
- ✅ All claims cite official Gazebo/ROS 2 documentation
- ✅ Readers can launch Gazebo and load humanoid URDF
- ✅ Sidebars.js updated with chapter reference

**Time Estimate**: 3-4 days (writing + testing)

---

#### Chapter 13: Physics Simulation (4-5 days)

**Structure**:
```
13-physics-simulation.md (5000 words)
├── Learning Outcomes
├── Prerequisites
├── Physics Engine Selection
│   ├── ODE (default, stable)
│   ├── Bullet (fast, less stable)
│   ├── DART (advanced)
│   ├── Comparison table
├── Physics Configuration Parameters
│   ├── Gravity & world forces
│   ├── Step size & solver iterations
│   ├── Contact parameters
├── Joint Types & Dynamics
│   ├── Revolute joints (rotary)
│   ├── Prismatic joints (sliding)
│   ├── Fixed & ball joints
│   ├── Joint dynamics equations
├── Mass & Inertia Configuration
│   ├── Link mass properties
│   ├── Inertia matrices
│   ├── Center of gravity placement
├── Friction & Contact Modeling
│   ├── Surface friction (mu)
│   ├── Contact stiffness & damping
│   ├── Rolling & spinning friction
├── Joint Limits & Damping
│   ├── Position limits
│   ├── Velocity limits
│   ├── Friction/damping parameters
├── Physics Debugging Tools
│   ├── Contact visualization
│   ├── Force/torque display
│   ├── Center of mass markers
├── Performance Tuning
│   ├── Step size impact
│   ├── Solver iterations
│   ├── Real-time factor measurement
└── Summary & Exercises
```

**Code Examples** (create in `ch4-gazebo-simulation/02-physics-tuning/`):
- `humanoid-physics.world` — World file with physics configuration
- `physics-config.yaml` — Parameter reference file
- `physics-tuning-script.py` — Interactive parameter adjustment tool
- `contact-visualizer.py` — Subscribe to contacts and visualize in RViz
- `stability-test.py` — Verify humanoid physics stability
- `joint-dynamics-demo.py` — Demonstrate joint control with torques

**Acceptance Criteria**:
- ✅ Physics concepts explained with mathematical equations where relevant
- ✅ Example humanoid can be dropped and settles stably
- ✅ Joint limits prevent unrealistic motion
- ✅ Physics runs at real-time speed (sim time ≈ wall time)
- ✅ Friction/damping parameters produce realistic behavior

**Time Estimate**: 4-5 days (writing + testing + tuning)

---

### PHASE 3: Write Gazebo Integration (Chapters 14-16) (2-3 weeks)

**Goal**: Complete Gazebo environment & sensor integration

#### Chapter 14: Humanoid Gazebo World (3-4 days)

**Key Examples** (create in `ch4-gazebo-simulation/03-environment/`):
- `humanoid-empty-world.world` — Minimal world (ground plane only)
- `humanoid-obstacle-course.world` — Complex environment with 10+ objects
- `humanoid-outdoor.world` — Terrain with height variations
- `multi-robot.world` — Two humanoids in same environment
- `world-builder.py` — Script to generate worlds programmatically

**Acceptance Criteria**:
- ✅ Humanoid can walk/move without falling through ground
- ✅ Objects render with proper textures and lighting
- ✅ Multi-robot scenario runs without instability
- ✅ World files are valid SDF (no parse errors)

---

#### Chapter 15: Gazebo Sensors (4-5 days)

**Key Examples** (create in `ch4-gazebo-simulation/04-sensors/`):
- `humanoid-with-sensors.urdf` — Symlink to Part 1 with sensor defs
- `sensor-publisher.py` — ROS 2 node publishing all sensor data
- `imu-verification.py` — Verify IMU calibration
- `camera-viewer.py` — Display camera images (subscribe & show in OpenCV)
- `lidar-visualization.py` — Visualize point cloud in RViz
- `sensor-sync.py` — Synchronize multiple sensor streams

**Acceptance Criteria**:
- ✅ `ros2 topic list` shows all configured sensors
- ✅ IMU publishes at 100+ Hz
- ✅ Camera publishes images at 30+ Hz
- ✅ Lidar publishes point clouds at 30+ Hz
- ✅ Joint state publishes at 100+ Hz
- ✅ All messages are valid ROS 2 message types

---

#### Chapter 16: Sensor Streaming to ROS 2 (3-4 days)

**Key Examples** (create in `ch4-gazebo-simulation/05-control/`):
- `sensor-config.yaml` — ROS 2 control configuration
- `sensor-subscriber.py` — Multi-sensor subscriber node
- `tf2-broadcaster.py` — Publish sensor frame transforms
- `topic-monitor.py` — Real-time topic visualization
- `rosbag-recorder.py` — Record all sensors to rosbag file

**Acceptance Criteria**:
- ✅ Multiple sensors stream simultaneously without data loss
- ✅ TF tree correctly shows all sensor frames
- ✅ ROS bag files can record & replay all data
- ✅ Cross-subscriber synchronization works (timestamps matched)

---

### PHASE 4: Write Gazebo Advanced (Chapter 17) (1-2 weeks)

**Goal**: Custom plugins and advanced physics

#### Chapter 17: Advanced Physics & Plugins (5-7 days)

**Key Examples** (create in `ch4-gazebo-simulation/06-plugins/`):
- `CMakeLists.txt` — Build configuration for plugins
- `custom-physics-plugin.cpp` — World plugin with custom physics
- `contact-callback-plugin.cpp` — Monitor & respond to contacts
- `wind-plugin.cpp` — Environmental effects (wind simulation)
- `plugin-performance-benchmark.cpp` — Performance testing framework
- `BUILD_INSTRUCTIONS.md` — Compilation guide

**Acceptance Criteria**:
- ✅ Plugins compile without warnings/errors
- ✅ Custom physics behaviors work as documented
- ✅ Plugin can be toggled on/off without crashing
- ✅ Performance impact is acceptable (real-time factor >0.9)

**Time Estimate**: 5-7 days (plugin development + debugging)

---

### PHASE 5: Write Unity Foundation (Chapters 18-20) (2-3 weeks)

**Goal**: Unity project setup and scene building

#### Chapter 18: Unity Fundamentals (3-4 days)

**Key Examples** (create in `ch5-unity-simulation/Assets/Scripts/Core/`):
- `HumanoidImporter.cs` — Import URDF models
- `ProjectSetup.md` — Step-by-step project initialization
- `PhysicsDebugger.cs` — Visual physics debugging
- `PerformanceMonitor.cs` — Real-time FPS tracking

---

#### Chapter 19: Unity Physics (4-5 days)

**Key Examples** (create in `ch5-unity-simulation/Assets/Scripts/Core/`):
- `HumanoidPhysicsSetup.cs` — Configure all joints
- `JointLimitConfiguration.cs` — Per-joint limit setup
- `MotorController.cs` — Motor control with target angles/velocities
- `PhysicsMaterial.asset` — Pre-configured physics materials

---

#### Chapter 20: Unity Scene Building (4-5 days)

**Key Examples** (create in `ch5-unity-simulation/Assets/Scripts/Environment/`):
- `TerrainBuilder.cs` — Procedural terrain generation
- `EnvironmentBuilder.cs` — Place objects in scene
- `LightingConfiguration.cs` — Advanced lighting setup
- `CameraController.cs` — Multiple camera modes

**Unity Scenes** (create in `Assets/Scenes/`):
- `RobotSimulation.unity` — Basic scene with humanoid
- `HumanoidObstacle.unity` — Scene with obstacles
- `IndoorEnvironment.unity` — Room with furniture
- `OutdoorEnvironment.unity` — Terrain and outdoor

---

### PHASE 6: Write Unity Integration & Advanced (Chapters 21-23) (2-3 weeks)

**Goal**: Sensor simulation, ROS 2 bridge, and advanced features

#### Chapter 21: Sensor Simulation (4-5 days)

**Key Examples** (create in `ch5-unity-simulation/Assets/Scripts/Sensors/`):
- `CameraSimulator.cs` — RGB camera rendering
- `IMUSensor.cs` — IMU simulation with noise
- `LidarSimulator.cs` — Raycast-based point cloud
- `SensorNoise.cs` — Noise injection
- `SensorManager.cs` — Multi-sensor coordination

---

#### Chapter 22: ROS 2 Bridge (5-6 days)

**Key Examples** (create in `ch5-unity-simulation/Assets/Scripts/ROS2/`):
- `ROS2Bridge.cs` — TCP/UDP bridge implementation
- `TopicPublisher.cs` — Sensor data publishing
- `TopicSubscriber.cs` — Command subscription
- `MessageSerialization.cs` — ROS 2 message formatting
- `NetworkMonitor.cs` — Connection health monitoring
- `SimulationController.py` — ROS 2 control node (Python reference)

**Configuration** (create in `Assets/Resources/`):
- `BridgeConfig.yaml` — Topic mapping & network settings
- `SensorConfig.json` — Per-sensor calibration

---

#### Chapter 23: Advanced Features (5-7 days)

**Key Examples** (create in `ch5-unity-simulation/Assets/Scripts/Advanced/`):
- `MultiRobotManager.cs` — Spawn multiple instances
- `BehaviorTree.cs` — AI decision framework
- `RecordingSystem.cs` — Record/replay scenarios
- `RLDataCollector.cs` — RL training data extraction
- `HeadlessRunner.cs` — Headless/server mode

**Deliverables**:
- Ubuntu builds (headless mode)
- Linux builds
- Documentation for deployment

---

## Writing Guidelines

### For Each Chapter

**Structure** (follow template from Module 1):
1. **Learning Outcomes** (3-5 bullet points)
2. **Prerequisites** (what readers need to know)
3. **Main Content** (5-10 sections with progressive depth)
4. **Code Examples** (inline or linked)
5. **Summary & Next Steps**
6. **Further Reading** (links to official docs)

**Content Quality Checklist**:
- [ ] ≥90% code examples include inline comments
- [ ] All technical claims cite authoritative sources
- [ ] Cross-references to other chapters are accurate
- [ ] No marketing language, only technical explanation
- [ ] Images/diagrams where helpful (architecture, data flow)

**Code Example Standards**:
- [ ] Copy-paste ready (all imports included)
- [ ] Runnable on clean system without modification
- [ ] Includes expected output examples
- [ ] Comments explain non-obvious behavior
- [ ] Error handling for common issues
- [ ] Links to official documentation

---

## Writing Workflow

### For Each Chapter (Daily Process)

```
Day 1: Research & Outline
  - Read official documentation (Gazebo/Unity docs)
  - Create detailed section outline
  - Identify 3-5 code examples needed
  - Create example code structure (scaffolding)

Day 2: Write Theory & Concepts
  - Write main chapter content (sections 1-7)
  - Add diagrams/visuals where needed
  - Create cross-references
  - Cite authoritative sources

Day 3: Write Examples & Code
  - Complete all code examples
  - Test on clean system
  - Add expected output
  - Document common errors

Day 4: Review & Polish
  - Self-review against template
  - Verify all links work
  - Check grammar & clarity
  - Update glossary with new terms

Day 5: QA & Integration
  - Run markdown lint
  - Build Docusaurus locally
  - Verify sidebar entry
  - Final verification checklist
```

---

## Key Milestones & Deliverables

### Milestone 1: Gazebo Complete (End of Phase 3)
- [ ] Chapters 12-17 (complete & reviewed)
- [ ] 25 example files (all tested)
- [ ] All Gazebo topics working in ROS 2
- [ ] Docusaurus builds without errors
- [ ] Verification passed on 2 clean systems

### Milestone 2: Unity Complete (End of Phase 6)
- [ ] Chapters 18-23 (complete & reviewed)
- [ ] 38 example files (all tested)
- [ ] ROS 2 bridge working (publish/subscribe)
- [ ] Multi-robot scenes functional
- [ ] Docusaurus builds with all 12 chapters

### Final Milestone: Module 2 Complete
- [ ] All 12 chapters ✅
- [ ] 7 reference documents ✅
- [ ] 63 code examples ✅
- [ ] Full Docusaurus build ✅
- [ ] Glossary updated ✅
- [ ] All verification checklists passed ✅

---

## Testing & Verification

### For Each Chapter (Before Publishing)

**Manual Testing**:
1. [ ] All code examples run on Ubuntu 22.04 (Gazebo) or development machine (Unity)
2. [ ] Example output matches expected output in documentation
3. [ ] No errors or warnings during execution
4. [ ] Gazebo windows render correctly
5. [ ] ROS 2 topics publish with correct frequencies

**Documentation Testing**:
1. [ ] Markdown syntax valid (lint passes)
2. [ ] All links work (internal & external)
3. [ ] Cross-references accurate
4. [ ] All images/diagrams render
5. [ ] Code blocks syntax-highlighted correctly

**Docusaurus Integration**:
1. [ ] `npm start` builds successfully
2. [ ] Chapter appears in sidebar
3. [ ] Table of contents auto-generates correctly
4. [ ] Search works for chapter content
5. [ ] Mobile layout renders correctly

---

## Common Issues & Troubleshooting

### Gazebo Issues

**Problem**: Gazebo won't start
**Solution**:
```bash
# Verify ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 pkg list | grep gazebo

# Install if missing
sudo apt install ros-humble-gz-sim
```

**Problem**: Humanoid URDF won't load
**Solution**:
```bash
# Check URDF syntax
check_urdf docs/part1-foundations/humanoid-robot.urdf

# Verify paths in world file
grep -n "filename=" yourfile.world
# Update paths if absolute instead of relative
```

**Problem**: Sensors not publishing
**Solution**:
```bash
# Check plugin configuration in URDF
# Verify topic names in gazebo plugin blocks

# Monitor topics
ros2 topic list
ros2 topic echo /gazebo/sensor/imu_data
```

### Unity Issues

**Problem**: Script compilation errors
**Solution**:
- Check Unity version (should be 2022 LTS+)
- Verify C# language version in ProjectSettings
- Check for missing using statements

**Problem**: Physics instability
**Solution**:
- Reduce physics step size (Edit > Project Settings > Physics)
- Increase solver iterations
- Check rigidbody mass values (should be 0.1-100 kg range)

**Problem**: ROS 2 bridge not connecting
**Solution**:
- Verify firewall allows TCP/UDP ports
- Check bridge IP address (localhost vs 127.0.0.1)
- Monitor network with Wireshark if needed

---

## Team Coordination

### Writing Assignments (Example)

```
Writer A: Chapters 12-13 (Gazebo fundamentals & physics)
Writer B: Chapters 14-16 (Gazebo environments & sensors)
Writer C: Chapter 17 (Advanced Gazebo)

Writer D: Chapters 18-19 (Unity fundamentals & physics)
Writer E: Chapters 20-22 (Unity scene building & ROS 2 bridge)
Writer F: Chapter 23 (Advanced Unity)

Reviewer: Review & integrate all chapters
Technical Lead: Verify physics/networking accuracy
```

### Coordination Points

- **Weekly syncs**: 30-min check-in on progress & blockers
- **Shared testing**: All code examples tested independently & by team
- **Cross-review**: Each chapter reviewed by another writer
- **Integration testing**: Full Docusaurus build & link verification weekly

---

## Success Metrics

By end of implementation:

```
Documentation Quality
  ✅ 100% code examples run without modification
  ✅ 100% technical claims cited
  ✅ 90%+ code examples include comments
  ✅ 100% cross-references accurate

Functionality
  ✅ Gazebo examples: physics real-time, sensors publishing
  ✅ Unity examples: 60+ FPS, ROS 2 bridge working
  ✅ All verification scripts pass

Reproducibility
  ✅ Setup time: ≤60 min per platform
  ✅ Clean system test: all examples work
  ✅ Troubleshooting guide: covers ≥90% issues

Docusaurus Integration
  ✅ Full build: no errors
  ✅ Navigation: all chapters accessible
  ✅ Search: chapter content discoverable
  ✅ Mobile: layouts responsive
```

---

## Getting Help

**Questions about**:
- **Gazebo**: Check official Gazebo documentation, ROS 2 forums
- **Unity**: Check Unity documentation, C# references
- **ROS 2**: Check ROS 2 documentation, ROS Discourse
- **Project Structure**: Review Module 1 chapters for established patterns
- **SDD Process**: Check `/CLAUDE.md` for project rules

---

## Start Implementation

**When you're ready to begin**:

1. **Confirm prerequisites above are met**
2. **Create directory structure** (Phase 1, 1-2 hours)
3. **Start Phase 2** with Chapter 12 (estimated 3-4 days)
4. **Follow phases sequentially** (6 weeks total, ~1 chapter/week)
5. **Update progress** on shared tracking system
6. **Test continuously** as you write

**First Task**: Create all directories & placeholder files from Phase 1 above.

---

**Status**: ✅ Ready for Implementation
**Estimated Duration**: 6 weeks (sequential), 3 weeks (parallel with 3 writers)
**Next Action**: Begin Phase 1 directory structure setup

