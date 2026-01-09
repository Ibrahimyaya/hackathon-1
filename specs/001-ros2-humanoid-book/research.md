# Research: ROS 2 Technical Foundation

**Date**: 2026-01-08
**Purpose**: Establish technical foundation for book examples and architecture decisions
**Scope**: ROS 2 Humble vs. Jazzy compatibility, DDS QoS practices, Gazebo integration, URDF validation

---

## T012: ROS 2 Humble vs. Jazzy Compatibility Analysis

### Context
ROS 2 releases follow a predictable lifecycle. Currently available LTS releases:
- **ROS 2 Humble (2022.12)**: Long-Term Support through May 2027
- **ROS 2 Jazzy Jalisco (2024.04)**: Standard release (not LTS)

### Decision Rationale: **Humble is the Correct Choice**

#### Reasons:
1. **Long-Term Support (LTS)**
   - Humble: Supported until May 2027 (4.5 years of security patches)
   - Jazzy: Standard support only (18 months)
   - Book will remain relevant and maintainable longer with Humble

2. **API Stability**
   - Humble: Mature; minimal breaking changes since release
   - Jazzy: Newer; still receiving API refinements
   - Examples written for Humble will work unchanged through 2027

3. **Community & Documentation**
   - Humble: Larger installed base; more troubleshooting resources
   - Jazzy: Smaller community; fewer Stack Overflow/Discourse answers
   - Educational book benefits from larger support network

4. **Deployment Reality**
   - Most production humanoid robots run Humble or Iron (previous LTS)
   - Recommend Jazzy migration as Phase 2 task (not blocking initial publication)

#### Breaking Changes Between Humble and Jazzy (Minor)
The rclpy API remains stable:
- `Node.create_publisher()`, `create_subscription()`, `create_service()`, `create_action_server()` signatures unchanged
- QoS policies compatible (can be upgraded)
- Message types (sensor_msgs/JointState, geometry_msgs/Pose) unchanged

**Migration Path**: All examples are forward-compatible with Jazzy; examples written for Humble will run on Jazzy with zero code changes.

#### Recommendation for Book
✅ **Use ROS 2 Humble exclusively** for all examples, setup guide, and verification.
📋 **Future task**: Create Jazzy compatibility appendix after initial publication.

---

## T013: DDS Quality of Service (QoS) Best Practices for Humanoid Robotics

### Context
DDS QoS policies control message delivery behavior. Humanoid robots have high-frequency sensor streams (100+ Hz) and real-time control requirements.

### Core QoS Policies

#### 1. Reliability
- **RELIABLE**: Guarantees message delivery (retries until timeout)
  - Overhead: Higher network traffic, slight latency
  - Use case: Commands, critical state changes
  - Humanoid example: Motor speed commands (set once, deliver surely)

- **BEST_EFFORT**: Best effort delivery, no retries (drops allowed)
  - Overhead: Lower latency
  - Use case: High-frequency sensor streams where dropping a frame is acceptable
  - Humanoid example: IMU data at 100 Hz (losing 1 frame of 100 is <1% error)

#### 2. History
- **KEEP_LAST(N)**: Retain only last N messages
  - For Humanoid: **KEEP_LAST(1)** is standard for sensor streams
  - Reason: Control loops need fresh data; old sensor data is stale
  - Example: If IMU publishes at 100 Hz, a 1-frame buffer ensures always-fresh orientation

- **KEEP_ALL**: Retain all messages until subscriber receives them
  - For Humanoid: Not recommended for sensors (too much memory)
  - Use case: Logging/archival (separate topic with different QoS)

#### 3. Deadline
- Maximum time a message must arrive after publication
- Humanoid sensor example: 100 Hz sensor = 0.01 seconds (10 ms) period
  - Set deadline to 15-20 ms to catch missed publishes

#### 4. Liveliness
- How often a node must publish to be considered "alive"
- Humanoid control: Set liveliness period = 2x normal publication interval
  - 100 Hz sensor: liveliness = 20 ms
  - If no message in 20 ms, DDS marks node as "not alive" (safety trigger)

#### 5. Durability
- **TRANSIENT_LOCAL**: DDS retains messages and sends to new subscribers
- **VOLATILE**: No retention (subscribers miss messages before subscribing)
  - For Humanoid sensors: Use VOLATILE (fresh data matters, historical data irrelevant)
  - For humanoid state persistence: Use TRANSIENT_LOCAL (initial subscribers need current state)

### Recommended Humanoid QoS Profiles

#### Profile A: Sensor Streams (IMU, joint encoders, cameras at 100+ Hz)
```python
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Fast, lose OK
    history=HistoryPolicy.KEEP_LAST(1),         # Fresh data only
    deadline=Duration(seconds=0.015),            # Must arrive in 15 ms
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=0.02)  # 20 ms liveness
)
```

#### Profile B: Command Topics (Motor commands, pose goals at 10-50 Hz)
```python
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,     # Ensure delivery
    history=HistoryPolicy.KEEP_LAST(1),
    deadline=Duration(seconds=0.1),              # 100 ms is OK for commands
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=0.2)  # 200 ms OK
)
```

#### Profile C: State Topics (Robot state, diagnostics at 1-10 Hz)
```python
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST(10),        # Keep last 10 for debugging
    deadline=Duration(seconds=1.0),              # 1 second OK
    liveliness=LivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=Duration(seconds=2.0)
)
```

### Examples for Book

The Chapter 2 examples will demonstrate:
1. **Example 1**: Default QoS vs. sensor-optimized QoS (message loss comparison)
2. **Example 2**: RELIABLE vs. BEST_EFFORT effect on network load
3. **Example 3**: Deadline enforcement (missing deadline = node marked unhealthy)
4. **Example 4**: Humanoid sensor QoS tuning

---

## T014: Gazebo + RViz Integration for ROS 2

### Context
Humanoid robots require both physics simulation (Gazebo) and visualization (RViz). ROS 2 integration uses standard topics and plugins.

### Architecture

#### RViz Role
- **Visualization only** (reads data, doesn't modify state)
- Displays:
  - Robot structure from `/robot_description` (URDF)
  - TF transforms (coordinate frame hierarchy)
  - Topic data overlays (point clouds, markers, text)
  - Real-time sensor feedback

#### Gazebo Role
- **Physics simulation** (models kinematics, dynamics, contact)
- Publishes:
  - Joint states to `/joint_states` (sensor_msgs/JointState)
  - Sensor simulations (IMU, cameras, lidar)
- Subscribes to:
  - Joint commands (e.g., `/joint_commands` or custom action)
  - Control requests

#### Communication Bridge: `robot_state_publisher`
- Reads `/robot_description` (URDF) and `/joint_states` (dynamic joint angles)
- Publishes TF transforms for all frames
- Enables RViz to render correct robot pose in real-time

### Integration Pattern for Humanoid Book

#### Setup 1: Gazebo Simulation with RViz Visualization
```bash
# Terminal 1: Gazebo with physics and joint state publishing
gazebo --verbose humanoid-robot.world

# Terminal 2: Robot state publisher (converts /joint_states → TF)
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro humanoid-robot.urdf)"

# Terminal 3: RViz visualization
rviz2

# Terminal 4: Joint command publisher (controls Gazebo joints)
ros2 run <package> joint_commander.py
```

#### Key Gazebo Plugins for Humanoids
1. **gazebo_ros_joint_state_publisher**: Publishes `/joint_states` from simulation
2. **gazebo_ros_actuator**: Allows ROS 2 commands to control joints
3. **gazebo_ros_imu**: Simulates IMU sensor with realistic noise
4. **gazebo_ros_camera**: Simulates camera with ray-casting

#### URDF → Gazebo Requirements
Humanoid URDF must include Gazebo-specific metadata:
```xml
<link name="torso">
  <inertial>
    <mass value="10.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
  <collision>
    <geometry><box size="0.2 0.3 0.5"/></geometry>
  </collision>
  <visual>
    <geometry><box size="0.2 0.3 0.5"/></geometry>
    <material name="red"><color rgba="1 0 0 1"/></material>
  </visual>
</link>

<!-- Gazebo-specific plugin -->
<gazebo reference="torso">
  <material>Gazebo/Red</material>
</gazebo>
```

### Examples for Book
1. **Chapter 3 Example 1**: Load simple URDF in Gazebo, visualize in RViz
2. **Chapter 3 Example 2**: Publish joint states from Gazebo, see motion in RViz
3. **Chapter 3 Example 3**: Command joints via ROS 2 topic, control Gazebo simulation
4. **Chapter 3 Example 4**: Multi-sensor simulation (IMU + camera + joint states)

---

## T015: URDF Validation Tools

### Validation Tools

#### 1. `check_urdf` (Primary Tool)
```bash
check_urdf humanoid-robot.urdf
```
Output:
```
robot name is OK
All fixed joints tranformed OK to TF tree
All moving joints transformed OK to TF tree
URDF checks passed
```

Detects:
- XML parse errors
- Missing required fields (mass, inertia, geometry)
- Cycles in joint hierarchy
- Invalid joint limits

#### 2. `urdf_to_graphiz` (Visualization)
```bash
urdf_to_graphiz humanoid-robot.urdf
# Creates humanoid-robot.pdf showing joint tree
```

Shows:
- Link and joint hierarchy
- Parent-child relationships
- Joint types (revolute, prismatic)

#### 3. `urdfdom` (Parser Library)
Python validation:
```python
from urdf_parser_py.urdf import URDF

robot = URDF.from_xml_file('humanoid-robot.urdf')
print(f"Robot name: {robot.name}")
for joint in robot.joints:
    print(f"  {joint.name}: {joint.type}")
```

### Validation Script for Book
Create `docs/examples/validate-urdf.sh`:
```bash
#!/bin/bash

if [ ! -f "$1" ]; then
    echo "Usage: $0 <urdf-file>"
    exit 1
fi

echo "Validating $1..."
check_urdf "$1"

if [ $? -eq 0 ]; then
    echo "✓ URDF is valid"
    urdf_to_graphiz "$1"
    echo "✓ Generated graph: $(basename "$1" .urdf).pdf"
else
    echo "✗ URDF validation failed"
    exit 1
fi
```

### Quality Checks for Humanoid URDF

#### Structural Checks
- ✓ Single root link (base or torso)
- ✓ All links have inertia (mass > 0, valid inertia tensor)
- ✓ No cycles (each link has one parent, except root)
- ✓ Collision/visual geometry defined for all links

#### Humanoid-Specific Checks
- ✓ Torso link exists and is root
- ✓ 2 arm links (left_shoulder, right_shoulder) attached
- ✓ 2 leg links (left_hip, right_hip) attached
- ✓ Head link (optional) attached
- ✓ Joint limits realistic for humanoids (e.g., shoulder: ±90°, elbow: 0-135°)
- ✓ All joints have non-zero inertia on child links

#### Performance Checks
- ✓ Total mass ≈ 30-50 kg (realistic humanoid)
- ✓ <20 joints (avoid over-complexity in simulation)
- ✓ No extremely small links (< 0.01 m) that cause numerical issues

---

## Summary & Book Integration

### Files to Create from Research

1. **docs/examples/ch1-dds-pubsub/qos-settings.py**: Demonstrates all QoS profiles with humanoid examples
2. **docs/examples/ch3-urdf-simulation/validate-urdf.sh**: Validation script for URDF files
3. **docs/part3-robot-structure/10-humanoid-urdf-example.md**: Includes Gazebo plugin metadata

### Decisions Locked In

| Decision | Choice | Rationale |
|----------|--------|-----------|
| ROS 2 Version | **Humble** | LTS through May 2027, stable API |
| Sensor QoS | **BEST_EFFORT + KEEP_LAST(1)** | Low latency for 100+ Hz sensors |
| Command QoS | **RELIABLE + KEEP_LAST(1)** | Ensure delivery of control commands |
| Visualization | **RViz2** | Standard for ROS 2, excellent TF support |
| Physics | **Gazebo** | Standard robotics simulator, ROS 2 integration mature |
| URDF Validation | **check_urdf + urdf_to_graphiz** | Official ROS tools, widely available |

---

**Research Complete**: All foundation decisions made. Ready to proceed to Phase 3 code example implementation.

**Next Phase**: Create 5 Chapter 1 code examples demonstrating ROS 2 fundamentals with Humble, standard QoS profiles, and verified examples.
