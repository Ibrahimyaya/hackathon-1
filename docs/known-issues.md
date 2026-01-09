---
sidebar_position: 204
---

# Known Issues and Troubleshooting

This page documents known issues when setting up and running examples from this book.

## Installation Issues

### Issue: "bash: ros2: command not found"

**Symptom**: When you run `ros2 --version`, bash says the command is not found.

**Cause**: ROS 2 is not sourced in your current shell session.

**Solution**:
```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

To make this permanent, add it to your shell profile:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Issue: "E: Unable to locate package ros-humble-desktop"

**Symptom**: apt update or apt install fails with "Unable to locate package."

**Cause**: The ROS 2 repository was not properly added or the repository GPG key was not set up.

**Solution**:
1. Verify the repository is configured:
   ```bash
   cat /etc/apt/sources.list.d/ros2.list
   ```

   Should show:
   ```
   deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu jammy main
   ```

2. If missing or incorrect, follow [Setup Guide](./setup-guide.md) Step 2a

3. Update and try install again:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop -y
   ```

---

### Issue: Gazebo fails to start or crashes

**Symptom**: `gazebo` command hangs or crashes with graphics errors.

**Cause**: GPU drivers not properly installed or set up.

**Solution**:
1. Verify GPU drivers are installed:
   ```bash
   nvidia-smi  # For NVIDIA GPUs
   glxinfo | grep "OpenGL vendor"
   ```

2. Start Gazebo with verbose output:
   ```bash
   gazebo --verbose
   ```
   Look for error messages about rendering or OpenGL.

3. Install graphics libraries:
   ```bash
   sudo apt install libgl1-mesa-glx libxrender1 -y
   ```

4. Try again:
   ```bash
   gazebo
   ```

---

## Python Examples Issues

### Issue: "ModuleNotFoundError: No module named 'rclpy'"

**Symptom**: Python script fails with "No module named 'rclpy'."

**Cause**: ROS 2 is not sourced in the Python environment.

**Solution**:
1. Source ROS 2 before running Python:
   ```bash
   source /opt/ros/humble/setup.bash
   python3 example.py
   ```

2. Or, make the Python script auto-source:
   ```python
   import sys
   import os
   # Add ROS 2 to Python path
   sys.path.append('/opt/ros/humble/lib/python3.10/site-packages')
   ```

---

### Issue: Example runs but produces no output

**Symptom**: Script runs but doesn't print anything.

**Cause**: ROS 2 is not sourced, or the node is not spinning.

**Solution**:
1. Verify ROS_DISTRO is set:
   ```bash
   echo $ROS_DISTRO
   ```
   Should output: `humble`

2. If empty, source ROS 2:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Run the script again.

---

### Issue: "TypeError: create_publisher() missing required arguments"

**Symptom**: Python script fails with TypeError about missing arguments.

**Cause**: Incorrect `create_publisher()` syntax. rclpy expects specific arguments.

**Solution**:
Correct syntax:
```python
self.publisher_ = self.create_publisher(
    JointState,           # Message type
    '/joint_states',      # Topic name
    10                    # Queue size
)
```

Check the [rclpy documentation](https://docs.ros2.org/latest/api/rclpy/) for correct method signatures.

---

## Topic and Message Issues

### Issue: "Topic '/joint_states' has no subscribers" (warning)

**Symptom**: Publisher prints a warning about no subscribers.

**Cause**: Normal behavior. No nodes are currently listening to the topic.

**Solution**: This is expected. The warning disappears when a subscriber connects:
```bash
# Terminal 1: Publisher
python3 example_publisher.py

# Terminal 2: Subscriber
ros2 topic echo /joint_states
```

---

### Issue: "Cannot publish message: Message type mismatch"

**Symptom**: ROS 2 complains about message type mismatch.

**Cause**: Publisher and subscriber expect different message types.

**Solution**:
1. Verify message type in publisher:
   ```python
   self.publisher_ = self.create_publisher(JointState, ...)
   ```

2. Verify subscriber uses the same type:
   ```python
   self.subscription = self.create_subscription(
       JointState,  # Must match publisher
       'topic_name',
       self.callback
   )
   ```

3. Check official message definitions: https://docs.ros2.org/latest/api/

---

## Network and Multi-Machine Issues

### Issue: Nodes can't see each other on different machines

**Symptom**: ROS 2 nodes on different computers don't communicate.

**Cause**: ROS 2 DDS discovery not configured for network communication.

**Solution**:
1. Ensure both machines are on same network:
   ```bash
   ping <other-machine-ip>
   ```

2. Set ROS_DOMAIN_ID (both machines must use same ID):
   ```bash
   export ROS_DOMAIN_ID=0
   ```

3. Add to ~/.bashrc for permanence:
   ```bash
   echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
   source ~/.bashrc
   ```

4. Verify DDS can discover nodes:
   ```bash
   ros2 node list  # Should see nodes from other machines
   ```

---

## Gazebo and Simulation Issues

### Issue: URDF fails to load in Gazebo

**Symptom**: Gazebo crashes or robot doesn't appear when loading URDF.

**Cause**: URDF syntax error or incorrect file path.

**Solution**:
1. Validate URDF:
   ```bash
   check_urdf humanoid-robot.urdf
   ```
   Should output: `robot name is OK`

2. Visualize URDF:
   ```bash
   urdf_to_graphiz humanoid-robot.urdf
   ```
   Opens graph of joints and links.

3. Load in RViz to verify:
   ```bash
   rviz2
   # Add RobotModel, select URDF file, set Fixed Frame
   ```

---

### Issue: Gazebo runs but shows blank world

**Symptom**: Gazebo opens but world is empty; no robot visible.

**Cause**: URDF not spawned or wrong model path.

**Solution**:
1. Ensure URDF is in correct path
2. Use launch file or manual spawn:
   ```bash
   gazebo --verbose humanoid-robot.urdf
   ```

3. Check Gazebo output for error messages

---

## RViz Issues

### Issue: RViz opens but displays nothing

**Symptom**: RViz window is blank; no robot or data visible.

**Cause**: No fixed frame set or RobotModel not added.

**Solution**:
1. Set Fixed Frame:
   - Right panel, under "Global Options" → Fixed Frame
   - Select "base_link" or "world"

2. Add RobotModel:
   - Left panel, "Add" button → By Topic → /robot_description → RobotModel

3. Load URDF:
   - RobotModel → Robot Description: `/robot_description`

---

### Issue: "No tf2 transform"

**Symptom**: RViz warns about missing tf2 transforms.

**Cause**: robot_state_publisher not running.

**Solution**:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro humanoid-robot.urdf)"
```

---

## Performance Issues

### Issue: Examples run slowly or lag

**Symptom**: Publisher frequency is lower than expected, or RViz visualization lags.

**Cause**: System load, CPU throttling, or expensive computations.

**Solution**:
1. Check system resources:
   ```bash
   top  # Ctrl+C to exit
   ```

2. Reduce message frequency (increase timer_period):
   ```python
   timer_period = 0.1  # 10 Hz instead of 100 Hz
   ```

3. Close unnecessary applications

4. Check if system is using power-saving mode

---

## Getting Help

If you encounter an issue not listed here:

1. **Search ROS Discourse**: https://discourse.ros.org/
2. **Check error messages**: Read the full output, not just the first line
3. **Verify setup**: Follow [Setup Guide](./setup-guide.md) step-by-step
4. **Test basic example**: Run the Quickstart example to confirm ROS 2 works
5. **Check Python version**: `python3 --version` (must be 3.10+)
6. **Verify ROS 2 version**: `ros2 --version` (must be Humble or later)

---

## Common Misunderstandings

### "My subscriber node doesn't receive messages"

**Possible causes**:
- Subscriber and publisher use different topic names
- Subscriber uses wrong message type
- Publisher hasn't started yet
- ROS 2 not sourced in shell

**Check**:
```bash
ros2 topic list  # See all topics
ros2 topic type /topic_name  # See message type
ros2 topic hz /topic_name  # See publish frequency
```

---

### "My service server doesn't respond"

**Possible causes**:
- Server not running
- Client uses wrong service name
- Different request/response types
- Server crashed

**Check**:
```bash
ros2 service list  # See all services
ros2 service type /service_name  # See request/response types
ros2 service call /service_name --help  # See how to call
```

---

This document is updated as issues are discovered. If you find a new issue, please report it on [GitHub](https://github.com/anthropics/ros2-humanoid-book/issues).
