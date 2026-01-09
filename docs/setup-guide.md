---
sidebar_position: 201
---

# Setup Guide: Ubuntu 22.04 LTS + ROS 2 Humble

This guide walks you through installing ROS 2 Humble and all dependencies needed to run examples in this book. **Estimated time: 20-30 minutes on a clean Ubuntu 22.04 system.**

## Prerequisites

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Hardware**: x86_64 desktop/laptop or VM (4GB RAM minimum, 20GB disk)
- **Network**: Internet connection for package downloads
- **Terminal**: Comfortable using bash/shell commands

> **Note**: This guide is for Ubuntu 22.04 LTS. For other Ubuntu versions or OS, refer to the [official ROS 2 installation docs](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

---

## Step 1: Update System (5 minutes)

Open a terminal and run:

```bash
sudo apt update
sudo apt upgrade -y
```

This ensures your package manager has the latest repository information.

---

## Step 2: Install ROS 2 Humble (10 minutes)

### 2a. Set up the ROS 2 repository

```bash
# Install curl (if not already installed)
sudo apt install curl gnupg lsb-release ubuntu-keyring -y

# Add the ROS 2 GPG key
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists
sudo apt update
```

### 2b. Install ROS 2 Humble

```bash
# Install ROS 2 Humble base distribution
sudo apt install ros-humble-desktop -y
```

This installs:
- ROS 2 core libraries
- rclpy (Python client library)
- Common message types (sensor_msgs, geometry_msgs, etc.)
- RViz (robot visualization)
- Additional tools

> **Installation size**: ~4-5 GB. This will take 5-10 minutes depending on internet speed.

### 2c. Verify Installation

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

Expected output: `ROS 2 version <date>`

---

## Step 3: Install Additional Dependencies (10 minutes)

### 3a. Gazebo Simulator (for Chapter 3 examples)

```bash
sudo apt install ros-humble-gazebo* -y
```

### 3b. ROS 2 Development Tools

```bash
sudo apt install ros-humble-ros2-control ros-humble-controller-manager python3-colcon-common-extensions -y
```

### 3c. Python Development Tools

```bash
sudo apt install python3-pip python3-venv -y
pip3 install --upgrade setuptools
```

### 3d. Verify Gazebo Installation

```bash
gazebo --version
```

Expected output: `Gazebo simulation version <number>`

---

## Step 4: Configure Shell (2 minutes)

To avoid manually sourcing ROS 2 setup on every terminal, add it to your shell profile:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Or if you use zsh:

```bash
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

Verify:

```bash
echo $ROS_DISTRO
```

Expected output: `humble`

---

## Step 5: Verify Complete Installation (5 minutes)

Run a quick test to ensure everything is working:

### 5a. Test ROS 2 Nodes

Open two terminal windows:

**Terminal 1 - Publisher**:
```bash
ros2 run demo_nodes_cpp talker
```

Expected output:
```
[INFO] [talker]: Publishing: 'Hello World: <number>'
```

**Terminal 2 - Subscriber**:
```bash
ros2 run demo_nodes cpp listener
```

Expected output:
```
[INFO] [listener]: I heard: [Hello World: <number>]
```

If both terminals show messages, ROS 2 is working correctly! ✅

### 5b. Test RViz

```bash
rviz2
```

RViz GUI should open. Close it with Ctrl+C.

### 5c. Test Gazebo

```bash
gazebo
```

Gazebo GUI should open. Close it with Ctrl+C.

---

## Step 6: Clone Book Examples (2 minutes)

All code examples from this book are in the GitHub repository:

```bash
# Clone the book repository
git clone https://github.com/anthropics/ros2-humanoid-book.git
cd ros2-humanoid-book

# Verify examples directory exists
ls docs/examples/
```

You should see:
- `ch1-dds-pubsub/`
- `ch2-communication-patterns/`
- `ch3-urdf-simulation/`

---

## Step 7: Run Your First Example (5 minutes)

Follow the book's [Quickstart Guide](./quickstart.md) to run your first ROS 2 publisher node.

Or manually run:

```bash
# From the book directory
cd docs/examples/ch1-dds-pubsub/

# Run the minimal publisher
python3 minimal-publisher.py
```

Expected output:
```
[INFO] [MinimalPublisher]: Publishing: 0
[INFO] [MinimalPublisher]: Publishing: 1
...
```

---

## Troubleshooting

### Issue: "bash: ros2: command not found"

**Solution**: Ensure ROS 2 is sourced:
```bash
source /opt/ros/humble/setup.bash
```

Or check that it was added to your `~/.bashrc`:
```bash
grep "ros/humble" ~/.bashrc
```

### Issue: "E: Unable to locate package ros-humble-desktop"

**Solution**: Verify the ROS 2 repository was added:
```bash
cat /etc/apt/sources.list.d/ros2.list
sudo apt update
```

### Issue: Gazebo fails to start

**Solution**: Ensure Gazebo is installed and your GPU drivers are up to date:
```bash
gazebo --verbose
# Look for error messages
sudo apt install ros-humble-gazebo-plugins -y
```

### Issue: "Permission denied" when running scripts

**Solution**: Make the script executable:
```bash
chmod +x script-name.py
python3 script-name.py
```

### Issue: Example runs but produces no output

**Solution**: Verify ROS 2 is properly sourced and ROS_DISTRO is set:
```bash
echo $ROS_DISTRO
# Should output: humble
```

If empty, source ROS 2 again:
```bash
source /opt/ros/humble/setup.bash
```

---

## Uninstall (Optional)

To remove ROS 2 Humble:

```bash
sudo apt remove ros-humble-* -y
sudo apt remove ros-*-gazebo* -y
sudo apt autoremove -y
```

---

## Next Steps

1. **[Read the Introduction](./index.md)** to understand the book structure
2. **[Part 1: Foundations](./part1-foundations/01-ros2-overview)** to learn ROS 2 and DDS concepts
3. **[Quickstart Guide](./quickstart.md)** to run your first example

---

## Additional Resources

- **Official ROS 2 Humble Docs**: https://docs.ros.org/en/humble/
- **ROS 2 Installation (detailed)**: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- **ROS 2 Gazebo Integration**: https://gazebosim.org/docs/garden/ros2/
- **RViz2 User Guide**: https://docs.ros.org/en/humble/Concepts/Intermediate/About-RViz2.html

---

## Verification Checklist

- [ ] Ubuntu 22.04 LTS is running (`lsb_release -a`)
- [ ] ROS 2 Humble is installed (`ros2 --version`)
- [ ] Gazebo is installed (`gazebo --version`)
- [ ] Publisher/subscriber test works (talker/listener)
- [ ] RViz opens successfully (`rviz2`)
- [ ] Gazebo starts successfully (`gazebo`)
- [ ] Book examples directory is cloned
- [ ] You can run `python3 minimal-publisher.py`

If all boxes are checked, you're ready to start learning! ✅

---

**Questions?** See the [Known Issues](./known-issues.md) page or the [ROS 2 Discourse](https://discourse.ros.org/).
