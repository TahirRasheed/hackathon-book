# Quick Start: Setting Up ROS 2 Environment for Module 1 Chapters

**Date**: 2025-12-17  
**Status**: Phase 1 Output  
**Target**: Students preparing to follow along with code examples in Module 1

## Prerequisites

- **Computer Setup**: Linux (Ubuntu 22.04 LTS recommended), macOS, or Windows with WSL2
- **Docker** (optional but recommended): If using Docker to avoid local ROS 2 installation complexity
- **Basic Programming**: Python 3.11 knowledge; familiarity with command-line terminals

## Option 1: Local ROS 2 Humble Installation (Ubuntu 22.04)

### Step 1: Add ROS 2 Repository and Install

```bash
# Set locale
locale  # Check en_US.UTF-8 is available
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 GPG key
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $VERSION_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install
sudo apt update
sudo apt install -y ros-humble-desktop
```

### Step 2: Source Setup Script

```bash
source /opt/ros/humble/setup.bash
```

### Step 3: Install rclpy and Development Tools

```bash
sudo apt install -y python3-pip
pip install --upgrade pip
pip install rclpy
sudo apt install -y ros-humble-std-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs
```

---

## Option 2: Docker Setup (Recommended for Portability)

### Quick Start with ROS 2 Docker Image

```bash
# Pull official ROS 2 Humble image
docker pull osrf/ros:humble-desktop

# Run interactive container
docker run -it --rm osrf/ros:humble-desktop /bin/bash

# Inside container, test ROS 2
source /opt/ros/humble/setup.bash
ros2 --version
```

### Docker Compose (Optional, for Persistent Development)

Create `docker-compose.yml`:

```yaml
version: '3.8'
services:
  ros2:
    image: osrf/ros:humble-desktop
    container_name: ros2-module1
    stdin_open: true
    tty: true
    volumes:
      - .:/root/ros2_ws/src
    working_dir: /root/ros2_ws
    command: /bin/bash
```

Run:

```bash
docker-compose up -d
docker-compose exec ros2 bash
```

---

## Step 4: Verify Installation

### Check ROS 2 Version

```bash
ros2 --version
# Expected output: ROS 2 humble (version 0.x.x)
```

### Run Talker/Listener Example

```bash
# Terminal 1: Talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Listener
ros2 run demo_nodes_py listener
```

### Test rclpy Publisher/Subscriber

Create `test_publisher.py`:

```python
import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('test_pub')
    pub = node.create_publisher(String, 'test_topic', 10)
    
    msg = String(data='Hello ROS 2!')
    pub.publish(msg)
    print('Published:', msg.data)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run:

```bash
python3 test_publisher.py
```

---

## Step 5: Install Additional Tools (Optional)

### For Chapter 3 (URDF Parsing)

```bash
pip install lxml  # For XML parsing
sudo apt install -y ros-humble-urdf  # URDF tools
```

### For Visualization (Gazebo Simulator Reference)

```bash
sudo apt install -y gazebo
sudo apt install -y ros-humble-gazebo-ros-pkgs
```

---

## Troubleshooting

### Issue: "ros2: command not found"
**Solution**: Ensure setup.bash is sourced:
```bash
source /opt/ros/humble/setup.bash
```
Add to `.bashrc` for persistence:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Issue: "ModuleNotFoundError: No module named 'rclpy'"
**Solution**: Install rclpy:
```bash
pip install rclpy
```

### Issue: Docker permission denied
**Solution**: Add user to docker group:
```bash
sudo usermod -aG docker $USER
newgrp docker
```

---

## Next Steps

1. **Read Chapter 1**: Introduction to ROS 2 and the Robotic Nervous System
2. **Run code examples** from Chapter 2 to test your installation
3. **Chapter 3**: Write your own Python ROS 2 nodes using this setup

---

## Resources

- [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [ROS 2 Beginner CLI Tools](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)
- [ROS 2 rclpy Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

---

## Phase 1 Status

✅ **COMPLETE** — Quickstart guide ready for students. All setup paths documented.
