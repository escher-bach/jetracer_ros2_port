# JetRacer ROS 2 Port

ROS 2 (Humble) port of the JetRacer platform, running inside Docker on a Jetson Nano (JetPack 4.6). Covers motor control, IMU/EKF odometry, LiDAR-based SLAM, autonomous navigation (Nav2), and CSI camera streaming.

For remote visualisation, vision pipelines, and teleoperation, see the companion repository: **[jetracer_remote_machine](https://github.com/escher-bach/jetracer_remote_machine)**.

---

## Installation

### 1. Prerequisites

```bash
# Docker
sudo apt install docker.io

# GStreamer (for the camera pipeline on the bare host)
sudo apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
```

### 2. Clone and build

```bash
git clone https://github.com/escher-bach/jetracer_ros2_port.git
cd jetracer_ros2_port
docker compose build
```

---

## Usage

### Step 1 — Start the containers

```bash
docker compose up
```

This starts `my_jetracer` (the ROS 2 stack) and `my_jetracer_zenoh` (the Zenoh bridge that exposes all topics over TCP to the remote machine).

### Step 2 — Launch the robot stack

In a new terminal, exec into the container and start the full demo:

```bash
docker exec -it my_jetracer bash
ros2 launch jetracer_ros2 camera_slam_nav_launch.py
```

### Step 3 — Connect the remote machine

On the host laptop, follow the setup in [jetracer_remote_machine](https://github.com/escher-bach/jetracer_remote_machine) then connect:

```bash
zenoh-bridge-ros2dds -e tcp/<jetson-ip>:7447
```

---

## Launch File Reference

| Launch file | What it starts |
|---|---|
| `jetracer_launch.py` | `jetracer_node` + EKF + static TFs |
| `slam_launch.py` | jetracer + LiDAR + laser filter + SLAM Toolbox |
| `camera_slam_nav_launch.py` | SLAM + camera + Nav2 all-in-one *(primary demo)* |
| `localization_launch.py` | Saved-map nav: map_server + AMCL + Nav2. Accepts `map:=` |
| `nav_launch.py` | Nav2 stack only |
| `csi_camera_launch.py` | Camera node reading from shared memory |
