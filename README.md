# JetRacer ROS 2 Port

This repository is a port of the original JetRacer's ROS 1 functionality to ROS 2.

## Instructions

### 1. Install Docker
```bash
sudo apt install docker
```

### 2. Install GStreamer Plugins
Install the GStreamer good, bad, and ugly plugins if they are not already installed:
```bash
sudo apt install gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
```

### 3. Clone the Repository
Clone this repository to your local machine:
```bash
git clone https://github.com/escher-bach/jetracer_ros2_port.git
```

### 4. Build the Docker Image
Navigate into the cloned repository and build the Docker image using Docker Compose:
```bash
cd jetracer_ros2_port
docker compose build
```
*(Make sure to start your containers with `docker compose up -d` before trying to execute into them.)*

### 5. Access the Container
To open a shell inside the running container, open a new terminal and execute:
```bash
docker exec -it my_jetracer bash
```

---

## Remote Machine Setup

To view data remotely, you will need a remote machine running **Ubuntu 22.04 (Jammy)** with **ROS 2 Humble** installed.

### Demonstration: Camera

1. **On the JetRacer (inside the Docker container):**
   Launch the camera node:
   ```bash
   ros2 launch jetracer_ros2 csi_camera_launch.py
   ```

2. **On the Remote Machine:**
   Run `rqt_image_view` to see the published camera image:
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```
