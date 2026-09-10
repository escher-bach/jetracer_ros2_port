# JetRacer ROS 2 Port

ROS 2 (Humble) port of the JetRacer platform, running inside Docker on a Jetson Nano (JetPack 4.5). Covers motor control, IMU/EKF odometry, LiDAR-based SLAM, autonomous navigation (Nav2), and CSI camera streaming.

For remote visualisation, vision pipelines, and teleoperation, see the companion repository: **[jetracer_remote_machine](https://github.com/escher-bach/jetracer_remote_machine)**.

---

## Development installation

Clone the repository
```bash
git clone https://github.com/escher-bach/jetracer_ros2_port.git
cd jetracer_ros2_port
```
Install prerequisites by running the install script (Docker, Docker Compose)
```bash
chmod -x install.sh
bash install.sh
```
Pull the Docker images (GHCR)
```bash
docker compose pull
```

The repository includes `docker-compose.override.yml`, which Docker Compose
loads automatically for local development. It restores the source/config bind
mounts and interactive shell workflow without putting those repository paths in
the deployable Compose definition.

## Standalone deployment

`docker-compose.yml` is the standalone definition intended for a deployment
system or a clean Jetson. It contains no repository-relative paths: application
code and static configuration are in the images, while maps and model artifacts
use Docker-managed named volumes.

To exercise that definition from a checkout without loading the development
override:

```bash
docker compose -f docker-compose.yml pull
docker compose -f docker-compose.yml up -d
```

The standalone JetRacer service launches `camera_slam_nav_launch.py`
automatically. Set `JETRACER_IMAGE`, `ZENOH_IMAGE`, and `JETRACER_TAG` when a
release is hosted somewhere other than the default GHCR repositories.

---

## Usage

### Step 1 — Start the containers

```bash
docker compose up
```

In a repository checkout this starts `my_jetracer` in its development shell and
`my_jetracer_zenoh` as the Zenoh bridge. A standalone deployment starts the ROS 2
stack automatically instead.

### Step 2 — Launch the robot stack

In a new terminal, exec into the container and start the full demo:

```bash
docker exec -it my_jetracer bash
ros2 launch jetracer_ros2 camera_slam_nav_launch.py
```

### Step 3 — Connect the remote machine

The robot dials the remote machine, not the other way round, so the link is
configured entirely on this side. `ROUTER_IP` in `.env` is the address of the
machine running the Zenoh router; change it if that machine moves:

```ini
ROUTER_IP=192.168.0.142
```

`my_jetracer_zenoh` runs in Zenoh `client` mode and retries that endpoint
indefinitely, so the robot may boot first and pick the router up whenever it
appears. On the host laptop, follow the setup in
[jetracer_remote_machine](https://github.com/escher-bach/jetracer_remote_machine)
and bring up its stack — its bridge listens as a router on `tcp/0.0.0.0:7447`
and keeps no list of robot IPs, so adding a robot means editing only that
robot's `.env`.

---

## Multirobot Integration

A namespace can be set in the `.env` file for multi-robot setups. On the jetracer, topics remain unchanged (e.g., `/cmd_vel`), but Zenoh attaches the namespace so that on the remote machine they appear as `/<namespace>/topic_name`.

For example, setting the namespace to `bot1` will cause the topics to appear on the remote machine as:

```bash
/bot1/cmd_vel
```

---

## Launch File Reference

| Launch file | What it starts |
|---|---|
| `jetracer_launch.py` | `jetracer_node` + EKF + static TFs |
| `slam_launch.py` | jetracer + LiDAR + laser filter + SLAM Toolbox |
| `rtabmap_slam_launch.py` | jetracer + LiDAR + filter + camera + RTAB-Map (instead of SLAM Toolbox — see `RTABMAP.md` in `../jetracer_ros2_port_documentation/`) |
| `camera_slam_nav_launch.py` | SLAM + camera + Nav2 all-in-one *(primary demo)* |
| `localization_launch.py` | Saved-map nav: map_server + AMCL + Nav2. Accepts `map:=` |
| `nav_launch.py` | Nav2 stack only |
| `csi_camera_launch.py` | Camera node reading from shared memory |
