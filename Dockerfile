FROM ros:humble-ros-base

# Prevent interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive

# Use UMD (US) mirror for ROS2
RUN sed -i --follow-symlinks 's|^URIs: .*|URIs: http://mirror.umd.edu/packages.ros.org/ros2/ubuntu/|g' /etc/apt/sources.list.d/ros2.sources

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    git \
    nano \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Install CycloneDDS
RUN apt-get update && apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Setup Workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Clone sllidar_ros2, pinned. Upstream publishes no tags, so this is the
# main-branch commit as of 2026-07-27; without a pin a cache-cold build (i.e.
# every first CI build) silently picks up whatever main is that day and freezes
# it into the buildcache. Bump by replacing the SHA.
RUN cd src && git clone https://github.com/Slamtec/sllidar_ros2.git && \
    git -C sllidar_ros2 checkout -q 34300099fadfc772965962dec837bf436706188f

# Copy ONLY package.xml first to cache the slow rosdep install step
COPY src/jetracer_ros2/package.xml src/jetracer_ros2/package.xml
COPY src/jetracer_segmentation/package.xml src/jetracer_segmentation/package.xml

# Initialize rosdep, update, and install dependencies
# The --fix-missing flag helps if Ubuntu ports mirrors flake out (403 errors)
RUN apt-get update --fix-missing && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    rm -rf /var/lib/apt/lists/*

# --- GPU, phase 1: host bind mounts (see docs GPU_IN_CONTAINER.md) ---
# Kept ABOVE the source COPY on purpose: nothing here depends on the source,
# so a routine code change leaves these layers cached and out of the registry
# pull delta. Kept BELOW rosdep equally on purpose: the focal apt source below
# must not exist while rosdep resolves.
# Linker search paths for the runtime mounts (harmless dangling when absent),
# mirroring a JetPack host's own nvidia-tegra ld.so.conf, plus the
# conventional /usr/local/cuda symlink.
RUN printf '%s\n' /usr/lib/aarch64-linux-gnu/tegra /usr/local/cuda-10.2/lib64 \
        > /etc/ld.so.conf.d/000-cuda-tegra.conf && \
    ln -s /usr/local/cuda-10.2 /usr/local/cuda

# gcc-8 as the nvcc host compiler: nvcc 10.2 requires gcc <= 8, which jammy
# does not ship. Pulled from focal ports pinned at priority 100 so jammy
# packages always win; only gcc-8 and its private deps come from focal.
# (Proven: jetson_cuda_experiment H3/H4.)
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports focal main universe" \
        > /etc/apt/sources.list.d/focal.list && \
    printf 'Package: *\nPin: release n=focal\nPin-Priority: 100\n' \
        > /etc/apt/preferences.d/focal && \
    apt-get update && apt-get install -y --no-install-recommends gcc-8 g++-8 && \
    rm -rf /var/lib/apt/lists/*

ENV PATH=/usr/local/cuda-10.2/bin:$PATH \
    CUDA_HOME=/usr/local/cuda-10.2 \
    CUDAHOSTCXX=/usr/bin/g++-8 \
    CUDAARCHS=53

# Global sourcing for interactive shells (source-independent, so also above)
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}" >> /root/.bashrc

# Now copy the rest of the source code. The segmentation package is shipped in
# the image so a deployed robot never needs a repository checkout, but remains
# intentionally excluded from the production build until the matching JetPack
# CUDA/TensorRT build environment is available in CI.
COPY src/jetracer_ros2 src/jetracer_ros2
COPY src/jetracer_segmentation src/jetracer_segmentation

# A development bind mount shadows this directory (and therefore this marker),
# preserving the existing on-Jetson experimental build workflow.
RUN touch src/jetracer_segmentation/COLCON_IGNORE

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Copy entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Default site configuration. Deployments may override DOCK_TABLE with a
# platform-managed config, while the development Compose override mounts the
# checkout's live copy at /data/docks.
COPY docks/docks.yaml /opt/jetracer/config/docks.yaml
ENV DOCK_TABLE=/opt/jetracer/config/docks.yaml

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "jetracer_ros2", "camera_slam_nav_launch.py"]
