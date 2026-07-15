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

# Clone sllidar_ros2
RUN cd src && git clone https://github.com/Slamtec/sllidar_ros2.git

# Copy ONLY package.xml first to cache the slow rosdep install step
COPY src/jetracer_ros2/package.xml src/jetracer_ros2/package.xml

# Initialize rosdep, update, and install dependencies
# The --fix-missing flag helps if Ubuntu ports mirrors flake out (403 errors)
RUN apt-get update --fix-missing && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    rm -rf /var/lib/apt/lists/*

# Now copy the rest of the source code
COPY src/jetracer_ros2 src/jetracer_ros2

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Global sourcing for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}" >> /root/.bashrc

# --- GPU, phase 1: host bind mounts (see docs GPU_IN_CONTAINER.md) ---
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

# Copy entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
