FROM ros:humble-ros-base

# Prevent interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive

# Force IPv4 and switch to a faster ROS2 mirror (University of Maryland)
RUN echo 'Acquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99force-ipv4 && \
    sed -i 's|http://packages.ros.org/ros2/ubuntu|http://mirrors.umd.edu/ros2/ubuntu|g' /etc/apt/sources.list.d/ros2.list

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    git \
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

# Copy jetracer_ros2
COPY src/jetracer_ros2 src/jetracer_ros2

# Initialize rosdep, update, and install dependencies
# We use --from-path src to cover sllidar_ros2, jetracer_ros2, and any others.
RUN apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Global sourcing for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}" >> /root/.bashrc

# Copy entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
