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
# Any changes to code will start rebuilding from here, skipping the 10-minute rosdep step!
COPY src/jetracer_ros2 src/jetracer_ros2

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
