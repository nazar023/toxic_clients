# Set the ROS distribution version
ARG ROS_DISTRO=humble
FROM osrf/ros2:nightly-rmw-nonfree

# Install necessary packages for ROS2, colcon, and development
RUN apt-get update && apt-get install -y \
      tree \
      git \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-ament-cmake \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-demo-nodes-py \
      build-essential \
      cmake && \
    rm -rf /var/lib/apt/lists/*

# Clone specific branch of your repository
RUN mkdir -p ~/ros2 && \
    cd ~/ros2 && \
    git clone https://github.com/nazar023/toxic_clients.git

# Create a script to build the packages
RUN echo '#!/bin/bash\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
cd ~/ros2/toxic_clients\n\
rm -rf build/ install/ log/\n\
colcon build --packages-select toxic_clients toxic_relationship' > /build.sh && \
    chmod +x /build.sh

# Run the build script
RUN /build.sh

# Optionally, set the entrypoint or command if you want to run your nodes after building
# CMD ["ros2", "run", "toxic_clients", "your_node_name"]
