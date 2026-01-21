FROM osrf/ros:humble-desktop-full

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-turtlesim \
    ros-humble-gazebo-ros-pkgs \
    make \
    flake8 \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
COPY . /ros2_ws/src/hello_physical_ai

# Setup entrypoint
COPY <<EOF /entrypoint.sh
#!/bin/bash
set -e
source "/opt/ros/humble/setup.bash"
exec "\$@"
EOF
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
