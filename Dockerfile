# Use the official ROS 2 Foxy image as the base image
FROM dustynv/ros:foxy-ros-base-l4t-r32.7.1

# Set ROS 2 environment variables
ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=0

# Install necessary dependencies and ROS 2 packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    libopencv-dev \
    python3-opencv \
    ros-foxy-v4l2-camera \
    && rm -rf /var/lib/apt/lists/*

# Reinstall colcon and install additional Python packages
RUN pip3 uninstall -y colcon-common-extensions && \
    pip3 install colcon-common-extensions

# Copy the workspace files into the container
COPY . /home/anthonykpinson/robot_ws

# Set environment variables
ENV USERNAME=ros
ENV USER_UID=1000
ENV USER_GID=$USER_UID

# Create a non-root user and set up sudo
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Set the default user
USER $USERNAME

# Set the working directory
WORKDIR /home/anthonykpinson/robot_ws