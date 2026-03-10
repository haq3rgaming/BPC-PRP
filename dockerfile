FROM osrf/ros:humble-desktop

# Create user with id 1000
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -rm -d /home/$USERNAME -s /bin/bash -g root -G sudo -u $USER_UID $USERNAME

# Add passwordless sudo for the user
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install necessary packages (nano, ping, ip utils)
RUN apt-get update && apt-get install -y \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Add bashrc for the user
RUN echo "export ROS_DOMAIN_ID=5" >> /home/$USERNAME/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Use default entrypoint and command
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]