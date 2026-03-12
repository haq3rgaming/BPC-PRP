FROM osrf/ros:humble-desktop

ARG CODE_SERVER_VERSION=4.102.0

# Create user with id 1000
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -rm -d /home/$USERNAME -s /bin/bash -g root -G sudo -u $USER_UID $USERNAME

# Add passwordless sudo for the user
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set ROS_DOMAIN_ID environment variable for the user
ARG ROS_DOMAIN_ID=7

# Install necessary packages (nano, ping, ip utils)
RUN apt-get update && apt-get install -y \
    nano \
    curl \
    ca-certificates \
    ros-humble-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# Preinstall VS Code Server (code-server)
RUN curl -fsSL "https://github.com/coder/code-server/releases/download/v${CODE_SERVER_VERSION}/code-server-${CODE_SERVER_VERSION}-linux-amd64.tar.gz" -o /tmp/code-server.tar.gz \
    && tar -xzf /tmp/code-server.tar.gz -C /tmp \
    && mv "/tmp/code-server-${CODE_SERVER_VERSION}-linux-amd64" /home/$USERNAME/.vscode-server \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/.vscode-server \
    && ln -s /home/$USERNAME/.vscode-server/bin/code-server /usr/bin/code-server \
    && rm /tmp/code-server.tar.gz

# Add bashrc for the user
RUN echo "export ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> /home/$USERNAME/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Use default entrypoint and command
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]