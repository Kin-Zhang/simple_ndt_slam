FROM osrf/ros:noetic-desktop-full
LABEL maintainer="Kin Zhang <kin_eng@163.com>"

# Just in case we need it
ENV DEBIAN_FRONTEND noninteractive

# install zsh
RUN apt update && apt install -y wget git zsh tmux vim g++
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
  -t robbyrussell \
  -p git \
  -p ssh-agent \
  -p https://github.com/agkozak/zsh-z \
  -p https://github.com/zsh-users/zsh-autosuggestions \
  -p https://github.com/zsh-users/zsh-completions \
  -p https://github.com/zsh-users/zsh-syntax-highlighting

RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
RUN echo "source /opt/ros/noetic/setup.bashrc" >> ~/.bashrc

# needs to be done before we can apply the patches
RUN git config --global user.email "xxx@163.com"
RUN git config --global user.name "kin-docker"

RUN mkdir -p /workspace/mapping_ws
RUN git clone --recurse-submodules https://github.com/Kin-Zhang/simple_ndt_slam.git /workspace/mapping_ws/src

RUN chmod +x /workspace/mapping_ws/src/assets/scripts/setup_lib.sh
RUN /workspace/mapping_ws/src/assets/scripts/setup_lib.sh
RUN apt-get install -y ros-noetic-catkin python3-catkin-tools
WORKDIR /workspace/mapping_ws