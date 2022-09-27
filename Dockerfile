# check more detail on: https://hub.docker.com/r/nvidia/cuda
FROM nvidia/cuda:10.2-devel-ubuntu18.04
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

# ==========> INSTALL ROS melodic <=============
RUN apt update && apt install -y curl lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update && apt install -y ros-melodic-desktop-full
RUN apt-get install -y python-catkin-pkg \
  python-catkin-tools \
  python-empy \
  python-nose \
  python-pip \
  libgtest-dev \
  ros-melodic-catkin \
  python-pip \
  python3-pip \
  ros-melodic-grid-map

RUN echo "source /opt/ros/melodic/setup.zsh" >> ~/.zshrc
RUN echo "source /opt/ros/melodic/setup.bashrc" >> ~/.bashrc

# needs to be done before we can apply the patches
RUN git config --global user.email "xxx@163.com"
RUN git config --global user.name "kin-docker"

RUN mkdir -p /workspace/mapping_ws
RUN git clone --recurse-submodules https://github.com/Kin-Zhang/simple_ndt_slam.git /workspace/mapping_ws/src

RUN chmod +x /workspace/mapping_ws/src/assets/scripts/setup_lib.sh
RUN /workspace/mapping_ws/src/assets/scripts/setup_lib.sh
WORKDIR /workspace/mapping_ws