ARG BASE_IMG=ros:melodic
FROM ${BASE_IMG}
ARG LOCAL_USER=turtle

## set apt repository in Japan
RUN sed -i'' 's/archive.ubuntu.com/jp.archive.ubuntu.com/' /etc/apt/sources.list
## locale
RUN apt-get update && \
  apt-get install -y \
  language-pack-ja
RUN update-locale LANG=ja_JP.UTF-8
ENV LANG="ja_JP.UTF-8" \ 
  LANGUAGE="ja_JP:ja" \ 
  LC_ALL="ja_JP.UTF-8" \ 
  TZ="JST-9"
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y tzdata
## install ROS
RUN apt-get update && apt-get install -y lsb-release curl 
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && \
    apt-get install -y \
    ros-melodic-desktop-full \
    ros-melodic-laser-proc \
    ros-melodic-rgbd-launch \
    ros-melodic-depthimage-to-laserscan \
    ros-melodic-amcl \
    ros-melodic-map-server \
    ros-melodic-move-base \
    ros-melodic-urdf \
    ros-melodic-xacro \
    ros-melodic-compressed-image-transport \
    ros-melodic-rqt-image-view \
    ros-melodic-gmapping \
    ros-melodic-navigation

RUN echo source /opt/ros/melodic/setup.bash > ~/.bashrc

# add sudo user
RUN groupadd -g 1000 developer && \
    useradd  -g      developer -G sudo -m -s /bin/bash ${LOCAL_USER} && \
        echo "${LOCAL_USER}:${LOCAL_USER}" | chpasswd

RUN echo "Defaults visiblepw"             >> /etc/sudoers
RUN echo "${LOCAL_USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN apt-get update && apt-get install -y git build-essential bash
USER ${LOCAL_USER}
WORKDIR /home/${LOCAL_USER}

## download and build turtlebot3 resources
RUN mkdir -p ~/catkin_ws/src 
WORKDIR /home/${LOCAL_USER}/catkin_ws/src
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
RUN git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
WORKDIR /home/${LOCAL_USER}/catkin_ws
RUN sudo apt-get install -y checkinstall python-rosinstall python-rosdep
RUN sudo rosdep init && rosdep update
## add init setting command to .bashrc
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && \
   catkin_make"
# install packages related with gazebo and simulation
RUN sudo apt-get update && \
    sudo apt-get install -y \
    libgl1-mesa-dev \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-teleop-twist-keyboard 
ENV TERM=xterm-256color 
# for managing with  Gezebo's error
RUN mkdir -p /home/${LOCAL_USER}/.ignition/fuel/
COPY fuel/config.yaml /home/${LOCAL_USER}/.ignition/fuel/config.yaml

ENTRYPOINT ["/bin/bash"]

