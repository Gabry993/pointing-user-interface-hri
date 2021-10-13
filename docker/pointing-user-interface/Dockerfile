FROM ros:galactic
MAINTAINER Gabriele Abbate "gabriele.abbate@idsia.ch"
RUN apt-get update && apt-get install -y \
    locales \
    python3-pip \
    ros-galactic-launch-testing-ament-cmake \
    ros-galactic-robot-state-publisher \
    ros-galactic-joint-state-publisher-gui \
    ros-galactic-joy \
    ros-galactic-rqt* \
    ros-galactic-xacro \
    python3-pykdl \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US.UTF-8
SHELL ["/bin/bash", "-c"]
RUN python3 -m pip install -U numpy pyyaml chime scipy transforms3d Shapely
RUN mkdir -p /ros_ws/src/
RUN git clone -b dashing-devel https://github.com/jeguzzi/teleop_tools.git
COPY ./code/ /ros_ws/src/
RUN cp -r /teleop_tools/. /ros_ws/src/
RUN cd /ros_ws/src && git clone -b ros2 https://github.com/Gabry993/volaly_kinematics.git && git clone -b ros2 https://github.com/Gabry993/volaly_msgs.git
RUN cd /ros_ws && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install && \
  rm -r build log
RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/ros_ws\/install\/setup.bash"' \
  /ros_entrypoint.sh
