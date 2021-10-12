FROM gabry993/pointing-user-interface:galactic
MAINTAINER Gabriele Abbate "gabriele.abbate@idsia.ch"
RUN apt-get update -q && \
	export DEBIAN_FRONTEND=noninteractive && \
    apt-get install -y --no-install-recommends \
        vim tar xz-utils \
        libx11-6 libxcb1 libxau6 libgl1-mesa-dev \
        xvfb dbus-x11 x11-utils libxkbcommon-x11-0 \
        xsltproc \
        ros-galactic-nav-msgs \
        ros-galactic-pendulum-msgs \
        ros-galactic-map-msgs \
        ros-galactic-rosgraph-msgs \
        ros-galactic-sensor-msgs \
        ros-galactic-shape-msgs \
        ros-galactic-std-msgs \
        ros-galactic-stereo-msgs \
        ros-galactic-tf2-geometry-msgs \
        ros-galactic-tf2-msgs \
        ros-galactic-trajectory-msgs \
        ros-galactic-unique-identifier-msgs \
        ros-galactic-visualization-msgs \
        ros-galactic-image-transport \
        ros-galactic-example-interfaces \
        ros-galactic-example-interfaces \
        libavcodec-dev libavformat-dev libswscale-dev \
        cmake libblkid-dev e2fslibs-dev libboost-all-dev libaudit-dev \
        && \
    apt-get autoclean -y && apt-get autoremove -y && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /shared /opt
RUN python3 -m pip install -U xmlschema

COPY ./download/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04.tar.xz /opt/
RUN tar -xf /opt/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04.tar.xz -C /opt && \
    rm /opt/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04.tar.xz

ENV COPPELIASIM_ROOT_DIR=/opt/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04/
ENV LD_LIBRARY_PATH=$COPPELIASIM_ROOT_DIR:$LD_LIBRARY_PATH
ENV PATH=$COPPELIASIM_ROOT_DIR:$PATH

#COPY ./sim_ros2_interface /ros_ws/src/sim_ros2_interface
RUN cd /ros_ws/src && \
git clone --recursive -b galactic https://github.com/jeguzzi/simExtROS2.git sim_ros2_interface

RUN cd /ros_ws && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install --packages-select sim_ros2_interface && \
  rm -r build log

# RUN echo '#!/bin/bash\nsource /ros_entrypoint.sh\ncd $COPPELIASIM_ROOT_DIR\n/usr/bin/xvfb-run --server-args "-ac -screen 0, 1024x1024x24" coppeliaSim "$@"' > /entrypoint && chmod a+x /entrypoint
# RUN echo '#!/bin/bash\ncd /opt/CoppeliaSim_Edu_V4_2_0_Ubuntu20_04/; coppeliaSim "$@"' > /entrypoint && chmod a+x /entrypoint
EXPOSE 19997
#ENTRYPOINT ["/entrypoint"]
