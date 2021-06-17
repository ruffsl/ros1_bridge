ARG FROM_IMAGE=osrf/ros2:nightly
ARG UNDERLAY_WS=/opt/underlay_ws
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

# clone underlay source
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS/src
COPY ./underlay.repos ../
RUN vcs import ./ < ../underlay.repos

# copy overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS/src
COPY ./ ./ros1_bridge

# copy manifests for caching
WORKDIR /opt
RUN find . -name "src" -type d \
      -mindepth 1 -maxdepth 2 -printf '%P\n' \
      | xargs -I % mkdir -p /tmp/opt/% && \
    find . -name "package.xml" \
      | xargs cp --parents -t /tmp/opt && \
    find . -name "COLCON_IGNORE" \
      | xargs cp --parents -t /tmp/opt || true

# multi-stage for building
FROM $FROM_IMAGE AS builder

# config dependencies install
ARG DEBIAN_FRONTEND=noninteractive
RUN echo '\
APT::Install-Recommends "0";\n\
APT::Install-Suggests "0";\n\
' > /etc/apt/apt.conf.d/01norecommend

# install CI dependencies
RUN apt-get update && \
    apt-get install -y \
      ccache \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO rolling

# install common dependencies
RUN apt-get update && apt-get install -y \
      ros-$ROS1_DISTRO-actionlib-tutorials \
      ros-$ROS1_DISTRO-control-msgs \
      ros-$ROS1_DISTRO-fetch-auto-dock-msgs \
      ros-$ROS1_DISTRO-fetch-driver-msgs \
      ros-$ROS1_DISTRO-ros-comm \
      ros-$ROS1_DISTRO-roscpp-tutorials \
      ros-$ROS1_DISTRO-rospy-tutorials \
    && rm -rf /var/lib/apt/lists/*
# RUN apt-get update && apt-get install -y \
#       ros-$ROS2_DISTRO-action-tutorials-cpp  \
#       ros-$ROS2_DISTRO-control-msgs \
#     && rm -rf /var/lib/apt/lists/*

# install underlay dependencies
ARG UNDERLAY_WS
ENV UNDERLAY_WS $UNDERLAY_WS
WORKDIR $UNDERLAY_WS
COPY --from=cacher /tmp/$UNDERLAY_WS ./
RUN . /opt/ros/$ROS2_DISTRO/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build underlay source
COPY --from=cacher $UNDERLAY_WS ./
ARG UNDERLAY_MIXINS="release ccache"
RUN . /opt/ros/$ROS2_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $UNDERLAY_MIXINS

# install overlay dependencies
ARG OVERLAY_WS
ENV OVERLAY_WS $OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS ./
RUN . /opt/ros/$ROS1_DISTRO/setup.sh && \
    . $UNDERLAY_WS/install/setup.sh && \
    apt-get update && rosdep install -q -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# multi-stage for testing
FROM builder AS tester

# build overlay source
COPY --from=cacher $OVERLAY_WS ./
ARG OVERLAY_MIXINS="release ccache"
RUN . /opt/ros/$ROS1_DISTRO/setup.sh && \
    . $UNDERLAY_WS/install/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS \
      --cmake-force-configure

# source overlay from entrypoint
COPY ./ros_entrypoint.sh /

# test overlay build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE=True
RUN if [ -n "$RUN_TESTS" ]; then \
        . install/setup.sh && \
        colcon test && \
        colcon test-result \
          || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1) \
    fi
