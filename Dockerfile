# ENV ROS1_DISTRO noetic
# ENV ROS2_DISTRO galactic

ARG FROM_IMAGE=ros:galactic-ros1-bridge
ARG OVERLAY_WS=/opt/overlay_ws

# multi-stage for caching
FROM $FROM_IMAGE AS cacher

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
      | xargs cp --parents -t /tmp/opt

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

# install overlay dependencies
ARG OVERLAY_WS
ENV OVERLAY_WS $OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY --from=cacher /tmp/$OVERLAY_WS ./
RUN . /opt/ros/$ROS1_DISTRO/setup.sh && \
    . /opt/ros/$ROS2_DISTRO/setup.sh && \
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
    . /opt/ros/$ROS2_DISTRO/setup.sh && \
    colcon build \
      --symlink-install \
      --mixin $OVERLAY_MIXINS

# source overlay from entrypoint
RUN sed --in-place \
      's|^source .*|source "$OVERLAY_WS/install/setup.bash"|' \
      /ros_entrypoint.sh

# test overlay build
ARG RUN_TESTS
ARG FAIL_ON_TEST_FAILURE=True
RUN if [ -n "$RUN_TESTS" ]; then \
        . install/setup.sh && \
        colcon test && \
        colcon test-result \
          || ([ -z "$FAIL_ON_TEST_FAILURE" ] || exit 1) \
    fi
