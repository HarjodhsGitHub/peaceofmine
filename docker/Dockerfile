# syntax=docker/dockerfile:1.7
ARG BUILD_TAG=ros:jazzy-ros-base
FROM ${BUILD_TAG}
ARG ROSDISTRO=jazzy
ARG WORKSPACE=/svea_ws
ARG DEBIAN_FRONTEND=noninteractive
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential cmake git curl python3-pip python3-venv python3-rosdep \
    python3-colcon-common-extensions python3-pytest \
    ros-${ROSDISTRO}-mavros ros-${ROSDISTRO}-mavros-extras \
    && rm -rf /var/lib/apt/lists/*
# Retained launch and vehicle-message dependencies. No mocap/Zenoh/Foxglove runtime.
RUN mkdir -p /tmp/dependencies/src && cd /tmp/dependencies/src \
    && git clone --depth 1 -b v1.16.1 https://github.com/PX4/px4_msgs.git \
    && git clone --depth 1 -b v1.6.0 https://github.com/dfki-ric/better_launch.git \
    && source /opt/ros/${ROSDISTRO}/setup.bash \
    && rosdep update --rosdistro ${ROSDISTRO} \
    && apt-get update \
    && rosdep install --rosdistro ${ROSDISTRO} --from-paths /tmp/dependencies/src --ignore-src -y \
    && cd /tmp/dependencies && colcon build --merge-install --install-base /opt/svea/${ROSDISTRO} \
    && ros2 run mavros install_geographiclib_datasets.sh \
    && rm -rf /tmp/dependencies /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/pom-python
ENV PATH="/opt/pom-python/bin:${PATH}"
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt
RUN printf 'APT::Install-Recommends "false";\nAPT::Install-Suggests "false";\n' > /etc/apt/apt.conf.d/99peaceofmine
WORKDIR ${WORKSPACE}
COPY src/svea_core/package.xml /tmp/rosdeps/svea_core/package.xml
COPY src/svea_localization/package.xml /tmp/rosdeps/svea_localization/package.xml
COPY src/peaceofmine_interfaces/package.xml /tmp/rosdeps/peaceofmine_interfaces/package.xml
COPY src/peaceofmine_operator/package.xml /tmp/rosdeps/peaceofmine_operator/package.xml
RUN source /opt/ros/${ROSDISTRO}/setup.bash && source /opt/svea/${ROSDISTRO}/setup.bash \
    && apt-get update \
    && rosdep install --rosdistro ${ROSDISTRO} --from-paths /tmp/rosdeps --ignore-src -y \
    && rm -rf /tmp/rosdeps /var/lib/apt/lists/*
COPY src ./src
RUN source /opt/ros/${ROSDISTRO}/setup.bash && source /opt/svea/${ROSDISTRO}/setup.bash \
    && colcon build \
    && rm -rf build log \
    && rm -rf /var/lib/apt/lists/*
COPY firmware ./firmware
COPY tools ./tools
COPY entrypoint ./entrypoint
RUN chmod +x ./entrypoint \
    && echo "source /opt/ros/${ROSDISTRO}/setup.bash" >> /etc/bash.bashrc \
    && echo "source /opt/svea/${ROSDISTRO}/setup.bash" >> /etc/bash.bashrc \
    && echo "source ${WORKSPACE}/install/setup.bash" >> /etc/bash.bashrc \
    && python3 -c "from aiohttp import web; import pynmea2; import pyubx2"
EXPOSE 8080
ENTRYPOINT ["./entrypoint"]
CMD ["bash"]
