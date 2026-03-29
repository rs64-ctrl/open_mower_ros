# Lean image without nginx and mosquitto for OSv2 (OS-level compose provides services)
FROM docker.io/ros:jazzy-ros-base-noble AS base
ENV DEBIAN_FRONTEND=noninteractive

# Ensure universe repository is enabled (needed for libcrypto++-dev etc.)
RUN apt-get update && apt-get install --yes --no-install-recommends software-properties-common \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# Install required base packages
RUN apt-get update && apt-get install --yes --no-install-recommends \
    git \
    python3-venv \
    python3-pip \
    python3-colcon-common-extensions \
    && pip3 install --break-system-packages cogapp \
    && rm -rf /var/lib/apt/lists/*

# Update rosdep
RUN rosdep update --rosdistro $ROS_DISTRO

####################################################################################################

# Build slic3r_coverage_planner once and re-use the prebuilt install
FROM base AS slic3r

ENV DEBIAN_FRONTEND=noninteractive

# Fetch the slic3r planner from the repo (this will cache if unchanged)
COPY --link src/lib/slic3r_coverage_planner /opt/slic3r_workspace/src/slic3r_coverage_planner

WORKDIR /opt/slic3r_workspace
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y && \
    rm -rf /var/lib/apt/lists/*
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    MAKEFLAGS='-j$(nproc)' colcon build --packages-select slic3r_coverage_planner --merge-install \
    --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release"

####################################################################################################

# Assemble final image
FROM base AS assemble

ENV DEBIAN_FRONTEND=noninteractive

# Fetch the slic3r built earlier
COPY --link --from=slic3r /opt/slic3r_workspace/install /opt/prebuilt/slic3r_coverage_planner

COPY --link ./ /opt/open_mower_ros

# Delete prebuilt slic3r source and any local build artifacts
RUN rm -rf /opt/open_mower_ros/src/lib/slic3r_coverage_planner \
    /opt/open_mower_ros/build \
    /opt/open_mower_ros/install \
    /opt/open_mower_ros/log

WORKDIR /opt/open_mower_ros

# Install all dependencies via rosdep, plus packages rosdep may skip due to special characters in keys
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y && \
    apt-get install --no-install-recommends --yes \
    pkg-config \
    libcrypto++-dev \
    && rm -rf /var/lib/apt/lists/*

# Build the workspace with colcon, using prebuilt slic3r as underlay
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /opt/prebuilt/slic3r_coverage_planner/setup.bash && \
    MAKEFLAGS='-j$(nproc)' colcon build --merge-install \
    --packages-ignore slic3r_coverage_planner \
    --parallel-workers $(nproc) \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "export OM_SOFTWARE_VERSION=$(git describe --tags || git describe --always || echo 'v0.0.0')" > version_info.env

RUN mkdir -p /config
COPY ./docker/openmower_entrypoint.sh /openmower_entrypoint.sh
RUN chmod +x /openmower_entrypoint.sh

# Create openmower runtime user so that we can run the container non-root
ARG USERNAME=openmower
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} 2>/dev/null || true \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && adduser ${USERNAME} dialout \
    && chown -R ${USER_UID}:${USER_GID} /config /home/${USERNAME}

# Append our bashrc extension to user's .bashrc
COPY ./docker/assets/openmower-bashrc.sh /tmp/openmower-bashrc.sh
RUN set -eux; \
    BRC="/home/${USERNAME}/.bashrc"; \
    touch "$BRC"; \
    chown ${USER_UID}:${USER_GID} "$BRC"; \
    printf '\n# OpenMower bashrc extension\n' >> "$BRC"; \
    cat /tmp/openmower-bashrc.sh >> "$BRC"; \
    rm -f /tmp/openmower-bashrc.sh

USER ${USERNAME}

ENTRYPOINT ["/openmower_entrypoint.sh"]
CMD ["bash", "-c", "ros2 launch open_mower open_mower_launch.py"]
