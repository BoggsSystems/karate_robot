FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    curl \
    git \
    gnupg2 \
    lsb-release \
    python3-pip \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe

RUN python3 -m pip install --no-cache-dir \
    rosdep \
    vcstool \
    colcon-common-extensions

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-ros-gz \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# Install CPU-only PyTorch (saves ~4GB vs CUDA version)
RUN python3 -m pip install --no-cache-dir \
    torch --index-url https://download.pytorch.org/whl/cpu

RUN python3 -m pip install --no-cache-dir \
    "stable-baselines3[extra]" \
    gymnasium \
    tensorboard

# Downgrade numpy for compatibility with system matplotlib
RUN python3 -m pip install --no-cache-dir "numpy<2"

WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

COPY . /ros2_ws/src/karate_robot

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths /ros2_ws/src/karate_robot/simulation/ros2 --ignore-src -r -y || true && \
    colcon build --base-paths /ros2_ws/src/karate_robot/simulation/ros2"

COPY docker/entrypoint.sh /ros2_ws/entrypoint.sh
RUN chmod +x /ros2_ws/entrypoint.sh

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
