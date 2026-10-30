# syntax=docker/dockerfile:1
# Official ROS image supplies Ubuntu 24.04 and the official ROS apt repository.
ARG ROS_IMAGE=ros:jazzy-ros-base
FROM ${ROS_IMAGE} AS environment
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_APT_MIRROR=http://packages.ros.org/ros2/ubuntu
ARG UBUNTU_APT_MIRROR=https://archive.ubuntu.com/ubuntu
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && sed -i "s|http://archive.ubuntu.com/ubuntu|${UBUNTU_APT_MIRROR}|g; s|http://security.ubuntu.com/ubuntu|${UBUNTU_APT_MIRROR}|g" /etc/apt/sources.list.d/ubuntu.sources \
    && sed -i "s|http://packages.ros.org/ros2/ubuntu|${ROS_APT_MIRROR}|g" /etc/apt/sources.list.d/* \
    && sed -i -E 's/^Types: deb deb-src/Types: deb/; /^deb-src /d' /etc/apt/sources.list.d/* \
    && apt-get update -o APT::Update::Error-Mode=any -o Acquire::Retries=3 && apt-get install -y --no-install-recommends -o Acquire::Retries=3 \
    ca-certificates curl gnupg build-essential cmake git patch pkg-config \
    python3-venv python3-pip python3-colcon-common-extensions \
    python3-numpy python3-scipy python3-yaml python3-pil \
    libeigen3-dev libpcl-dev libopencv-dev libfmt-dev libboost-thread-dev \
    libssl-dev nlohmann-json3-dev libtinyxml2-dev libsqlite3-dev libasio-dev \
    libapr1-dev libgoogle-glog-dev libgtest-dev libprotobuf-dev protobuf-compiler \
    ros-jazzy-sophus ros-jazzy-pcl-ros ros-jazzy-pcl-conversions \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-sensor-msgs-py \
    ros-jazzy-ament-cmake-auto ros-jazzy-demo-nodes-cpp \
    ros-jazzy-rmw-fastrtps-cpp ros-jazzy-fastrtps ros-jazzy-fastcdr \
    ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces \
    libgl1-mesa-dri libegl1 libgl1 mesa-utils \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main' > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends gz-harmonic libgz-sim8-dev \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/navigation-venv \
    && /opt/navigation-venv/bin/pip install --no-cache-dir torch==2.7.1 --index-url https://download.pytorch.org/whl/cpu
ENV NAVIGATION_ACTOR_PYTHON=/opt/navigation-venv/bin/python3
ENV PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
WORKDIR /workspace/navigation

FROM environment AS native_builder
COPY build_support /workspace/navigation/build_support
COPY third_party /workspace/navigation/third_party
COPY src/sdk /workspace/navigation/src/sdk
COPY src/localization/frontend /workspace/navigation/src/localization/frontend
COPY src/planning/scan /workspace/navigation/src/planning/scan
COPY src/planning/pct /workspace/navigation/src/planning/pct
COPY src/localization/backend/native /workspace/navigation/src/localization/backend/native
COPY src/simulation/actuators /workspace/navigation/src/simulation/actuators
COPY src/common/communication /workspace/navigation/src/common/communication
ARG BUILD_JOBS=2
RUN source /opt/ros/jazzy/setup.bash && python3 build_support/build.py --jobs ${BUILD_JOBS}

FROM environment AS navigation
COPY --from=native_builder /workspace/navigation/install /workspace/navigation/install
COPY . /workspace/navigation
RUN mkdir -p /workspace/navigation/runs && chown 1000:1000 /workspace/navigation/runs
USER 1000:1000
ENTRYPOINT ["python3", "/workspace/navigation/start.py"]
