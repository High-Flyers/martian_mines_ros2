FROM ubuntu:22.04 AS setup-ros-gpg

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages and setup the ROS repository
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Add the ROS GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# Configure the ROS repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

FROM ros:humble-ros-base AS build-librealsense

ARG LIBRS_VERSION=2.55.1

COPY --from=setup-ros-gpg /usr/share/keyrings/ros2-latest-archive-keyring.gpg /usr/share/keyrings/ros2-latest-archive-keyring.gpg

# To avoid waiting for input during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Builder dependencies installation
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    curl \
    python3 \
    python3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Download sources
WORKDIR /usr/src
RUN curl https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$LIBRS_VERSION -o librealsense.tar.gz
RUN tar -zxf librealsense.tar.gz \
    && rm librealsense.tar.gz
RUN ln -s /usr/src/librealsense-$LIBRS_VERSION /usr/src/librealsense

# Build and install
WORKDIR /usr/src/librealsense
RUN mkdir build
WORKDIR /usr/src/librealsense/build
RUN cmake \
    -DCMAKE_C_FLAGS_RELEASE="${CMAKE_C_FLAGS_RELEASE} -s" \
    -DCMAKE_CXX_FLAGS_RELEASE="${CMAKE_CXX_FLAGS_RELEASE} -s" \
    -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_PYTHON_BINDINGS:bool=true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DFORCE_RSUSB_BACKEND=ON \
    -DCMAKE_BUILD_TYPE=Release ../ \
    && make -j$(($(nproc)-1)) all \
    && make install

FROM ros:humble-ros-base AS base

COPY --from=setup-ros-gpg /usr/share/keyrings/ros2-latest-archive-keyring.gpg /usr/share/keyrings/ros2-latest-archive-keyring.gpg

COPY --from=build-librealsense /opt/librealsense /usr/local/
COPY --from=build-librealsense /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=build-librealsense /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=build-librealsense /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/

ARG USERNAME=highflyers
ARG ROS_DISTRO=humble
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Install general dependencies
RUN apt-get update --allow-releaseinfo-change && \
    apt-get install -y --no-install-recommends --fix-missing \
    openssh-client \
    build-essential \
    cmake \
    udev \
    ros-dev-tools \
    python3-pip \
    ros-humble-geographic-msgs \
    ros-humble-vision-msgs \
    ros-humble-std-msgs \
    ros-humble-image-geometry \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-launch \
    ros-humble-launch-xml \
    ros-humble-launch-ros \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install ultralytics dill pyrr shapely transitions matplotlib opencv-contrib-python cv_bridge
RUN pip3 install -U numpy==1.26.4 numpy-quaternion Jetson.GPIO

# Create a non-root user with sudo privileges
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

RUN groupadd -f -r gpio
RUN usermod -a -G gpio ${USERNAME}

USER ${USERNAME}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Clone repositories to workspace
ENV ROS_WORKSPACE=/home/${USERNAME}/ws

## Install realsense-ros wrapper
RUN mkdir -p ${ROS_WORKSPACE}/src
WORKDIR ${ROS_WORKSPACE}/src
RUN git clone --depth 1 https://github.com/IntelRealSense/realsense-ros.git -b 4.55.1

WORKDIR ${ROS_WORKSPACE}/src
RUN git clone "https://github.com/eProsima/Micro-XRCE-DDS-Agent.git" --branch v2.4.2 && \
    git clone "https://github.com/PX4/px4_msgs.git" --branch "release/1.15"

# Install ROS dependencies
WORKDIR $ROS_WORKSPACE
RUN sudo apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} --skip-keys=librealsense2 -y

# Build
WORKDIR $ROS_WORKSPACE
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build

# Copy simulation package to workspace
WORKDIR $ROS_WORKSPACE/src
RUN pwd
COPY . martian_mines_ros2

WORKDIR $ROS_WORKSPACE
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" 

RUN echo "source \"/opt/ros/${ROS_DISTRO}/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "source \"${ROS_WORKSPACE}/install/setup.bash\"" >> "/home/${USERNAME}/.bashrc" && \
    echo "export LRS_LOG_LEVEL=ERROR" >> "/home/${USERNAME}/.bashrc"

RUN sudo sed -i '$i source $ROS_WORKSPACE/install/setup.bash' /ros_entrypoint.sh

RUN sudo mkdir -p /usr/local/share/middleware_profiles
COPY middleware_profiles/cyclone_profile.xml /usr/local/share/middleware_profiles/

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI="file:///usr/local/share/middleware_profiles/cyclone_profile.xml"


ENTRYPOINT ["/ros_entrypoint.sh"]
