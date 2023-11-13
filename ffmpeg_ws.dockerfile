FROM ros:humble-ros-base-jammy

ENV ROS_DISTRO=humble
ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}video

RUN apt update \
    && apt install -y software-properties-common \
    && add-apt-repository ppa:ubuntuhandbook1/ffmpeg6 \
    && apt update \
    && apt install -y \
    ros-${ROS_DISTRO}-common-interfaces \
    ffmpeg \
    libavcodec-dev \
    libavutil-dev \
    libswscale-dev \
    libavformat-dev \
    libavfilter-dev

RUN echo ". /opt/ros/${ROS_DISTRO}/setup.bash" > ~/.bashrc