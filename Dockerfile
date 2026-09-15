FROM ubuntu:22.04

# 避免安裝過程出現互動視窗，導致建置卡住
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# 分兩階段安裝：先裝環境設定工具，再裝 ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# 加入 ROS 2 金鑰與儲存庫
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安裝 ROS 2
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 自動 source 環境
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# 在 Dockerfile 中加入
RUN apt-get update && apt-get install -y python3-rosdep \
    && rosdep init && rosdep update