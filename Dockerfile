# 假設你原本的基底是 ros:humble (如果不同，請保留你原本的 FROM 第一行)
FROM osrf/ros:humble-desktop

# 設定環境變數，避免安裝過程中卡在時區或互動式選項
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    wget \
    tar \
    gzip \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    fontconfig \
    fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-cv-bridge \
    ros-humble-message-filters \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --upgrade pip

RUN pip3 install --no-cache-dir --ignore-installed sympy
RUN pip3 install --no-cache-dir torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install --no-cache-dir ultralytics
RUN pip3 install --no-cache-dir pyrealsense2 pyserial

RUN pip3 install --no-cache-dir "numpy==1.26.4" "opencv-python==4.9.0.80"

WORKDIR /workspace

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc

CMD ["/bin/bash"]