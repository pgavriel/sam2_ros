ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO} AS deps

ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3 \
    python3-pip \
    python3-venv \
    curl \
    ffmpeg \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

# Clone SAM2 and download model checkpoints
WORKDIR /root/models
RUN git clone https://github.com/facebookresearch/sam2.git
WORKDIR /root/models/sam2
RUN cd checkpoints && ./download_ckpts.sh && cd ..

# Upgrade pip and install Python requirements
# RUN pip3 install --no-cache-dir --break-system-packages --upgrade pip
RUN pip3 install --no-cache-dir --break-system-packages  -e .
# Install PyTorch with CUDA support (correct for CUDA 12.1)
RUN pip3 install --break-system-packages torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

RUN pip install --break-system-packages --ignore-installed opencv-python 
RUN pip install --break-system-packages matplotlib numpy==1.26.4
RUN apt-get update && apt-get install -y python3-tk


# Create ros2_ws and copy files
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
COPY ./sam2_msgs /root/ros2_ws/src/sam2_msgs/
COPY ./sam2_server /root/ros2_ws/src/sam2_server/
# Install ROS Package Dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Add the startup script
COPY ./docker_startup.sh /root/startup.sh
RUN chmod +x /root/startup.sh

# Build the ws with colcon
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# Source the ROS 2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["/bin/bash"]
