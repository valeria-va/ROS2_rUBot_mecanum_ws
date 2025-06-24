# Utilitzem una imatge base més adient, que ja porta eines bàsiques de ROS2
FROM ros:humble-ros-base

# Canviem a l'intèrpret de comandes a bash per a més consistència
SHELL ["/bin/bash", "-c"]

# Instal·la dependències essencials per construir i executar
# Combinem les dues comandes apt-get en una per optimitzar les capes de la imatge
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Eines de construcció
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-pip \
    wget \
    unzip \
    git \
    iputils-ping \
    # Dependències del teu robot (llista mínima)
    libusb-1.0-0-dev \
    libeigen3-dev \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-geometry \
    ros-humble-image-publisher \
    ros-humble-image-transport \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf-transformations \
    ros-humble-usb-cam \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-demo-nodes-cpp \
    # >>> AFEGIT: Eines gràfiques de ROS2 <<<
    ros-humble-rviz2 \
    ros-humble-rqt-graph \
    ros-humble-rqt-plot \
    # >>> AFEGIT: Dependències per a visualització remota (X11 Forwarding) <<<
    x11-apps \
    libgl1-mesa-glx \
    mesa-utils \
    libqt5x11extras5 \
    libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/*

# Install Yolo
RUN pip3 install ultralytics "numpy<2.0"

# Install Serial comunication module for arduino nano
RUN pip install pyserial
# Create the Workspace (ROS2_rUBot_mecanum_ws)
WORKDIR /root
RUN git clone https://github.com/manelpuig/ROS2_rUBot_mecanum_ws.git
WORKDIR /root/ROS2_rUBot_mecanum_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Source and .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ROS2_rUBot_mecanum_ws/install/setup.bash" >> /root/.bashrc

# Set the default command to run when the container starts
CMD ["/bin/bash"]