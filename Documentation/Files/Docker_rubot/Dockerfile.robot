# Utilitzem una imatge base més adient, que ja porta eines bàsiques de ROS2
FROM ros:humble-ros-base

# Instal·la dependències essencials per construir i executar
# Combinem les dues comandes apt-get en una per optimitzar les capes de la imatge
RUN apt-get update && apt-get install -y \
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
    && rm -rf /var/lib/apt/lists/*

# Set up the working directory for ROS workspace
WORKDIR /root

#Installing Serial comunication module for arduino nan
RUN pip install pyserial
# Copy and unzip the ROS2_rUBot_mecanum_ws.zip file
#COPY ROS2_rUBot_mecanum_ws.zip /root/ROS2_rUBot_mecanum_ws.zip
#RUN unzip /root/ROS2_rUBot_mecanum_ws.zip -d /root/ && \
#    rm /root/ROS2_rUBot_mecanum_ws.zip
# Clone the rplidar_ros package
#RUN git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git /root/ROS2_rUBot_mecanum_ws/src/rplidar_ros

# Build the ROS workspace
#RUN bash -c "source /opt/ros/humble/setup.bash && cd /root/ROS2_rUBot_mecanum_ws && colcon build"

# Add workspace and ROS environment setup to .bashrc
#RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
#    echo 'source /root/ROS2_rUBot_mecanum_ws/install/setup.bash' >> /root/.bashrc && \
#    echo 'cd /root/ROS2_rUBot_mecanum_ws' >> /root/.bashrc

# Copy the rubot_entrypoint.sh into the image
COPY rubot_entrypoint.robot.sh /root/rubot_entrypoint.robot.sh
RUN cd /root &&\
    chown root:root /root/rubot_entrypoint.robot.sh &&\
    chmod 755 /root/rubot_entrypoint.robot.sh

# Set permissions for rubot_nano_driver.py (assuming it's in your workspace)
#RUN cd /root/ROS2_rUBot_mecanum_ws/src/Robot_drivers/my_robot_driver/my_robot_driver && \
#    chown root:root rubot_nano_driver.py && \
#    chmod 4777 rubot_nano_driver.py

# Set the default command to run when the container starts
CMD ["bash"]