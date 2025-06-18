# ===================================================================
# Dockerfile per a ROS2 Humble amb dos Workspaces:
# - agilex_ws (de la imatge base)
# - ROS2_rUBot_mecanum_ws (afegit)
# ===================================================================

# 1. Imatge Base (ja conté /root/agilex_ws)
FROM theconstructai/limo:humble-v3

# Canviem a l'intèrpret de comandes a bash per a més consistència
SHELL ["/bin/bash", "-c"]

# 2. Instal·lació de dependències del sistema i Python
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    python3-colcon-common-extensions \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# 3. Instal·lació de paquets de Python amb pip
RUN pip3 install ultralytics "numpy<2.0"

# 4. Creació i construcció del segon Workspace (ROS2_rUBot_mecanum_ws)
WORKDIR /root
RUN git clone https://github.com/manelpuig/ROS2_rUBot_mecanum_ws.git
WORKDIR /root/ROS2_rUBot_mecanum_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# 5. Configuració de l'entorn del contenidor per a sessions interactives
# -> Afegim al .bashrc les comandes per carregar ROS i AMBDÓS workspaces.
#    L'últim workspace carregat té preferència.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/agilex_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /root/ROS2_rUBot_mecanum_ws/install/setup.bash" >> /root/.bashrc

# 6. Comanda per defecte
# -> Obre un terminal bash interactiu amb tot l'entorn carregat.
CMD ["/bin/bash"]
