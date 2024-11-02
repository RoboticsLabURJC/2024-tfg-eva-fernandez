# Base image
FROM osrf/ros:humble-desktop

# Instalar herramientas necesarias
RUN apt-get update && apt-get install -y \
    git \
    gedit \
    wget \
    gnupg2 \
    python3 \
    python3-distutils \
    && curl -sSL https://bootstrap.pypa.io/get-pip.py -o get-pip.py \
    && python3 get-pip.py \
    && rm get-pip.py \
    && rm -rf /var/lib/apt/lists/*

# Instalar Gazebo
# Actualiza el sistema y instala las dependencias necesarias
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg

# Añade la clave GPG del repositorio de Gazebo
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Añade el repositorio de Gazebo a la lista de fuentes
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list

# Actualiza el sistema e instala gz-harmonic
RUN apt-get update && apt-get install -y \
    gz-harmonic

# Instalar ROS-Gazebo sim y bridge
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge

# Instalar Visual Studio Code
RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | apt-key add - \
    && echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list \
    && apt-get update \
    && apt-get install -y code

# Configurar Git
RUN git config --global user.name "evafc003" \
    && git config --global user.email "e.fernandezde.2021@alumnos.urjc.es"

# Instalar pybullet
RUN pip install pybullet

# Limpiar
RUN apt-get clean

# Establecer el directorio de trabajo
WORKDIR /home

