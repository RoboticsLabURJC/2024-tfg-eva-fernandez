# Base image
FROM osrf/ros:humble-desktop

# Instalar herramientas necesarias
RUN apt-get update && apt-get install -y \
    git \
    gedit \
    wget \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Instalar Gazebo
# Actualizar y instalar dependencias necesarias
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Agregar la clave del repositorio de Gazebo
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Agregar el repositorio de Gazebo
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# Actualizar el índice de paquetes e instalar Gazebo
RUN apt-get update && apt-get install -y gz-ionic && rm -rf /var/lib/apt/lists/*


# Instalar Visual Studio Code
RUN wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | apt-key add - \
    && echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list \
    && apt-get update \
    && apt-get install -y code

# Configurar Git
RUN git config --global user.name "evafc003" \
    && git config --global user.email "e.fernandezde.2021@alumnos.urjc.es"

# Limpiar
RUN apt-get clean

# Establecer el directorio de trabajo
WORKDIR /home

