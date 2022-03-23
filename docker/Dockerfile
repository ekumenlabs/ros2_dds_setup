# Download base image ubuntu 20.04
FROM ubuntu:20.04

# Arguments for building
ARG USERID
ARG USER

# Image name
ARG "Ubuntu base for ros2 foxy development in ubuntu focal."

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Copy installation scripts, and perform the dependency installation.
COPY docker/install.sh .
COPY docker/requirements.txt .
COPY docker/requirements_ros.txt .
RUN ./install.sh
RUN rm install.sh requirements.txt requirements_ros.txt

# Create a user with passwordless sudo
RUN adduser --uid $USERID --gecos "ekumen developer" --disabled-password $USER
RUN adduser $USER sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN echo "export QT_X11_NO_MITSHM=1" >> /home/$USER/.bashrc
USER $USER

# Run rosdep init and update.
RUN sudo rosdep init && rosdep update

# Creates the src folder of the workspace
RUN mkdir -p /home/$USER/ws/src

# Adds to bashrc the ros2 foxy overlay sourcing.
RUN echo "source /opt/ros/foxy/setup.bash" >> /home/$USER/.bashrc

# Defines a workspace folder.
WORKDIR /home/$USER/ws

CMD ["/bin/bash"]
