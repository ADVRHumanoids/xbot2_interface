FROM ros:jazzy-ros-core-noble
ENV USER_ID=1000

# install build dependencies
RUN apt-get update && apt-get install -y python3-venv python3-pip clang sudo git cmake build-essential

# delete ubuntu user
RUN sudo userdel ubuntu

# create a regular user, switch to it, cd to its home
RUN useradd -u ${USER_ID} -ms /bin/bash user
RUN adduser user sudo
RUN echo 'user:user' | chpasswd
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER user
WORKDIR /home/user

# set debian noninteractive mode (for apt)
ENV DEBIAN_FRONTEND=noninteractive

# otherwise autocompletion is broken with apt
RUN sudo rm /etc/apt/apt.conf.d/docker-clean

# install more packages
RUN sudo apt-get update && sudo apt-get install -y \
ros-$ROS_DISTRO-urdf \
ros-$ROS_DISTRO-srdfdom \
ros-$ROS_DISTRO-tf2-eigen \
ros-$ROS_DISTRO-tf2-geometry-msgs \
ros-$ROS_DISTRO-geometric-shapes \
libhdf5-dev \
libboost-dev \
pybind11-dev \
libboost-chrono-dev \
libboost-serialization-dev \ 
libboost-filesystem-dev \
liboctomap-dev \
libassimp-dev