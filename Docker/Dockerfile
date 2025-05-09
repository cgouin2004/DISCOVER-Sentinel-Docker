FROM arm64v8/ros:noetic
ENV ROS_DISTRO=noetic

# Turn off interactive shell during building
#ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
 wget \
 git \
 bash-completion \
 build-essential \
 sudo \
 && rm -rf /var/lib/apt/lists/*

RUN cd ~/ && wget https://bootstrap.pypa.io/get-pip.py \
&& python3 get-pip.py && rm get-pip.py
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install --upgrade pymavlink
RUN python3 -m pip install matplotlib

RUN apt-get update && apt-get install -y \
 vim \
 python3 \
 wireless-tools \
 iw \
 net-tools \
 iputils-ping \
 ros-noetic-mavros \
 ros-noetic-mavros-extras \
 ros-noetic-image-transport \
 ros-noetic-image-common \
 ros-noetic-camera-info-manager \
 ros-noetic-tf2-geometry-msgs \
 ros-noetic-cv-bridge \
 && rm -rf /var/lib/apt/lists/*

run bash /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

run apt-get update && apt-get upgrade -y
 
# Now create the same user as the host itself
ARG UID_=1000
ARG GID_=1000
RUN addgroup --gid ${GID_} ros
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID_} --gid ${GID_} ros
RUN usermod -a -G dialout ros
ADD config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Choose to run as user
ENV USER ros
USER ros 
# Change HOME environment variable
ENV HOME /home/${USER} 
# workspace setup
RUN mkdir -p ${HOME}/ros_ws/src


WORKDIR ${HOME}/ros_ws/src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_init_workspace"
WORKDIR ${HOME}/ros_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make"


# set up environment
COPY config/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ros /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc

# Change entrypoint to source ~/.bashrc and start in ~
COPY config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ros /ros_entrypoint.sh ;

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
