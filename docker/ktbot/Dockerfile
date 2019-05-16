FROM tensorflow/tensorflow:1.13.1-gpu

# RUN apt update && apt install -y curl gnupg2 lsb-release locales

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
    ros-kinetic-ros-base \
    ros-kinetic-joint-state-controller \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-diff-drive-controller \
    ros-kinetic-rqt-common-plugins \
    ros-kinetic-rqt-robot-plugins \
    ros-kinetic-rosserial-python \
    ros-kinetic-mqtt-bridge \
    ros-kinetic-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-xacro \
    libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-kinetic-moveit-core \
    ros-kinetic-moveit-visual-tools \
    ros-kinetic-moveit-ros-planning-interface \
    && rm -rf /var/lib/apt/lists/*

# Test locale
# RUN locale-gen en_US en_US.UTF-8
# RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# RUN export LANG=en_US.UTF-8
#
# Install ros2 base libs so we can publish
# RUN curl http://repo.ros2.org/repos.key | apt-key add -
# RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
# RUN apt update && apt install -y ros-crystal-ros-base
# RUN echo "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash" >> ~/.bashrc

# Install colcon to allow builds
# RUN apt install -y python3-colcon-common-extensions

# Auto-install workspace source files
# RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

RUN echo "source /opt/ros/kinetic/setup.bash && cd /catkin_ws" >> ~/.bashrc

# ENTRYPOINT [ "sh", "-c", "echo $HOME" ]
