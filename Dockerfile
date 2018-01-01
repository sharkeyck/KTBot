FROM ros:kinetic-robot

RUN apt-get update
RUN apt-get install -y zsh tmux ssh
RUN apt-get install -y ros-kinetic-ros-tutorials
RUN apt-get install -y ros-kinetic-stdr-simulator
RUN apt-get install -y ros-kinetic-rqt
RUN apt-get install -y ros-kinetic-rqt-common-plugins
RUN apt-get install -y ros-kinetic-turtlesim
RUN apt-get install -y ros-kinetic-turtlebot-teleop
#RUN apt-get install -y ros-kinetic-turtlebot-stage
#RUN apt-get install -y ros-kinetic-turtlebot-navigation
#RUN apt-get install -y ros-kinetic-turtlebot-rviz-launchers
#RUN apt-get install -y ros-kinetic-openni-camera ros-kinetic-openni-launch
#RUN apt-get install -y ros-kinetic-xv-11-laser-driver
RUN apt-get install -y ros-kinetic-mqtt-bridge
RUN apt-get install -y vim

RUN echo "source /opt/ros/kinetic/setup.bash" > /root/.bashrc
# place here your application's setup specifics
# CMD [ "roslaunch", "my-ros-app my-ros-app.launch" ]
VOLUME /volume

CMD "/usr/bin/tmux"
