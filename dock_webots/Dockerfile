FROM ros:foxy
RUN apt update && apt install -y kmod wget software-properties-common
ADD nvidia-driver-runfile.run /tmp/nvidia-driver-runfile.run
ADD pycharm-community-2021.1.2.tar.gz /opt/
RUN sh /tmp/nvidia-driver-runfile.run -a -N --ui=none --no-kernel-module
RUN rm /tmp/nvidia-driver-runfile.run
RUN wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add - && sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/' && apt update && DEBIAN_FRONTEND=noninteractive apt install -y ros-foxy-webots-ros2 nano ros-foxy-rqt* python3-opencv ros-foxy-turtlesim ros-foxy-image-pipeline ros-foxy-rviz2 nano ros-foxy-rqt* python3-opencv ros-foxy-turtlesim ros-foxy-image-pipeline ros-foxy-rviz2 ros-foxy-ros2-control ros-foxy-ros2-controllers
RUN DEBIAN_FRONTEND=noninteractive apt install -y webots
RUN apt install -y python3-pip
RUN pip install open3d
RUN useradd -m aayush && yes notreal | passwd aayush
RUN adduser aayush sudo
RUN bash -c "source /opt/ros/foxy/setup.bash" && bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh" && bash -c "export _colcon_cd_root=~/ros2_install"
RUN echo "source /opt/ros/foxy/setup.bash" >> /home/aayush/.bashrc
RUN echo "source /home/aayush/sahayak_root/ws/install/setup.bash" >>/home/aayush/.bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/aayush/.bashrc
RUN echo "export _colcon_cd_root=~/ros2_install" >> /home/aayush/.bashrc
USER aayush
CMD /bin/bash -c "source /home/aayush/.bashrc && /opt/pycharm-community-2021.1.2/bin/pycharm.sh"
