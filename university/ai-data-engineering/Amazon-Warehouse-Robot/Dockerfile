FROM osrf/ros:humble-desktop-full

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/" >> ~/.bashrc

RUN sudo apt update
RUN sudo apt install -y\
    clang-14 \
    clang-format-14 \
    clang-tidy-14 \
    libeigen3-dev \
    libignition-gazebo6-dev \
    libpoco-dev \
    python3-pip \
    ros-humble-ament-clang-format \
    ros-humble-ament-cmake-clang-format \
    ros-humble-ament-cmake-clang-tidy \
    ros-humble-ament-flake8 \
    ros-humble-angles \
    ros-humble-control-msgs \
    ros-humble-control-toolbox \
    ros-humble-controller-interface \
    ros-humble-controller-manager \
    ros-humble-generate-parameter-library \
    ros-humble-hardware-interface \
    ros-humble-hardware-interface-testing \
    ros-humble-launch-testing \
    ros-humble-moveit \
    ros-humble-pinocchio \
    ros-humble-realtime-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-control-test-assets \
    ros-humble-turtlebot3* \
    ros-humble-xacro

RUN mkdir -p ~/ros2_ws/src