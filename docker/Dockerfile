# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:noetic-perception-focal

ENV ROS_DISTRO=noetic

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}

RUN apt-get update && \
    apt-get install -y \
        python3-catkin-tools \
        python3-pip \
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-trajectory-msgs \
        ros-noetic-control-msgs \
        ros-noetic-joint-state-publisher \
        ros-noetic-joint-trajectory-controller \
        ros-noetic-resource-retriever \
        ros-noetic-vision-msgs \
    && apt-get clean \ 
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy\
    opencv-python \
    ultralytics \
    pyrr

RUN pip3 install --upgrade numpy

# Source setup.bash of ROS2 humble
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# RUN rosdep update && \
#     rosdep install

# Copy the ros_entrypoint.sh script from ros:ros-base
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

# Set the ros_entrypoint.sh script as the entry point
ENTRYPOINT ["/ros_entrypoint.sh"]

# Create a new workspace folder for ROS2
RUN mkdir -p /root/ros_ws/src

# Set the new workspace folder as the working directory
WORKDIR /root/ros_ws
