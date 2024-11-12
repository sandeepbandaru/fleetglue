# Use a ROS2 base image
FROM ros:foxy

RUN apt-get clean

# Install ROS2 action libraries
RUN apt-get update && \
    apt-get install -y ros-foxy-rclpy ros-foxy-action-msgs

# Copy the RN2 code into the container
COPY . /ros2_ws/src/
WORKDIR /ros2_ws

# list the contents of the workspace
RUN ls -l /ros2_ws/src

# Install python dependencies
RUN apt-get install -y python3-pip
RUN python3 -m pip install -U requests



# Build the workspace
RUN . /opt/ros/foxy/setup.sh && \
    colcon build

# Source the ROS2 environment and run RN2
CMD ["bash", "-c", "source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run rn2 rn2_node"]
