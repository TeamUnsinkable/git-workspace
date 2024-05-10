FROM ros:humble

SHELL ["/bin/bash", "-c"]
WORKDIR /app

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install ros-humble-image-tools -y
ENV ROS_DOMAIN_ID 69
COPY ./src ./src

RUN . /opt/ros/humble/setup.sh && \
	rosdep install -i --from-paths src

RUN . /opt/ros/humble/setup.sh && \
	colcon build

# COPY scripts/udev-rules /etc/udev/rules.d/
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

