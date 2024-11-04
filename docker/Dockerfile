###########################################################
# Basic ROS1 image containing roscpp tutorials
###########################################################
FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-c"]

# install ros packages
RUN source /opt/ros/noetic/setup.bash \
  && apt-get update -y \
  && apt-get install -y --no-install-recommends \
     ros-noetic-roscpp-tutorials \
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*

# Set up launcher shell script
COPY launch.sh /
RUN chmod 775 /launch.sh

# Set up the entrypoint
#COPY entrypoint.sh /
#RUN chmod 775 /entrypoint.sh
#ENTRYPOINT [ "/entrypoint.sh" ]
