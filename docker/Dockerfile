# rich base image
FROM focal-noetic-ego:3.0 

# install build and runtime dependencies
RUN set -eux; \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 \
    || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654; \
    apt-get update && apt-get install -y --no-install-recommends \
    python3-catkin-tools \
    python3-rosdep \
    psmisc \
    clang-11 \
    bash-completion \
    htop

# copy project to container temporarily for installing dependencies
COPY src /home/src
RUN /bin/bash -c "apt-get update && \
    source /opt/ros/noetic/setup.bash && \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi && \
    rosdep update --rosdistro=noetic && \
    rosdep install -y --from-paths /home/src --ignore-src --rosdistro noetic" \
 && rm -rf /var/lib/apt/lists/*

# remove temporary source files after installing dependencies
RUN rm -rf /home/src

# set entrypoint including process of sourcing ros environment
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

# set default command
CMD ["/bin/bash"]
