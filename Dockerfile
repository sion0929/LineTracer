# competition image (design §3.1). Base: course lab image with ROS 2 Jazzy + Gazebo Harmonic + ros_gz.
ARG BASE_IMAGE=rp/ros2:jazzy-lab
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
WORKDIR /ws

COPY src ./src

# Only the packages the competition launch needs; dev-only GCS and tests are not built.
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-up-to line_follower_bringup \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF && \
    rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "line_follower_bringup", "competition.launch.py"]
