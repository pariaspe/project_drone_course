FROM aerostack2/humble:control-v1.1.3

USER root
SHELL ["/bin/bash", "-c"]

RUN echo "set -g mouse on" > ~/.tmux.conf 

WORKDIR /root/project_drone_course
COPY . /root/project_drone_course

WORKDIR /root/project_drone_course/control/drone_course_ws
RUN source /opt/ros/humble/setup.bash \
  && source /root/aerostack2_ws/install/setup.bash \
  && (rosdep init 2>/dev/null || true) \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y \
  && colcon build --symlink-install

RUN . /opt/ros/humble/setup.sh \
  && . /root/aerostack2_ws/install/setup.sh \
  && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
  && echo "source /root/aerostack2_ws/install/setup.bash" >> /root/.bashrc \
  && echo "[ -f /root/project_drone_course/control/drone_course_ws/install/setup.bash ] && source /root/project_drone_course/control/drone_course_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/project_drone_course
CMD ["bash"]
