FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux

RUN pip3 install transforms3d

# f1tenth gym
RUN git clone https://github.com/f1tenth/f1tenth_gym.git
RUN cd f1tenth_gym && \
    pip3 install -e .

# ros2 gym bridge
RUN mkdir -p /sim_ws/src/convoy/
COPY . /sim_ws/src/
RUN source /opt/ros/foxy/setup.bash && \
    cd /sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro foxy -y && \
    colcon build

WORKDIR '/sim_ws'
ENTRYPOINT ["/bin/bash"]
