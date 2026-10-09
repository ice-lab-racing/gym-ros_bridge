# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       python3-transforms3d \
                       libeigen3-dev \
                       tmux \
                       ros-jazzy-rviz2
RUN apt-get -y dist-upgrade

# pip: install into the system python that rclpy is built for (PEP 668 blocks
# this by default, PIP_BREAK_SYSTEM_PACKAGES bypasses the safety lock) and stay on the apt numpy 1.x that ros-jazzy is built against
RUN echo 'numpy<2' > /etc/pip-constraints.txt
ENV PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_CONSTRAINT=/etc/pip-constraints.txt

# f1tenth gym: the repo is private, so it is cloned with the GH_TOKEN from .env.
# .env is mounted as a build secret for this step only, it is not stored in the image
RUN --mount=type=secret,id=dotenv,required=true \
    set -a && source /run/secrets/dotenv && set +a && \
    : "${GH_TOKEN:?is not set in .env}" && \
    git -c credential.helper='!f() { echo username=x-access-token; echo "password=$GH_TOKEN"; }; f' \
        clone -b develop https://github.com/ice-lab-racing/gym-environment
RUN cd gym-environment && \
    pip3 install -e .

# ros2 gym bridge
RUN mkdir -p sim_ws/src/gym-ros_bridge
COPY . /sim_ws/src/gym-ros_bridge
RUN source /opt/ros/jazzy/setup.bash && \
    cd sim_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro jazzy -y && \
    colcon build

WORKDIR '/sim_ws'
ENTRYPOINT ["/bin/bash"]
