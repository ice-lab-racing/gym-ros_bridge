# gym-ros_bridge

ROS 2 communication bridge that turns the ICELab F1TENTH gym environment into a ROS 2 simulation.

This repository is the ICELab fork of [f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros). The README has two parts:

- **[Part 1 – ICELab fork](#part-1--icelab-fork)**: how to build and run *this* repository. Follow this part.
- **[Part 2 – Original upstream README](#part-2--original-upstream-readme)**: the unmodified documentation of the original project, kept for reference (topics, configuration, keyboard teleop). Its installation instructions target ROS 2 Foxy and the public `f1tenth_gym` and do **not** apply to this fork.

---

# Part 1 – ICELab fork

## What is different from upstream

| | Upstream | This fork |
|---|---|---|
| ROS 2 distro | Foxy (Ubuntu 20.04, Python 3.8) | **Jazzy** (Ubuntu 24.04, Python 3.12) |
| Simulator | public `f1tenth/f1tenth_gym` | private [`ice-lab-racing/gym-environment`](https://github.com/ice-lab-racing/gym-environment), branch `develop` (needs Python ≥ 3.11) |
| Path in the container | `/sim_ws/src/f1tenth_gym_ros` | `/sim_ws/src/gym-ros_bridge` |
| Image / container name | `f1tenth_gym_ros` | `gym-ros_bridge` |

The ROS package name is unchanged: `f1tenth_gym_ros`.

## Prerequisites

- Docker with Docker Compose v2 (`docker compose`, not the legacy `docker-compose`).
- A GitHub access token that can read `ice-lab-racing/gym-environment` (as of now the repository is private, thus a PAT is needed).
  add the token in the .env file before building the image

`.env` is listed in `.gitignore` and `.dockerignore`: it is never committed and never copied into the image. 
During the build it is mounted as a secret for the clone step only.


## 1. Build the image

With Docker Compose (recommended):
```bash
docker compose build
```

Or with plain Docker:
```bash
docker build --secret id=dotenv,src=.env -t gym-ros_bridge .
```

The clone of `gym-environment` is cached by Docker. To pick up new commits from `develop`, rebuild with `docker compose build --no-cache sim`.

| Build error | Meaning |
|---|---|
| `failed to stat .../.env` / `secret dotenv: not found` | `.env` is missing, or `--secret` was not passed to `docker build` |
| `GH_TOKEN: is not set in .env` | `.env` has no `GH_TOKEN=` line |
| `Invalid username or token` | the token is wrong, expired or was regenerated |
| `Write access to repository not granted` (403) | the token is valid but cannot read `gym-environment`: check resource owner and repository access |

## 2. Start the containers

```bash
docker compose up -d
```

This starts two containers:

- `sim`: the bridge image, idle with a `bash` shell. This repository is mounted into it at `/sim_ws/src/gym-ros_bridge`, so edits on the host are visible in the container.
- `novnc`: a virtual desktop served in the browser, used to display RViz. Open [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html) and click **Connect**.


## 3. Open a terminal in the container

```bash
docker compose exec sim bash
```

`tmux` is also installed for multiple terminal sessions. 
The shell starts in `/sim_ws`.

## 4. Source ROS 2 Jazzy and build the workspace

The image already contains a built workspace. Rebuild after changing the code, `config/sim.yaml`, launch files or maps:

```bash
source /opt/ros/jazzy/setup.bash
cd /sim_ws
colcon build
```

## 5. Launch the simulation

In every new shell, source ROS 2 and the workspace first:

```bash
source /opt/ros/jazzy/setup.bash
source /sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

RViz appears in the noVNC browser tab. To run only the bridge node, without RViz and the map server:

```bash
ros2 run f1tenth_gym_ros gym_bridge --ros-args -r __node:=bridge --params-file /sim_ws/src/gym-ros_bridge/config/sim.yaml
```

Keyboard teleop (with `kb_teleop: True` in `config/sim.yaml`), from a second shell:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


# Part 2 – Original upstream README

> Everything below is the documentation of the original [f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros). It refers to ROS 2 Foxy, the public `f1tenth_gym` and the path `/sim_ws/src/f1tenth_gym_ros`. For this fork use Part 1 for installation and launching; the sections on topics, configuration and teleop still apply.

# F1TENTH gym environment ROS2 communication bridge
This is a containerized ROS communication bridge for the F1TENTH gym environment that turns it into a simulation in ROS2.

# Installation

**Supported System:**

- Ubuntu (tested on 20.04) native with ROS 2
- Ubuntu (tested on 20.04) with an NVIDIA gpu and nvidia-docker2 support
- Windows 10, macOS, and Ubuntu without an NVIDIA gpu (using noVNC)

This installation guide will be split into instruction for installing the ROS 2 package natively, and for systems with or without an NVIDIA gpu in Docker containers.

## Native on Ubuntu 20.04

**Install the following dependencies:**
- **ROS 2** Follow the instructions [here](https://docs.ros.org/en/foxy/Installation.html) to install ROS 2 Foxy.
- **F1TENTH Gym**
  ```bash
  git clone https://github.com/f1tenth/f1tenth_gym
  cd f1tenth_gym && pip3 install -e .
  ```

**Installing the simulation:**
- Create a workspace: ```cd $HOME && mkdir -p sim_ws/src```
- Clone the repo into the workspace:
  ```bash
  cd $HOME/sim_ws/src
  git clone https://github.com/f1tenth/f1tenth_gym_ros
  ```
- Update correct parameter for path to map file:
  Go to `sim.yaml` [https://github.com/f1tenth/f1tenth_gym_ros/blob/main/config/sim.yaml](https://github.com/f1tenth/f1tenth_gym_ros/blob/main/config/sim.yaml) in your cloned repo, change the `map_path` parameter to point to the correct location. It should be `'<your_home_dir>/sim_ws/src/f1tenth_gym_ros/maps/levine'`
- Install dependencies with rosdep:
  ```bash
  source /opt/ros/foxy/setup.bash
  cd ..
  rosdep install -i --from-path src --rosdistro foxy -y
  ```
- Build the workspace: ```colcon build```

## With an NVIDIA gpu:

**Install the following dependencies:**

- **Docker** Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker. If you followed the post-installation steps you won't have to prepend your docker and docker-compose commands with sudo.
- **nvidia-docker2**, follow the instructions [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) if you have a support GPU. It is also possible to use Intel integrated graphics to forward the display, see details instructions from the Rocker repo. If you are on windows with an NVIDIA GPU, you'll have to use WSL (Windows Subsystem for Linux). Please refer to the guide [here](https://developer.nvidia.com/cuda/wsl), [here](https://docs.nvidia.com/cuda/wsl-user-guide/index.html), and [here](https://dilililabs.com/zh/blog/2021/01/26/deploying-docker-with-gpu-support-on-windows-subsystem-for-linux/).
- **rocker** [https://github.com/osrf/rocker](https://github.com/osrf/rocker). This is a tool developed by OSRF to run Docker images with local support injected. We use it for GUI forwarding. If you're on Windows, WSL should also support this.

**Installing the simulation:**

1. Clone this repo
2. Build the docker image by:
```bash
$ cd f1tenth_gym_ros
$ docker build -t f1tenth_gym_ros -f Dockerfile .
```
3. To run the containerized environment, start a docker container by running the following. (example showned here with nvidia-docker support). By running this, the current directory that you're in (should be `f1tenth_gym_ros`) is mounted in the container at `/sim_ws/src/f1tenth_gym_ros`. Which means that the changes you make in the repo on the host system will also reflect in the container.
```bash
$ rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

## Without an NVIDIA gpu:

**Install the following dependencies:**

If your system does not support nvidia-docker2, noVNC will have to be used to forward the display.
- Again you'll need **Docker**. Follow the instruction from above.
- Additionally you'll need **docker-compose**. Follow the instruction [here](https://docs.docker.com/compose/install/) to install docker-compose.

**Installing the simulation:**

1. Clone this repo 
2. Bringup the novnc container and the sim container with docker-compose:
```bash
docker-compose up
``` 
3. In a separate terminal, run the following, and you'll have the a bash session in the simulation container. `tmux` is available for convenience.
```bash
docker exec -it f1tenth_gym_ros-sim-1 /bin/bash
```
4. In your browser, navigate to [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html), you should see the noVNC logo with the connect button. Click the connect button to connect to the session.

# Launching the Simulation

1. `tmux` is included in the contianer, so you can create multiple bash sessions in the same terminal.
2. To launch the simulation, make sure you source both the ROS2 setup script and the local workspace setup script. Run the following in the bash session from the container:
```bash
$ source /opt/ros/foxy/setup.bash
$ source install/local_setup.bash
$ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
A rviz window should pop up showing the simulation either on your host system or in the browser window depending on the display forwarding you chose.

You can then run another node by creating another bash session in `tmux`.

# Configuring the simulation
- The configuration file for the simulation is at `f1tenth_gym_ros/config/sim.yaml`.
- Topic names and namespaces can be configured but is recommended to leave uncahnged.
- The map can be changed via the `map_path` parameter. You'll have to use the full path to the map file in the container. The map follows the ROS convention. It is assumed that the image file and the `yaml` file for the map are in the same directory with the same name. See the note below about mounting a volume to see where to put your map file.
- The `num_agent` parameter can be changed to either 1 or 2 for single or two agent racing.
- The ego and opponent starting pose can also be changed via parameters, these are in the global map coordinate frame.

The entire directory of the repo is mounted to a workspace `/sim_ws/src` as a package. All changes made in the repo on the host system will also reflect in the container. After changing the configuration, run `colcon build` again in the container workspace to make sure the changes are reflected.

# Topics published by the simulation

In **single** agent:

`/scan`: The ego agent's laser scan

`/ego_racecar/odom`: The ego agent's odometry

`/map`: The map of the environment

A `tf` tree is also maintained.

In **two** agents:

In addition to the topics available in the single agent scenario, these topics are also available:

`/opp_scan`: The opponent agent's laser scan

`/ego_racecar/opp_odom`: The opponent agent's odometry for the ego agent's planner

`/opp_racecar/odom`: The opponent agents' odometry

`/opp_racecar/opp_odom`: The ego agent's odometry for the opponent agent's planner

# Topics subscribed by the simulation

In **single** agent:

`/drive`: The ego agent's drive command via `AckermannDriveStamped` messages

`/initalpose`: This is the topic for resetting the ego's pose via RViz's 2D Pose Estimate tool. Do **NOT** publish directly to this topic unless you know what you're doing.

TODO: kb teleop topics

In **two** agents:

In addition to all topics in the single agent scenario, these topics are also available:

`/opp_drive`: The opponent agent's drive command via `AckermannDriveStamped` messages. Note that you'll need to publish to **both** the ego's drive topic and the opponent's drive topic for the cars to move when using 2 agents.

`/goal_pose`: This is the topic for resetting the opponent agent's pose via RViz's 2D Goal Pose tool. Do **NOT** publish directly to this topic unless you know what you're doing.

# Keyboard Teleop

The keyboard teleop node from `teleop_twist_keyboard` is also installed as part of the simulation's dependency. To enable keyboard teleop, set `kb_teleop` to `True` in `sim.yaml`. After launching the simulation, in another terminal, run:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Then, press `i` to move forward, `u` and `o` to move forward and turn, `,` to move backwards, `m` and `.` to move backwards and turn, and `k` to stop in the terminal window running the teleop node.

# Developing and creating your own agent in ROS 2

There are multiple ways to launch your own agent to control the vehicles.

- The first one is creating a new package for your agent in the `/sim_ws` workspace inside the sim container. After launch the simulation, launch the agent node in another bash session while the sim is running.
- The second one is to create a new ROS 2 container for you agent node. Then create your own package and nodes inside. Launch the sim container and the agent container both. With default networking configurations for `docker`, the behavior is to put The two containers on the same network, and they should be able to discover and talk to each other on different topics. If you're using noVNC, create a new service in `docker-compose.yml` for your agent node. You'll also have to put your container on the same network as the sim and novnc containers.
