# gym-ros_bridge

ROS 2 communication bridge that turns the ICELab F1TENTH gym environment into a ROS 2 simulation.

This repository is the ICELab fork of [f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros).


## What is different from upstream

| | Upstream | This fork |
|---|---|---|
| ROS 2 distro | Foxy (Ubuntu 20.04, Python 3.8) | **Jazzy** (Ubuntu 24.04, Python 3.12) |
| Simulator | public `f1tenth/f1tenth_gym` | private [`ice-lab-racing/gym-environment`](https://github.com/ice-lab-racing/gym-environment), branch `develop` (Gymnasium API, needs Python ≥ 3.11) |
| Driving stack | – | private [`ice-lab-racing/car_stack`](https://github.com/ice-lab-racing/car_stack), built in the same workspace |
| Path in the container | `/sim_ws/src/f1tenth_gym_ros` | `/sim_ws/src/gym-ros_bridge` |
| Image / container name | `f1tenth_gym_ros` | `gym-ros_bridge` |
| Episode end | ignored | automatic reset (see [Simulation behaviour](#simulation-behaviour)) |

The ROS package name is unchanged: `f1tenth_gym_ros`.

## How the container is put together

The simulation uses three repositories. Inside the `sim` container they live here:

| Repository | Path in the container | What it is |
|---|---|---|
| `gym-ros_bridge` (this repo) | `/sim_ws/src/gym-ros_bridge` | ROS package `f1tenth_gym_ros`: the bridge node, launch file, maps, `config/sim.yaml` |
| `gym-environment` | `/gym-environment` | the simulator, a Python package installed with `pip install -e` (not a ROS package) |
| `car_stack` | `/sim_ws/src/car_stack` | ROS packages that drive the car (`pure_pursuit`, `gap_follow`) |

Each repository reaches the container in one of two ways:

- **Cloned** into the image when it is built. The image then works on its own, but it holds a snapshot: new commits only arrive with an image rebuild.
- **Mounted** from a checkout on your machine when the container starts. The container sees your files live: edit on the host, rebuild or restart inside the container, no image rebuild.

| Repository | Cloned into the image | Mounted |
|---|---|---|
| `gym-ros_bridge` | copied at build time | always, by `docker-compose.yml` |
| `gym-environment` | yes, branch `develop` | optional, by `docker-compose.override.yml` |
| `car_stack` | yes, default branch | optional, by `docker-compose.override.yml` |

A mount hides the cloned copy at the same path, so when a repository is mounted the clone in the image is simply not used.

### Option A: cloned only

No extra setup: build the image and start it. Use this to run the simulator without working on the gym or the driving stack.

To pick up new commits of `gym-environment` or `car_stack`, rebuild the image without cache (the clone step is cached by Docker):

```bash
docker compose build --no-cache sim
```

### Option B: mounted local checkouts (recommended for development)

Check out the three repositories next to each other:

```
ICE_RoboRacer/
├── gym-ros_bridge/
├── gym-environment/
└── car_stack/
```

Create `gym-ros_bridge/docker-compose.override.yml`. Docker Compose merges it automatically into `docker-compose.yml`, so all commands stay the same:

```yaml
services:
  sim:
    volumes:
      # local checkouts replace the copies cloned into the image at build time
      - ../gym-environment:/gym-environment
      - ../car_stack:/sim_ws/src/car_stack
    environment:
      # the container runs as root: keep its cache files out of the mounted sources
      - NUMBA_CACHE_DIR=/tmp/numba_cache
      - PYTHONDONTWRITEBYTECODE=1
```

Keep it out of git, since the paths depend on your machine:

```bash
echo docker-compose.override.yml >> .git/info/exclude
```

What to do after a change:

| You changed | Then |
|---|---|
| code in `gym-environment` | restart the launch (no build needed: the editable install points at the mounted folder) |
| code in `car_stack` or in this repo, `config/sim.yaml`, launch files, maps | `colcon build` in `/sim_ws`, then relaunch |
| a dependency (`pyproject.toml` of the gym, a `package.xml`) | rebuild the image |
| `docker-compose.yml` or the override file | `docker compose up -d` (recreates the container, then run `colcon build` again) |

Things to know:

- If a mounted folder does not exist on your machine, Docker creates an empty one and mounts it over the cloned copy, so the packages disappear. Only list repositories you have checked out.
- `build/` and `install/` of the workspace live inside the container. When the container is recreated they are reset to the ones built into the image, so run `colcon build` once before the first launch.
- The image still clones both repositories at build time, so building needs the access token even when you only use the mounts.

## Prerequisites

- Docker with Docker Compose v2 (`docker compose`, not the legacy `docker-compose`).
- A GitHub access token that can read `ice-lab-racing/gym-environment` and `ice-lab-racing/car_stack` (both repositories are private, so the image build cannot clone them anonymously).

### Access token

1. On GitHub, create a fine-grained personal access token with:
   - **Resource owner:** `ice-lab-racing` (not your personal account)
   - **Repository access:** Only select repositories → `gym-environment` and `car_stack`
   - **Repository permissions:** Contents → Read-only
2. Copy `.env.example` to `.env` in the root of this repository and put the token in it:
   ```
   GH_TOKEN=github_pat_...
   ```

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

| Build error | Meaning |
|---|---|
| `failed to stat .../.env` / `secret dotenv: not found` | `.env` is missing, or `--secret` was not passed to `docker build` |
| `GH_TOKEN: is not set in .env` | `.env` has no `GH_TOKEN=` line |
| `Invalid username or token` | the token is wrong, expired or was regenerated |
| `Write access to repository not granted` (403) / `Repository not found` | the token is valid but cannot read that repository: check resource owner and repository access |

## 2. Start the containers

```bash
docker compose up -d
```

This starts two containers:

- `sim`: the bridge image, idle with a `bash` shell. The repositories are cloned or mounted as described above.
- `novnc`: a virtual desktop served in the browser, used to display RViz. The `sim` container has no screen of its own: `DISPLAY=novnc:0.0` sends its windows here. Open [http://localhost:8080/vnc.html](http://localhost:8080/vnc.html) and click **Connect**.

Stop everything with `docker compose down`.

## 3. Open a terminal in the container

```bash
docker compose exec sim bash
```

Run it again from another host terminal for every extra shell you need; `tmux` is also installed.
The shell starts in `/sim_ws`.

## 4. Source ROS 2 Jazzy and build the workspace

The image contains a workspace built at image build time. Build again after a change (see the table above) and after the container is recreated:

```bash
source /opt/ros/jazzy/setup.bash
cd /sim_ws
colcon build
```

`colcon build` must run from `/sim_ws`, not from inside `src/`. It builds every package in `src/`: `f1tenth_gym_ros`, `pure_pursuit` and `gap_follow`.

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

## 6. Run the driving stack

With the simulation running, in a second shell (after sourcing as above). Once the workspace is sourced, ROS finds packages by name, so the current folder does not matter:

```bash
ros2 launch pure_pursuit pure_pursuit_launch.py track_dir:=/sim_ws/src/car_stack/tracks/levine
ros2 launch gap_follow gap_follow_launch.py
```

`pure_pursuit` reads `/ego_racecar/odom` and follows the track centerline; `gap_follow` reads `/scan`. Both publish on `/drive`.
Run one at a time, since they publish on the same topic.

Keyboard teleop (with `kb_teleop: True` in `config/sim.yaml`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Simulation behaviour

- When an episode terminates (ego collision, ego rollover, or all cars completing two laps) the simulation resets automatically: every car goes back to its starting pose from `config/sim.yaml` and stands still until the next drive command. A node that keeps publishing drive commands starts moving again immediately.
- An opponent collision does not end the episode.

## Configuration notes

- The configuration file is `config/sim.yaml`. `map_path` must be a path inside the container, without extension, e.g. `/sim_ws/src/gym-ros_bridge/maps/levine`.
- `map_img_ext` must start with a dot (`.png`).
- `scan_fov` and `scan_beams` are passed to the simulator, so the published `LaserScan` matches the simulated one.
- With two agents (`num_agent: 2`) the simulation only steps once both `/drive` and `/opp_drive` have received a command.
