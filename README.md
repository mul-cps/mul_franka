# Franka ROS 2 Setup at MUL

## Installation

### Set Up Control PC

1. install the latest [Ubuntu LTS](https://ubuntu.com/download/desktop)
2. either set up Ubuntu Pro real-time kernel as suggested by the [Franka documentation](https://support.franka.de/docs/installation_linux.html#setting-up-the-real-time-kernel) or install the [XanMod Kernel](https://xanmod.org) with realtime (`PREEMPT_RT`) patches (`sudo apt install linux-xanmod-rt-x64v3` or `linux-xanmod-rt-x64v2` for older systems)
3. set [realtime permissions](https://support.franka.de/docs/installation_linux.html#installation-real-time) for user
4. (optional for CUDA) add the [NVIDIA driver and CUDA repo](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu) (select `deb (network)`) and install the driver package with `IGNORE_PREEMPT_RT_PRESENCE=1 sudo -E apt install cuda-drivers` (you also have to use `IGNORE_PREEMPT_RT_PRESENCE=1` when updating the driver, e.g. `IGNORE_PREEMPT_RT_PRESENCE=1 sudo -E apt upgrade`, or simply add `export IGNORE_PREEMPT_RT_PRESENCE=1` to your `~/.bashrc`)

### Compile ROS 2 Workspace

Run the following commands to build the Franka workspace:
```sh
mkdir ~/franka_ws && cd ~/franka_ws
vcs import --recursive --input https://raw.githubusercontent.com/mul-cps/mul_franka/refs/heads/main/sources.repos
rosdep install --from-paths src --ignore-src -y --skip-keys=libfranka
colcon build
. install/setup.bash
```

## Usage

### Start Sequence

1. Power on the controller. The lights on the robot will flash yellow. Wait until the lights are solid yellow.
2. Connect to the Franka Desk Web UI (`https://$ROBOT_IP`).
3. Unlock the breaks. If the light turn pink, press and release the e-stop. The lights will turn blue.
4. Activate FCI. This will lock the Web UI and you can now connect to the robot via [libfranka](https://github.com/frankaemika/libfranka) on ip `$ROBOT_IP` and port `1337`.

### Launch Files

This repo contains a couple of launch files:
1. [`franka.launch.py`](mul_franka_launch/launch/franka.launch.py): hardware interface and controllers
2. [`moveit.launch.py`](mul_franka_launch/launch/moveit.launch.py): MoveIt 2 for motion planning
3. [`rviz.launch.py`](mul_franka_launch/launch/rviz.launch.py): visualisation of the robot state and UI for MoveIt 2


Once FCI is activated on the robot, you can use these launch files via `ros2 launch`:
```sh
# start hardware interface, connect to FCI, and start controllers
ros2 launch mul_franka_launch franka.launch.py robot_ip:=$ROBOT_IP
# start MoveIt 2
ros2 launch mul_franka_launch moveit.launch.py
# optionally, start RViz
ros2 launch mul_franka_launch rviz.launch.py
```
