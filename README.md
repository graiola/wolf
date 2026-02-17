# WoLF: Whole-body Locomotion Framework for quadruped robots

## ROS1 Quickstart

These steps create a ROS1 workspace, clone WoLF with all submodules, build in `Release`, and launch the controller.

1. Create a catkin workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

2. Clone WoLF

```bash
git clone https://github.com/graiola/wolf.git
cd wolf
```

3. Initialize and update submodules (recursive)

```bash
git submodule update --init --recursive
```

4. Build with `catkin` in `Release`

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

5. Launch `wolf_controller`

```bash
roslaunch wolf_controller wolf_controller_bringup.launch robot_model:=spot robot_name:=ras_1
```

## Setup

see documentation [here](https://github.com/graiola/wolf-setup/blob/master/README.md)

## Changelog

see changelog [here](https://github.com/graiola/wolf-setup/blob/master/CHANGELOG.md)
