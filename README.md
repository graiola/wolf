# WoLF: Whole-body Locomotion Framework for quadruped robots

## Branches

- ROS1 Noetic: `ros1-noetic-pub`
- ROS2 Humble: `ros2-humble-pub`

## ROS2 Setup (Humble)

1. Create a ROS2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone WoLF ROS2 branch:

```bash
git clone -b ros2-humble-pub https://github.com/graiola/wolf.git
cd wolf
git submodule update --init --recursive
```

3. Install dependencies:

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. Build in `Release`:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

5. Launch the controller:

```bash
ros2 launch wolf_controller wolf_controller_bringup.launch.xml robot_model:=spot robot_name:=ras_1
```

## ROS1 Setup (Noetic)

For ROS1, use branch `ros1-noetic-pub` and follow the ROS1/catkin instructions.


## Changelog

See changelog [here](https://github.com/graiola/wolf-setup/blob/master/CHANGELOG.md).
