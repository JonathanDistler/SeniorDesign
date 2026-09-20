# ROS 2 PX4 Drone Swarm Controller

This is an `ament_python` ROS 2 package wrapping `drone_test_swarms.py`.
It is designed for the PX4/uXRCE-DDS pipeline with three vehicles under:

- `/uav_0/fmu/...`
- `/uav_1/fmu/...`
- `/uav_2/fmu/...`

## Install into the workspace

Copy this directory into `~/px4_ros2_ws/src/`:

```bash
cp -r drone_swarm_controller ~/px4_ros2_ws/src/
```

Then build:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/px4_ros2_ws
colcon build --symlink-install --packages-select drone_swarm_controller
source install/setup.bash
```

## Run

Start the Micro XRCE-DDS Agent first:

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros_uxrce_dds_ws/install/local_setup.bash
MicroXRCEAgent udp4 -p 8888
```

In another terminal, start the three-drone PX4/Gazebo swarm:

```bash
cd ~/drone_obstacle_course
./run_drone_swarm.bash 3
```

In a third terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run drone_swarm_controller drone_test_swarms --n 3
```

Optional random seed:

```bash
ros2 run drone_swarm_controller drone_test_swarms --n 3 --seed 1234
```

The script creates `swarm_test_results.csv` in the directory from which it is launched.

## Tests

`1` = 3 m simultaneous East translation + XY/YZ mean-squared pairwise distance.

`2` = random positions, user ENTER signal, then an orthogonal straight-line formation.

`3` = randomly selected LEFT/RIGHT flag, 2-vs-1 split, then 1 m simultaneous translation.
