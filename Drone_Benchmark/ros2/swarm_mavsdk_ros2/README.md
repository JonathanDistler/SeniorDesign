# Scratch 3-Drone PX4 / Gazebo / ROS 2 / MAVSDK

This intentionally replaces the earlier direct `px4_msgs` flight controller.

Architecture:

    Gazebo default world
            |
        3 x gz_x500
            |
        PX4 SITL
            |
        MAVLink UDP
            |
        MAVSDK-Python
            |
        ROS 2 node
        /          \
    publishers   subscribers

PX4 multi-instance behavior:

    instance 0 -> system 1 -> ROS /
    instance 1 -> system 2 -> ROS /px4_1
    instance 2 -> system 3 -> ROS /px4_2

The standard x500 model is used instead of `gz_x500_vision`. This removes the external-vision estimator dependency.

Tests:

    0 = direct velocity sanity test
    1 = 3 m East translation + XY/YZ MSE
    2 = random positions -> orthogonal straight line
    3 = flag turn -> split -> 1 m translation

Results:

    ~/swarm_mavsdk_results/swarm_test_results.csv

ROS 2 topics:

    /uav_0/ned_position
    /uav_0/ned_velocity
    /uav_0/altitude
    /uav_0/battery
    /uav_0/status
    /uav_0/cmd_velocity_ned

and similarly for uav_1 and uav_2.

The `/swarm/command` subscriber is also provided for future high-level swarm commands.
