# The Goal of This Project

The goal of this project is to create a viable drone benchmark tests that incorporates both static and dynamic elements, as well as viability for singular drones and drone swarms. 

Although there are drone benchmark tests, as per my literature review, they don't incorporate any reinforcement-learning (RL) based testing, dynamic elements, or compatibility with drone swarms - a growing research feature. 

![Gazebo Simulation](images\model.png)

# Basic Assumptions

The following are the main assumptions for the work:

1. The drones in use are (in theory) capable of the desired movement patterns. 

2. All in-vitro drone tests will be compared to a 1:1 Gazebo simulation, with the same mechanisms in play. 

# Basic Research Question

The basic research question is as follows:

> **"Can I build a physical drone benchmark test that is backed with a simulation environment that can test drone computation?"**

# Methodology

The general methodology is to create a representation of the drone test-bed in Gazebo, then transfer the exact setup into a physical environmnet. Then, using basic circuitry and computer vision, the physical tests will be overlapped with the simulation tests to guide drone development. 

# Setup
The setup is as follows. In the first terminal, after having cloned PX4-Autopilot, configured a pathway to the *drone_obstacle_course.sdf*, imported *gazebo_box.STL* and *gazebo_flag.STL*:

*Terminal 1*
```bash
cd ~/PX4-Autopilot

export PX4_GZ_WORLD=drone_obstacle_course

export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH
```
With the absolute-pathway being dependent on the folder structure. 

Then,

```bash
make px4_sitl gz_x500_vision
```
It is very important to use the gz_x500_vision model as it has onboard vision which allows the simulation to run without a magnetometer, which is crucial. 

Finally, set the parameters with the following: 

```bash
param set EKF2_MAG_TYPE 5

param set EKF2_EV_CTRL 15

param set COM_ARM_MAG_STR 0 

param set COM_ARM_MAG_ANG -1 

param set NAV_DLL_ACT 0 

param set COM_DLL_EXCEPT 4 

param set
```
*Terminal 2*
This terminal runs the scripts for each practical test that corresponds to the real world test. The general folder layout is described below:

![drone_obstacle_course folder tree](images\drone_obstacle_course_tree.png)


Firstly, create a virtual-environment (in this case .venv). After setting up the previous terminal and getting prompted to "takeoff":

```bash
cd ~/drone_obstacle_course

source .venv/bin/activate

python drone_tests.py 1
```
The last line *python drone_tests.py 1* uses an integer as the last argument, to carry out test 2 or 3, replace the 1 with the respective test. 


