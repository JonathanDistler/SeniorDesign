# The Goal of This Project

The goal of this project is to create a viable drone benchmark tests that incorporates both static and dynamic elements, as well as viability for singular drones and drone swarms. 

Although there are drone benchmark tests, as per my literature review, they don't incorporate any reinforcement-learning (RL) based testing, dynamic elements, or compatibility with drone swarms - a growing research feature. 

# Basic Assumptions

The following are the main assumptions for the work:

1. The drones in use are (in theory) capable of the desired movement patterns. 

2. All in-vitro drone tests will be compared to a 1:1 Gazebo simulation, with the same mechanisms in play. 

# Basic Research Question

The basic research question is as follows:

> **"Can I build a physical drone benchmark test that is backed with a simulation environment that can test drone computation?"**

# Methodology

The general methodology is to create a representation of the drone test-bed in Gazebo, then transfer the exact setup into a physical environmnet. Then, using basic circuitry and computer vision, the physical tests will be overlapped with the simulation tests to guide drone development. 

