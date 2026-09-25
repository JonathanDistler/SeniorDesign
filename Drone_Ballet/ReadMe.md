# The Goal of This Project

The goal of this project is to increase the length of survival for a drone under high-energy laser confrontation.

The drone is readily able to detect if it has been hit, as well as the face it has been hit via previous work on diode configurations. The basic premise is that the drone recognizes one of its faces has been hit, then it will spin about an orthogonal axis to the face, so that the laser's energy is dissipated across multiple faces (and components), as well as through the driven convection due to rotational effects.

# Basic Assumptions

The following are the main assumptions for the work:

1. The laser will be a "lower-power" (2 kW–20 kW) laser at a 500–3000 m engagement distance.
2. Lasers have instantaneous engagement capability, meaning they only require a line of sight of the drone ("hit scan" dynamics).
3. Lasers require some amount of dwell time to cause damage to the drone, rendering it useless.
4. Dwell time is a function of laser power, laser optics, engagement distance, atmospherics (including turbulence models), and material composition of the target (in this case, the drone).

# Basic Research Question

The basic research question is as follows:

> **"Does the rotation of a drone enable it to mitigate the effects of a high-energy laser?"**

With **"mitigate"** defined as **"the increase of dwell time needed to render the drone useless and/or nullify the laser effects completely."**

# Methodology

The general methodology is to create a representation of the drone with realistic material composition (mainly using a quadcopter base), then simulate via ANSYS the effect of the laser's irradiance pattern with varying degrees of stochasticity. Then, iterating over rotation rates, determine system degradation and failure modes with tabulated material properties. 

# Failure Modes
1) **Full Penetration**: Point at which inner layer of the hull >= melting point of the material
2) **Mechanical Deformation of Aerodynamic Supporting Structure**: Significant change in angle of the support arm - with exact number yet to be determined
3) **Destruction of Motor**: Temperature of the motor above some threshold - e.g. Aluminum melting point or >= specification sheet operating temperature
4) **Thermal Runaway**: 
* 1 Casing/housing of the drone battery reaches a critical temperature such that the battery failes
* 2 Overheating of the electronics enclosure, such that the minimum temperature threshold of any component is surpassed
5) **Internal Overheating of Thermal Load Bearing**: Minimum temperature threshold of any component is surpassed in the hull of the drone

# ANSYS Setup
Firstly, define two modules: Transient Thermal and Static Structural (or Transient Structural), with the Engineering Data, Geometry, and Solution from Transient Thermal serving as inputs into the Static Structural module.

Then, define the specific material properties for the system of interest (e.g. Carbon-Fiber), making sure to add thermal properties for orthotropic thermal conductivity and specific heat.

Assign the relevant materials to each body of the drone. Then, for use with our particular APDL script, define two named-selections: LASER_PATH is defined as the faces on which the laser will be traveling; DRONE_EXTERNAL is defined as all surface faces, such that convection correlations can be used. Then, mesh the drone, I found the default settings worked well, but future work could look into finer meshes along discontinuities (especially in inorganic drone models). 

Then, in the Transient Thermal module, underneath "Analysis Settings" define the Step End Time (in our case I used 30s), make sure that Auto Time Stepping is toggled to "Off", and that Time Integration is toggled to "On". I found empirically that a time-step of .01 s worked very well, but for higher fidelity models, one could use a <.01 s time-step. 

Right click the "Transient Thermal" module in ANSYS Mechanical, and add a command. Then, copy the APDL script provided in the repo. and place it in the provided dropdown. Then, solve the Transient Thermal response. 

Then, add a constraint to the Static/Transient Structural structural module, I added a fixed support along the bottom of the drone, but fixed supports about the motors could also be relevant. Then, match the step-end time to that of the Transient Thermal, and use 1 step (for "Number of Steps"). Then, solve the module