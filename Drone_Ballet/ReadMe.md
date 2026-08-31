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

The general methodology is to create a representation of the drone with realistic material composition (mainly using a quadcopter base), then simulate via MATLAB (and its respective PDE solver) the effects of the laser's irradiance pattern with some degree of stochasticity.

Then, iterating over rotation rates, determine the energy irradiated on each component of the drone and cross-check that with material degradation properties.