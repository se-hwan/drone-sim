# Description
Drone Simulation [WORK IN PROGRESS]

Simulation and control of 2D drone in Python. Practice with animation and implementing simple control strategies.

Simple first order Euler method used for simulation.

Broad to do and ideas to explore:

2D:
## drone control
- visualization
- initialize with initial conditions subject to gravity
- control orientation and position to 0 and desired (feedback linearization? control options should be explored)
- explore trajectory optimization?

## swarm control
- 2D topdown "hockey puck" model of drone
- force inputs alone principal axes
- give "leader" drone path, control drone "followers" to be always be at some des. posn near it
- implement orientation dynamics
- explore lidar mapping and localization? unfamiliar, worth learning about
- create "environments"/2d mazes for drones to explore if above successful


3D:
- clarify EoM: https://towardsdatascience.com/demystifying-drone-dynamics-ee98b1ba882f
- visualization
- initialize with x0 subject to gravity
- control orientation and position to desired
