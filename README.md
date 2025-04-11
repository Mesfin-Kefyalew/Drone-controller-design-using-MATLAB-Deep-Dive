# Drone-controller-design-using-MATLAB

This repository is based on [MATLAB Simulation of a Quadrotor UAV Dynamics and Control](https://www.youtube.com/watch?v=7F9cG64kRxI). In this version, I have only placed the original project files. Note, however, that a detailed explanation will be issued ASAP. Furthermore, the problems stated in the original challenge will also be solved in this repository. So, stay tuned!

# Code breakdown
Drone.m

Is basically a CLASS defining a DRONE OBJECT. In it are defined two kinds of properties: ones associated with drone dynamics such as the gravity constant, length, mass, Inertia, state variables[X, Y, Z, dx, dy, dz, phi, theta, psi, p, q, r], and input variables[T_sum, M1, M2, M3]. The other type of variables it contains have to do with the PID control.

Next to these properties as in any conventional class definition, there are sets of FUNCTIONS under METHODS:

Constructor method: 
- Drone() -> which initializes the properties already defined based on the call arguments and default values.
- GetState() -> enables data encapsulation
- EvalEOM() -> contains equations of motion for dx or the time derivative of the state variables.
- UpdateState() -> whenever the simulation loop is run in the main file, this function adds time steps and updates the state variable via outputs from the attitude controller and EvalEOM functions.
- AttitudeCtrl() -> takes in desired values of [phi, theta, psi, z_dot] as reference signals and adjust the [T, M1, M2, M3] by running a code for PID control.

main.m
- initializes drone parametes(including periphery coordinates), initial states, with numerical values.
- defines gains for the PID controller
- defines desired values for [psi, phi, theta, zdot]
- runs a loop for the simulation of the drone within a 3d figure.
- - 

