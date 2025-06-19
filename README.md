# Safe Navigation in Dynamic Environments using Density Functions
This work presents a density-based framework for safe navigation in dynamic environments characterized by time-varying obstacle sets and time-varying target regions. We propose an analytical construction of time-varying density functions that enables the synthesis of a feedback controller defined as the positive gradient of the resulting density field. The primary contribution of this paper is a rigorous convergence proof demonstrating almost-everywhere safe navigation under the proposed framework, specifically for systems governed by single-integrator dynamics. To the best of our knowledge, these are the first analytical guarantees of their kind for navigation in dynamic environments using density functions. We illustrate the applicability of the framework to systems with more complex dynamics, including multi-agent systems and robotic manipulators, using standard control design techniques such as backstepping and inverse dynamics. These results provide a foundation for extending density-based navigation methods to a broad class of robotic systems operating in time-varying environments.

```
@article{narayanan2024safe,
  title={Safe Navigation in Dynamic Environments using Density Functions},
  author={Narayanan, Sriram SKS and Moyalan, Joseph and Vaidya, Umesh},
  journal={arXiv preprint arXiv:2411.12206},
  year={2024}
}
```

# Density functions
Density functions are a physically intuitive way to solve almost everywhere (a.e.) safe navigation problems.
<p align="center">
  <img src="figures_paper/density_figure.png" alt="fig" width="600" />
</p>
Inverse bump function: (a) top view showing contours and (b) 3D view.

We exploit the occupancy-based interpretation of density in constructing analytical expressions for time-varying density functions.
<p align="center">
  <img src="figures_paper/occ_map.png" alt="fig" width="600" />
</p>
(a) Density function defined on an environment with a circular unsafe set and a point target, (b) Corresponding occupancy measure obtained using trajectories from 100 initial conditions sampled within the initial set.

# Multiagent Systems
A four-agent scenario where two agents are bigger than the other agents
<p align="center">
  <img src="animations/4_agent_scenario_2.gif" alt="animated" />
</p>

A six-agent scenario where all the agents are the same size
<p align="center">
  <img src="animations/6_agent_scenario.gif" alt="animated" />
</p>


# Robotic Arm
Safe trajectory tracking of a robotic arm while avoinding obstalces
<p align="center">
  <img src="animations/planarRR.gif" alt="animated" width="400"/>
</p>

# Lane tracking
In this example, the density-based controller is able to account for time-varying obstacles (gray) and track a time-varying target (green). Even though the target passes through the unsafe sets, the density-based controller always finds a safe trajectory 

Scenario 1:
<div align="center">
  <img src="animations/lane_tracking1.gif" width="300" alt="Lane tracking 1" />
</div>

Scenario 2:
<div align="center">
  <img src="animations/lane_tracking1.gif" width="300" alt="Lane tracking 2" />
</div>

## Quadruped Navigation
In this scenario, the quadruped robot has to navigate to the goal while avoiding the time-varying obstacles. The safe trajectory obtained from the density-based controller is  used as a motion plan along with a nonlinear MPC to to ensure safe navgiation of the robot
<div align="center">
  <img src="animations/density_time_quadruped.gif" width="300" alt="Quadruped" />
</div>
