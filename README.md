![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![Status](https://img.shields.io/badge/Status-Active-success.svg)

This repository contains a continuous-time physics simulation of surface-to-air and air-to-air missile engagements.

The engine uses a 4th-order Runge-Kutta numerical integrator for flight dynamics, Proportional Navigation for terminal guidance, and a Kalman Filter for state estimation. The simulation accurately accounts for physical and aerodynamic variables such as fin stall and dynamic mass depletion.

### David's Sling vs Cruise Missile
<img width="500" height="300" alt="Image" src="https://github.com/user-attachments/assets/081832f2-09c2-4a11-907b-0937422b5a5b" />

### Iron Dome vs Ballistic Rocket
<img width="500" height="300" alt="Image" src="https://github.com/user-attachments/assets/d2108fda-0aa5-40b2-8f00-d4e4e07adade" />

## Physics and Flight Dynamics
The simulation calculates continuous flight dynamics rather than relying on basic constant-acceleration models.
* **Dynamic Mass Depletion:** Thrust applies a constant force, which increases the acceleration rate as the solid rocket motor burns propellant and reduces the vehicle's mass.
* **Aerodynamic Drag:** Atmospheric density and drag deceleration are calculated based on the current altitude and velocity.
* **Dynamic G-Limits:** The interceptor is constrained by structural limits and dynamic pressure. At low speeds or high altitudes, the fins stall, simulating real-world minimum engagement range.
* **Discrete Tunneling Prevention:** The engine uses Point of Closest Approach (PCA) logic to detect sub-meter kinetic impacts at hypersonic closing velocities, preventing collision detection failures between discrete physics frames.

## Mathematical Models
The simulation is driven by standard aerospace flight mechanics equations:
* **Proportional Navigation Law:** Generates acceleration commands perpendicular to the line-of-sight based on the closing velocity and Line of Sight (LOS) rate:
  $$a_c = N V_c \dot{\lambda}$$
* **Aerodynamic Drag:** Deceleration is dynamically calculated using air density, velocity, reference area, and a drag coefficient:
  $$F_D = \frac{1}{2} \rho v^2 C_D A$$
* **Numerical Integration:** State updates (position, velocity) are computed using a 4th-Order Runge-Kutta (RK4) method for high stability over large time steps.
* **Kalman Filter State Estimation:** A 2D discrete-time filter that fuses noisy sensor measurements with a predictive kinematic physics model. The implementation accounts for state transitions ($F$) and optional commanded acceleration control inputs ($u$) mapped through a control matrix ($B$). The optimal state vector ($X$) is recursively computed in two phases:
  * **Predict:** Extrapolates the state and system uncertainty forward in time:
    $$\hat{X}_{k}^- = F\hat{X}_{k-1} + Bu_k$$
  * **Update:** Corrects the prior estimate using new 2D positional sensor measurements ($Z_k$), the measurement matrix ($H$), and the calculated Kalman Gain ($K$):
    $$\hat{X}_k = \hat{X}_{k}^- + K(Z_k - H\hat{X}_{k}^-)$$
    
## Guidance and Control
The system architecture cleanly separates the continuous physical environment from the discrete sensor and control logic.
* **Kalman Filter:** Processes simulated noisy GPS/radar measurements to accurately estimate the state of the interceptor and target.
* **Proportional Navigation:** Calculates line-of-sight angular rates to command terminal intercept trajectories.
* **Multi-Phase Guidance:** Vertical launch profiles execute a programmed pitch-over maneuver to clear the lower atmosphere before activating the active Pro-Nav seeker.
  
## Installation and Usage 
```bash
git clone https://github.com/AlonSheidin/Missile-Intercept-Simulator.git
cd Missile-Intercept-Simulator
pip install -r requirements.txt
```

## Running the Simulation
Engagement geometries are defined in Scenarios.py. Run the main entry point to execute the simulation and launch the Matplotlib visualization:
```Bash
python Main.py
```
## Configured Engagement Scenarios

The Scenarios.py file uses dataclasses for dependency injection to test different physical geometries.

* Vertical Launch (David's Sling profile): A heavy, multi-pulse interceptor fires vertically at 80 degrees, executing a pitch-over to loft over the lower atmosphere before diving onto a cruise missile.
* Surface-to-Air (Iron Dome profile): A short-range interceptor fires from a slanted canister to intercept an unguided rocket following a ballistic arc.
* Air-to-Air: A Mach 0.9 launch against a target executing a continuous 9-G sinusoidal evasive weave.
