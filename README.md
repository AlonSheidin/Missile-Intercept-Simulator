# 3-DOF Kinematic Missile Intercept Simulator

This repository contains a continuous-time physics simulation of surface-to-air and air-to-air missile engagements.

The engine uses a 4th-order Runge-Kutta numerical integrator for flight dynamics, Proportional Navigation for terminal guidance, and an Kalman Filter for state estimation. The simulation accounts for aerodynamic variables such as fin stall, dynamic mass depletion.
### David's Sling vs Cruise Missile
<img width="500" height="300" alt="Image" src="https://github.com/user-attachments/assets/30306c66-54d2-45b1-8e8e-22860649fe69" />

### Iron Dome vs Ballistic Rocket
<img width="500" height="300" alt="Image" src="https://github.com/user-attachments/assets/d2108fda-0aa5-40b2-8f00-d4e4e07adade" />

## Physics and Flight Dynamics
The simulation calculates continuous flight dynamics rather than relying on constant-acceleration models.
* Dynamic mass depletion: Thrust applies a constant force, which increases the acceleration rate as the solid rocket motor burns propellant and reduces the vehicle's mass.
* Aerodynamic drag: Atmospheric density and drag deceleration are calculated based on the current altitude and velocity.
* Dynamic G-Limits: The interceptor is constrained by structural limits and dynamic pressure. At low speeds or high altitudes, the fins stall, simulating real-world minimum engagement ranges.
* Discrete tunneling prevention: The engine uses Point of Closest Approach (PCA) logic to detect sub-meter kinetic impacts at hypersonic closing velocities, preventing collision detection failures between physics frames.

## Guidance and Control
The system architecture separates the continuous physical environment from the discrete sensor and control logic.
* Kalman Filter (KF): Processes simulated GPS and radar measurements to estimate the state of the interceptor and target.
* Proportional Navigation: Calculates line-of-sight angular rates to command terminal intercept trajectories.
* Multi-phase guidance: Vertical launch profiles execute a programmed pitch-over maneuver to clear the lower atmosphere before activating the Pro-Nav seeker.

## Usage

**Installation**
```bash
git clone https://github.com/AlonSheidin/Missile-Intercept-Simulator.git
cd Missile-Intercept-Simulator
pip install -r requirements.txt
```

## Running the Simulation
Engagement geometries are defined in Scenarios.py. Run the main entry point to execute the simulation and launch the Matplotlib visualization:
```Bash
python main.py
```
## Configured Engagement Scenarios

The Scenarios.py file uses dataclasses for dependency injection to test different physical geometries.

* Vertical Launch (David's Sling profile): A heavy, multi-pulse interceptor fires vertically at 80 degrees, executing a TVC pitch-over to loft over the lower atmosphere before diving onto a cruise missile.
* Surface-to-Air (Iron Dome profile): A short-range interceptor fires from a slanted canister to intercept an unguided rocket following a ballistic arc.
* Air-to-Air: A Mach 0.9 launch against a target executing a continuous 9-G sinusoidal evasive weave.
