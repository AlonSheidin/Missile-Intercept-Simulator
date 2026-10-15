import numpy as np

from DataLogger import DataLogger
from Dataclasses import Derivative, SimConfig, State
from KinematicBodies.Missile import Missile
from KinematicBodies.Target import Target

X, Y = 0, 1

class SimulationEnvironment:
    """
    Manages the overarching continuous physics loop, numerical integration, and collision detection.
    Strictly separates the discrete control logic from continuous kinematics.
    """
    def __init__(self, missile: Missile, target: Target, logger: DataLogger, sim_config: SimConfig) -> None:
        self.missile = missile
        self.target = target
        self.logger = logger
        self.sim_config = sim_config
        self.previous_distance = 2**63 # max int

    def _equation_of_motion(self, t, interceptor_state: State, target_state: State):
        """Triggers the control loop (Brain) of entities to fetch their commanded accelerations."""
        interceptor_derivative = self.missile.get_derivatives(t, interceptor_state, target_state=target_state)
        target_derivative = self.target.get_derivatives(t, target_state)
        return interceptor_derivative, target_derivative

    def run(self):
        """Executes the simulation loop until collision or max_time is reached."""
        max_time = self.sim_config.max_time
        dt = self.sim_config.dt
        time = 0.0
    
        while time < max_time:
            self.logger.log_frame(
                t=time,
                m_truth=self.missile.truth_state,
                t_truth=self.target.truth_state,
                m_perc=self.missile.perceived_self,
                t_perc=self.missile.perceived_target,
                m_mass=self.missile.get_mass(time),
            )

            # 1. Control Update (1x per frame)
            inter_der, target_der = self._equation_of_motion(time, self.missile.truth_state, self.target.truth_state)

            # 2. Physics Update (RK4 Integration, 4x micro-steps per frame)
            self.missile.truth_state = rk4_step(
                current_state=self.missile.truth_state,
                commanded_accel=inter_der.accel,
                t=time,
                dt=dt,
                thrust_func=self.missile.get_thrust_acceleration,
                drag_func=self.missile.get_drag_acceleration
            )
            self.target.truth_state = rk4_step(
                current_state=self.target.truth_state,
                commanded_accel=target_der.accel,
                t=time,
                dt=dt
            )

            if self.check_collision():
                break

            time += dt
            
    def check_collision(self) -> bool:
        """Determines if the missile has impacted the target or the ground."""
        inter_pos = self.missile.truth_state.pos
        target_pos = self.target.truth_state.pos

        distance = np.linalg.norm(inter_pos - target_pos)

        if distance > self.previous_distance:
            if self.previous_distance <= self.sim_config.collision_radius:
                print(f"\n--- INTERCEPT SUCCESSFUL ---")
                print(f"Kinetic Impact! Miss Distance: {self.previous_distance:.2f} meters\n")
                return True
            else:
                # The missile missed completely
                print(f"\n--- INTERCEPT FAILED ---")
                print(f"Missed target by: {self.previous_distance:.2f} meters\n")
                return True

        self.previous_distance = distance
        # Check Ground Collision
        if target_pos[Y] < self.sim_config.ground_altitude:
            return True
        return False


def rk4_step(current_state: State, commanded_accel: np.ndarray, t: float, dt: float, thrust_func=None, drag_func=None) -> State:
    """
    Executes a 4th-order Runge-Kutta numerical integration step.
    Averages 4 discrete micro-steps over the frame window to safely integrate 
    continuous, non-linear environmental forces.
    """
    def get_physics_deriv(state: State, current_t) -> Derivative:
        thrust_acceleration = thrust_func(current_t, state.vel) if thrust_func else np.zeros(2)
        drag_acceleration = drag_func(current_t, state.pos[1], state.vel) if drag_func else np.zeros(2)
        gravity_acceleration = np.array([0, -9.81])

        sum_acceleration = commanded_accel + thrust_acceleration + drag_acceleration + gravity_acceleration
        return Derivative(vel=state.vel, accel=sum_acceleration)

    # k1 (Evaluate at start of frame)
    d1 = get_physics_deriv(current_state, t)
    
    # k2 (Evaluate at mid-point)
    s2 = State(
        pos=current_state.pos + d1.vel * (dt / 2.0),
        vel=current_state.vel + d1.accel * (dt / 2.0)
    )
    d2 = get_physics_deriv(s2, t + (dt / 2.0))
    
    # k3 (Evaluate at mid-point using k2 momentum)
    s3 = State(
        pos=current_state.pos + d2.vel * (dt / 2.0),
        vel=current_state.vel + d2.accel * (dt / 2.0)
    )
    d3 = get_physics_deriv(s3, t + (dt / 2.0))
    
    # k4 (Evaluate at end of frame using k3 momentum)
    s4 = State(
        pos=current_state.pos + d3.vel * dt,
        vel=current_state.vel + d3.accel * dt
    )
    d4 = get_physics_deriv(s4, t + dt)
    
    # Combine using the RK4 weighted average formula
    new_pos = current_state.pos + (dt / 6.0) * (d1.vel + 2*d2.vel + 2*d3.vel + d4.vel)
    new_vel = current_state.vel + (dt / 6.0) * (d1.accel + 2*d2.accel + 2*d3.accel + d4.accel)
    
    return State(pos=new_pos, vel=new_vel)