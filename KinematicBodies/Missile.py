import numpy as np
from typing import Optional

from Dataclasses import Derivative, MissileSpecs, State
from KalmanFilter import KalmanFilter
from KinematicBodies.KinematicBody import KinematicBody


class Missile(KinematicBody):
    """
    Simulates an autonomous interceptor. Contains the guidance logic ("Brain"), 
    sensor models, state estimation (Kalman Filter), and aerodynamic models.
    """
    def __init__(self, truth_state: State, guidance_func,
                 specs: MissileSpecs,
                 thrust_func=None):
        super().__init__(truth_state, guidance_func)
        
        # Physical Properties
        self.thrust_func = thrust_func
        self.specs = specs
        self.initial_mass = self.specs.initial_mass
        self.dry_mass = self.specs.dry_mass
        self.burn_time = self.specs.burn_time
        self.thrust_magnitude = self.specs.thrust_magnitude


        # State Variables
        self.last_time = 0.0
        self.last_acceleration = np.array([0.0, 0.0])

        # Self-Estimation Kalman Filter
        self.perceived_self = State(pos=self.truth_state.pos.copy(), vel=self.truth_state.vel.copy())
        self.self_klm_filter = KalmanFilter(
            initial_state=self.perceived_self,
            q_variance=9.0,
            r_variance=100.0
        )

        # Target-Estimation Kalman Filter (Initialized on first detection)
        self.perceived_target = None
        self.target_klm_filter: Optional[KalmanFilter] = None

    def get_derivatives(self, t: float, state: State, **kwargs) -> Derivative:
        """
        Executes the missile's control loop: updates filters, calculates guidance, 
        and applies dynamic G-limits.
        """
        target_state = kwargs.get('target_state')
        if target_state is None:
            raise ValueError("Missile requires 'target_state' to calculate guidance!")

        dt = t - self.last_time
        if dt > 0:
            # 1. Predict Self State (The Brain expects thrust, drag, and gravity)
            thrust_accel = self.get_thrust_acceleration(t, state.vel) if self.thrust_func else np.zeros(2)
            
            # FIX: Use pos[1] for altitude to accurately predict atmospheric drag
            drag_accel = self.get_drag_acceleration(t, self.perceived_self.pos[1], self.perceived_self.vel)
            gravity_accel = np.array([0, -9.81])

            prediction_acceleration = self.last_acceleration + thrust_accel + drag_accel + gravity_accel
            self.self_klm_filter.predict(dt, commanded_acceleration=prediction_acceleration)

        # 2. Acquire Sensor Data
        gps_measurement = self.sense_self(t, state)
        sensed_target = self.sense_target(target_state)

        # 3. Target Filter Initialization & Update
        if self.target_klm_filter is None:
            self.target_klm_filter = KalmanFilter(
                initial_state=sensed_target,
                q_variance=25.0,
                r_variance=2500.0
            )

        pos_dif = self.perceived_self.pos - sensed_target.pos
        distance = np.linalg.norm(pos_dif)

        if distance <= self.specs.seeker_lock_range:
            # Terminal Phase: Active seeker achieves perfect lock.
            self.perceived_target = target_state
        else:
            if self.target_klm_filter is not None:
                if dt > 0:
                    self.target_klm_filter.predict(dt)
                self.target_klm_filter.update(sensed_target.pos)

                target_estimate = self.target_klm_filter.current_estimate
                self.perceived_target = State(pos=target_estimate[0:2], vel=target_estimate[2:4])
            
            self.self_klm_filter.update(gps_measurement.pos)

        # Extract cleaned self-state
        clean_array = self.self_klm_filter.current_estimate
        self.perceived_self = State(pos=clean_array[0:2], vel=clean_array[2:4])

        # 4. Calculate Guidance Commands
        ax, ay = self.guidance_func(self.perceived_self, self.perceived_target)
        accel_magnitude = np.sqrt(ax**2 + ay**2)
        
        # 5. Apply Dynamic Physical Limits
        max_accel = self.get_max_acceleration(t, self.perceived_self.pos[1], self.perceived_self.vel)
        if accel_magnitude > max_accel:
            ax = ax * (max_accel / accel_magnitude)
            ay = ay * (max_accel / accel_magnitude)

        self.last_acceleration = np.array([ax, ay])
        self.last_time = t

        return Derivative(vel=state.vel, accel=self.last_acceleration)

    def sense_target(self, target_truth: State) -> State:
        """Simulates noisy radar tracking of the target."""
        noisy_pos = target_truth.pos + np.random.normal(0, 50.0, size=2)
        noisy_vel = target_truth.vel + np.random.normal(0, 5.0, size=2)
        return State(pos=noisy_pos, vel=noisy_vel)

    def sense_self(self, t, self_state: State) -> State:
        """Simulates noisy onboard GPS/IMU data."""
        noisy_pos = self_state.pos + np.random.normal(0, 10.0, size=2)
        return State(pos=noisy_pos, vel=self_state.vel)

    def get_drag_acceleration(self, t: float, altitude: float, velocity: np.ndarray) -> np.ndarray:
        """Calculates expected deceleration due to aerodynamic drag."""
        area = self.specs.ref_area  # m^2 (Sidewinder cross-section)
        cd = self.specs.drag_cd 
        
        clamped_alt = max(0.0, altitude) 
        rho = 1.225 * np.exp(-clamped_alt / 8420.0)
        
        speed = np.linalg.norm(velocity)
        if speed > 0:
            drag_force_x = -0.5 * rho * area * cd * speed * velocity[0]
            drag_force_y = -0.5 * rho * area * cd * speed * velocity[1]
            return np.array([drag_force_x, drag_force_y]) / self.get_mass(t)
            
        return np.array([0.0, 0.0])

    def get_thrust_acceleration(self, t: float, velocity: np.ndarray) -> np.ndarray:
        """Returns the axial thrust acceleration vector based on current dynamic mass."""
        current_mass = self.get_mass(t)
        if self.thrust_func:
            force = self.thrust_func(
                t=t,
                velocity=velocity,
                burn_time=self.burn_time,
                thrust_magnitude=self.thrust_magnitude
            )
        else:
            force = np.zeros(2)

        return force / current_mass

    def get_mass(self, t: float) -> float:
        """Calculates dynamic mass depletion over the burn time."""
        if t < self.burn_time:
            mass_flow_rate = (self.initial_mass - self.dry_mass) / self.burn_time
            return self.initial_mass - (mass_flow_rate * t)
        return self.dry_mass

    def get_max_acceleration(self, t: float, altitude: float, velocity: np.ndarray) -> float:
        """
        Determines the physical G-limit by taking the lesser of the structural
        limit (airframe destruction) and aerodynamic limit (fin stall).
        """
        structural_limit = self.specs.structural_g_limit
        
        current_mass = self.get_mass(t)
        clamped_alt = max(0.0, altitude) 
        rho = 1.225 * np.exp(-clamped_alt / 8420.0)
        speed_sq = velocity[0]**2 + velocity[1]**2
        
        q = 0.5 * rho * speed_sq
        lift_area_product = self.specs.lift_area_product
        max_lift_force = q * lift_area_product
        
        aero_limit = max_lift_force / current_mass
        return min(structural_limit, aero_limit)
