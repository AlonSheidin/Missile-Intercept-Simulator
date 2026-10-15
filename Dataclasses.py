from dataclasses import dataclass
import numpy as np


@dataclass
class State:
    """
    Represents the physical state of a kinematic body at a specific point in time.
    """
    pos: np.ndarray  # [x, y] in meters
    vel: np.ndarray  # [vx, vy] in meters/second


@dataclass
class Derivative:
    """
    Represents the rate of change for a kinematic body, used for numerical integration.
    """
    vel: np.ndarray    # [vx, vy] in meters/second
    accel: np.ndarray  # [ax, ay] in meters/second^2


@dataclass
class MissileSpecs:
    """
    Configuration parameters for missile propulsion, aerodynamics,
    and seeker performance.

    (Defaults configuration is of the Aim-9)
    """
    initial_mass: float = 85.0       # kg
    dry_mass: float = 55.0           # kg
    burn_time: float = 5.0           # seconds
    thrust_magnitude: float = 17000.0 # Newtons
    ref_area: float = 0.0127         # m^2
    drag_cd: float = 0.4             # Drag coefficient
    structural_g_limit: float = 350.0 # m/s^2
    lift_area_product: float = 0.105 # S_ref * C_L_max
    seeker_lock_range: float = 2500.0 # meters

@dataclass
class TargetSpecs:
    max_g_accel: float = 90.0        # m/s^2
    weave_period: float = 6.0        # seconds for a full sine wave

@dataclass
class SimConfig:
    dt: float = 0.01                 # physics step size
    max_time: float = 100.0          # timeout
    collision_radius: float = 30.0   # meters
    ground_altitude: float = -1.0    # meters
