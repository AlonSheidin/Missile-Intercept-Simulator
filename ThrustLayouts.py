import numpy as np


def step_thrust_func(t: float, velocity: np.ndarray, burn_time: float, thrust_magnitude: float) -> np.ndarray:
    """
    Simulates a solid rocket motor producing a constant axial thrust force until burnout.
    
    Args:
        t: Current simulation time.
        velocity: The missile's current velocity vector.
        burn_time: Time at which the motor depletes its fuel.
        thrust_magnitude: Force in Newtons.
        
    Returns:
        np.ndarray: The thrust force vector in the direction of travel.
    """
    if t < burn_time:
        speed = np.linalg.norm(velocity)
        if speed > 0:
            heading = velocity / speed
            return heading * thrust_magnitude
        return np.array([1.0, 0.0]) # Fallback heading if stationary
        
    return np.array([0.0, 0.0])