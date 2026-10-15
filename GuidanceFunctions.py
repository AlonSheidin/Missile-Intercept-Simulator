import numpy as np
from Dataclasses import State, TargetSpecs

X, Y = 0, 1

def proportional_navigation(missile_state: State, target_state: State, N=3) -> tuple[float, float]:
    """
    Calculates the lateral acceleration required to intercept a target using Proportional Navigation.
    
    Args:
        missile_state: The perceived state of the interceptor.
        target_state: The perceived state of the target.
        N: Navigation constant (typically 3-5).
        
    Returns:
        tuple: Commanded (ax, ay) acceleration.
    """
    pos_rel = target_state.pos - missile_state.pos
    vel_rel = target_state.vel - missile_state.vel

    closing_vector = np.linalg.norm(vel_rel)

    # Line-of-Sight (LOS) angular rate of change
    los_angle_rate_of_change = (pos_rel[X] * vel_rel[Y] - pos_rel[Y] * vel_rel[X]) / (pos_rel[X]**2 + pos_rel[Y]**2)
    los_angle = np.arctan2(pos_rel[Y], pos_rel[X])

    # Commanded lateral acceleration magnitude
    a_c = N * los_angle_rate_of_change * closing_vector

    # Distribute perpendicularly to the LOS vector
    ax = -a_c * np.sin(los_angle)
    ay = a_c * np.cos(los_angle)
    
    return ax, ay

def pitch_over_then_pronav(missile_state: State, target_state: State) -> tuple[float, float]:
    """
    Multi-phase guidance for vertical launchers.
    Phase 1: Max-G pitch-over to a 25-degree loft.
    Phase 2: Standard Proportional Navigation for terminal intercept.
    """
    vx, vy = missile_state.vel
    speed = np.linalg.norm(missile_state.vel)
    
    if speed > 0:
        # Calculate current flight path angle in degrees
        flight_path_angle = np.degrees(np.arctan2(vy, vx))
    else:
        flight_path_angle = 90.0

    # Phase 1: Aggressive Pitch-Over
    if flight_path_angle > 25.0:
        # Determine if target is to the right (+1) or left (-1)
        direction = 1.0 if target_state.pos[0] > missile_state.pos[0] else -1.0
        
        # Calculate the vector perpendicular to velocity (pushing down toward the horizon)
        perp_dir = np.array([vy * direction, -vx * direction]) / speed
        
        # Command an absurdly high G-pull (the Missile physics will dynamically clamp this 
        # to the true aerodynamic limit based on current airspeed)
        pull_accel = 500.0 
        return float(perp_dir[0] * pull_accel), float(perp_dir[1] * pull_accel)

    # Phase 2: Terminal Proportional Navigation
    # We increase the navigation constant (N=4) for a more aggressive final dive
    return proportional_navigation(missile_state, target_state, N=4)

def target_evasion_maneuver(t: float, self_state: State, **kwargs) -> tuple[float, float]:
    """
    Simulates a fighter jet executing a continuous 9-G sinusoidal weave to bleed missile energy.
    """
    specs = kwargs.get("specs")
    if specs is not TargetSpecs:
        return 0.0, 0.0

    max_g_accel = specs.max_g_accel  
    weave_frequency = (2.0 * np.pi) / specs.weave_period  
    
    speed = np.linalg.norm(self_state.vel)
    if speed == 0:
        return 0.0, 9.81 # Generate lift even if stalled (fallback)
        
    perp_dir = np.array([-self_state.vel[1], self_state.vel[0]]) / speed
    current_g = max_g_accel * np.sin(weave_frequency * t)
    
    ax = perp_dir[0] * current_g
    
    # Maneuvering acceleration + Wing Lift to fight gravity
    ay = (perp_dir[1] * current_g) + 9.81 
    
    return float(ax), float(ay)

def target_u_turn_maneuver(t: float, self_state: State, **kwargs) -> tuple[float, float]:
    """
    Simulates a rapid 9-G U-turn. Target flies straight, executes a 180-degree turn, and runs away.
    """
    max_g_accel = 90.0  
    turn_start_time = 2.0
    
    speed = np.linalg.norm(self_state.vel)
    if speed == 0:
        return 0.0, 0.0
        
    turn_rate = max_g_accel / speed 
    turn_duration = np.pi / turn_rate
    turn_end_time = turn_start_time + turn_duration
    
    if turn_start_time <= t <= turn_end_time:
        perp_dir = np.array([-self_state.vel[1], self_state.vel[0]]) / speed
        ax = perp_dir[0] * max_g_accel
        ay = perp_dir[1] * max_g_accel
        return float(ax), float(ay)
        
    return 0.0, 0.0

def unguided_ballistic_arc(t: float, self_state: State, **kwargs) -> tuple[float, float]:
    """
    Simulates an unguided artillery rocket. 
    Commands zero aerodynamic acceleration, allowing environmental gravity to form the arc.
    """
    return 0.0, 0.0

def cruise_missile_level_flight(t: float, self_state: State, **kwargs) -> tuple[float, float]:
    """
    Simulates a terrain-skimming cruise missile.
    Generates constant vertical lift to counteract gravity and maintain level flight.
    """
    # 0.0 horizontal acceleration (constant speed)
    # 9.81 vertical acceleration (lift equaling gravity)
    return 0.0, 9.81