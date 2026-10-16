import numpy as np
from Dataclasses import State, MissileSpecs, TargetSpecs, SimConfig
from KinematicBodies.Missile import Missile
from KinematicBodies.Target import Target
from KinematicBodies.GuidanceFunctions import *
from ThrustLayouts import step_thrust_func

mach = 1234.8 # speed of sound in 343

def create_crossing_evasion_scenario():
    """Builds a standard 12km crossing intercept with a 9-G weaving target."""
    
    missile_specs = MissileSpecs() # Uses defaults (AIM-9 profile)
    target_specs = TargetSpecs(max_g_accel=90.0, weave_period=6.0)
    sim_config = SimConfig(max_time=100.0)

    missile = Missile(
        truth_state=State(pos=np.array([0.0, 5000.0]), vel=np.array([300.0, 0.0])),
        guidance_func=proportional_navigation,
        specs=missile_specs,
        thrust_func=step_thrust_func
    )

    target = Target(
        truth_state=State(pos=np.array([12000.0, 6000.0]), vel=np.array([-250.0, -20.0])),
        guidance_func=target_evasion_maneuver,
        specs=target_specs
    )

    return missile, target, sim_config

def create_head_on_scenario():
    """Builds a high-closure-rate head-on engagement."""
    # Custom specs for a heavier, longer-burning BVR missile like an AIM-120 AMRAAM
    amraam_specs = MissileSpecs(
        initial_mass=150.0, 
        dry_mass=100.0, 
        burn_time=8.0, 
        seeker_lock_range=15000.0
    )
    
    missile = Missile(
        truth_state=State(pos=np.array([0.0, 5000.0]), vel=np.array([400.0, 0.0])),
        guidance_func=proportional_navigation,
        specs=amraam_specs,
        thrust_func=step_thrust_func
    )

    target = Target(
        truth_state=State(pos=np.array([25000.0, 5000.0]), vel=np.array([-300.0, 0.0])),
        guidance_func=None, # Target flies straight
        specs=None
    )
    
    return missile, target, SimConfig(max_time=60.0)

def create_iron_dome_scenario(radar_delay_seconds=10.0):
    """
    Simulates a surface-to-air intercept.
    A Tamir interceptor launches vertically to destroy an incoming ballistic rocket.
    """
    # Tamir Interceptor Specs
    # High thrust, fast burn, extremely high G-limit for terminal maneuvers
    tamir_specs = MissileSpecs(
        initial_mass=90.0,        # kg
        dry_mass=60.0,            # kg
        burn_time=3.0,            # seconds (burns very fast)
        thrust_magnitude=28000.0, # Newtons (High boost to fight gravity)
        structural_g_limit=400.0, # 40 Gs
        seeker_lock_range=5000.0  # Active radar activates at 5km
    )

    # Launcher is at X=0, Y=0. 
    launch_angle_rad = np.radians(70)
    initial_speed = 30.0
    vx = initial_speed * np.cos(launch_angle_rad)
    vy = initial_speed * np.sin(launch_angle_rad)

    tamir_interceptor = Missile(
        truth_state=State(
            pos=np.array([0.0, 0.0]), 
            vel=np.array([vx, vy]) 
        ),
        guidance_func=proportional_navigation,
        specs=tamir_specs,
        thrust_func=step_thrust_func
    )


    grad_launch_x = 15000.0
    grad_launch_y = 0.0
    grad_vx = -350.0
    grad_vy = 400.0

    # Fast-forward the Grad's physics by the radar delay time
    # X = x0 + v*t
    # Y = y0 + v*t + 0.5*a*t^2
    # Vy = v0 + a*t 
    delayed_x = grad_launch_x + (grad_vx * radar_delay_seconds)
    delayed_y = grad_launch_y + (grad_vy * radar_delay_seconds) + (0.5 * -9.81 * radar_delay_seconds**2)
    delayed_vy = grad_vy + (-9.81 * radar_delay_seconds)

    grad_rocket = Target(
        truth_state=State(
            pos=np.array([delayed_x, delayed_y]), 
            vel=np.array([grad_vx, delayed_vy])
        ),
        guidance_func=unguided_ballistic_arc,
        specs=None
    )

    #  Environment Config
    sim_config = SimConfig(
        max_time=60.0, 
        ground_altitude=0.0 # Ground hits at Y=0
    )

    return tamir_interceptor, grad_rocket, sim_config

def create_davids_sling_scenario():
    """
    Simulates a long-range David's Sling intercept of a cruise missile.
    The Stunner interceptor uses massive thrust to climb into the thin upper 
    atmosphere, accelerating to high Mach numbers before diving onto the low-altitude target.
    """
    # 1. Stunner Interceptor Specs 
    stunner_specs = MissileSpecs(
        initial_mass=400.0,        # Much heavier than Tamir
        dry_mass=150.0,            
        burn_time=12.0,            # Simulating a long-burning multi-pulse motor
        thrust_magnitude=75000.0,  # Massive thrust to reach Mach 5+ 
        structural_g_limit=500.0,  # 50 Gs for hit-to-kill terminal maneuvering
        seeker_lock_range=20000.0  # Advanced dual-seeker activates at 20km
    )

    launch_angle_rad = np.radians(80)
    initial_speed = 30.0
    vx = initial_speed * np.cos(launch_angle_rad)
    vy = initial_speed * np.sin(launch_angle_rad)

    stunner_interceptor = Missile(
        truth_state=State(
            pos=np.array([0.0, 0.0]), 
            vel=np.array([vx, vy]) 
        ),
        guidance_func=pitch_over_then_pronav,  
        specs=stunner_specs,
        thrust_func=step_thrust_func
    )


    # 3. Cruise Missile Specs
    cruise_missile = Target(
        truth_state=State(
            pos=np.array([40000.0, 100]), 
            vel=np.array([-1*mach, 0.0]) 
        ),
        guidance_func=cruise_missile_level_flight
    )

    sim_config = SimConfig(max_time=120.0, ground_altitude=0.0)


    return stunner_interceptor, cruise_missile, sim_config