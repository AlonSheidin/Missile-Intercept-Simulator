import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


def animate_flight(flight_data: dict) -> FuncAnimation:
    """
    Generates a 2D animation of the missile and target trajectories.
    Returns the FuncAnimation object to prevent garbage collection.
    """
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Calculate bounding box for the plot
    all_x = np.concatenate([flight_data["m_truth_x"], flight_data["t_truth_x"]])
    all_y = np.concatenate([flight_data["m_truth_y"], flight_data["t_truth_y"]])
    
    x_span = np.max(all_x) - np.min(all_x)
    y_span = np.max(all_y) - np.min(all_y)
    
    # Dynamic 5% padding (with a 1000m minimum so flat trajectories don't break the Y-axis)
    x_pad = max(x_span * 0.05, 1000.0)
    y_pad = max(y_span * 0.05, 1000.0)
    
    ax.set_xlim(np.min(all_x) - x_pad, np.max(all_x) + x_pad)
    
    # Ensure the ground (Y=0) is always visible
    min_y = min(0.0, np.min(all_y)) 
    ax.set_ylim(min_y - (y_pad * 0.5), np.max(all_y) + y_pad)
    
    ax.grid(True)
    ax.set_title("Kinematic Intercept Simulation")

    # Initialize line objects
    missile_trail_truth, = ax.plot([], [], "r-", label="Missile Path - True")
    target_trail_truth, = ax.plot([], [], "b-", label="Target Path - True")
    missile_trail_perceived, = ax.plot([], [], "r--", label="Missile Path - Perceived")
    target_trail_perceived, = ax.plot([], [], "b--", label="Target Path - Perceived")
    missile_head, = ax.plot([], [], "ro")
    target_head, = ax.plot([], [], "bo")
    ax.legend(loc="upper left")

    def update(frame_index):
        missile_trail_truth.set_data(flight_data["m_truth_x"][:frame_index], flight_data["m_truth_y"][:frame_index])
        target_trail_truth.set_data(flight_data["t_truth_x"][:frame_index], flight_data["t_truth_y"][:frame_index])
        missile_trail_perceived.set_data(flight_data["m_perc_x"][:frame_index], flight_data["m_perc_y"][:frame_index])
        target_trail_perceived.set_data(flight_data["t_perc_x"][:frame_index], flight_data["t_perc_y"][:frame_index])
        
        missile_head.set_data([flight_data["m_truth_x"][frame_index]], [flight_data["m_truth_y"][frame_index]])
        target_head.set_data([flight_data["t_truth_x"][frame_index]], [flight_data["t_truth_y"][frame_index]])

        return (
            missile_trail_truth, target_trail_truth,
            missile_trail_perceived, target_trail_perceived,
            missile_head, target_head,
        )

    # Calculate an optimal frame step based on dt resolution to keep playback smooth
    total_frames = len(flight_data["time"])
    frame_step = max(1, total_frames // 400) 
    frames_to_render = range(0, total_frames, frame_step)
    
    animation = FuncAnimation(fig, update, frames=frames_to_render, interval=10, blit=True)
    return animation


def plot_telemetry(flight_data: dict) -> None:
    """
    Generates a single window containing vertically stacked subplots for Velocity and Mass.
    """
    fig, (ax_vel, ax_mass) = plt.subplots(nrows=2, ncols=1, figsize=(10, 10))

    # --- Velocity Subplot ---
    ax_vel.plot(flight_data["time"], flight_data["m_truth_vx"], "r-", label="Missile Vx")
    ax_vel.plot(flight_data["time"], flight_data["m_truth_vy"], "r--", label="Missile Vy")
    ax_vel.plot(flight_data["time"], flight_data["t_truth_vx"], "b-", label="Target Vx")
    ax_vel.plot(flight_data["time"], flight_data["t_truth_vy"], "b--", label="Target Vy")
    ax_vel.plot(flight_data["time"], flight_data["m_truth_speed"], "r:", label="Missile Speed")
    ax_vel.plot(flight_data["time"], flight_data["t_truth_speed"], "b:", label="Target Speed")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("Velocity (m/s)")
    ax_vel.set_title("Velocity Components Over Time")
    ax_vel.grid(True)
    ax_vel.legend(loc="upper right")

    # --- Mass Subplot ---
    ax_mass.plot(flight_data["time"], flight_data["m_truth_mass"], "k-", label="Missile Mass")
    ax_mass.set_xlabel("Time (s)")
    ax_mass.set_ylabel("Mass (kg)")
    ax_mass.set_title("Missile Mass Over Time (Propellant Burn)")
    ax_mass.grid(True)
    ax_mass.legend(loc="upper right")

    fig.tight_layout()