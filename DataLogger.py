import numpy as np


class DataLogger:
    """
    Handles high-performance telemetry logging during the simulation loop.
    Stores frame-by-frame data for post-flight Matplotlib analysis.
    """
    def __init__(self):
        # Store lists for fast appending during the simulation loop
        self.history = {
            "time": [],
            "m_truth_mass": [],
            "m_truth_x": [], "m_truth_y": [],
            "t_truth_x": [], "t_truth_y": [],
            "m_truth_vx": [], "m_truth_vy": [],
            "t_truth_vx": [], "t_truth_vy": [],
            "m_truth_speed": [], "t_truth_speed": [],
            "m_perc_x": [], "m_perc_y": [],
            "t_perc_x": [], "t_perc_y": []
        }

    def log_frame(self, t, m_truth, t_truth, m_perc, t_perc, m_mass):
        """Records a single simulation frame."""
        self.history["time"].append(t)
        self.history["m_truth_mass"].append(m_mass)
        
        self.history["m_truth_x"].append(m_truth.pos[0])
        self.history["m_truth_y"].append(m_truth.pos[1])
        self.history["t_truth_x"].append(t_truth.pos[0])
        self.history["t_truth_y"].append(t_truth.pos[1])
        self.history["m_truth_vx"].append(m_truth.vel[0])
        self.history["m_truth_vy"].append(m_truth.vel[1])
        self.history["t_truth_vx"].append(t_truth.vel[0])
        self.history["t_truth_vy"].append(t_truth.vel[1])
        self.history["m_truth_speed"].append(np.linalg.norm(m_truth.vel))
        self.history["t_truth_speed"].append(np.linalg.norm(t_truth.vel))

        # If the missile hasn't perceived anything yet, default to truth to avoid plotting crashes
        self.history["m_perc_x"].append(m_perc.pos[0] if m_perc else m_truth.pos[0])
        self.history["m_perc_y"].append(m_perc.pos[1] if m_perc else m_truth.pos[1])
        self.history["t_perc_x"].append(t_perc.pos[0] if t_perc else t_truth.pos[0])
        self.history["t_perc_y"].append(t_perc.pos[1] if t_perc else t_truth.pos[1])

    def get_arrays(self) -> dict:
        """Converts history lists to numpy arrays for efficient Matplotlib plotting."""
        return {key: np.array(value) for key, value in self.history.items()}