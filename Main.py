import matplotlib.pyplot as plt

from DataLogger import DataLogger
from SimulationEnvironment import SimulationEnvironment
import Scenarios
import Visualizer

# ==========================================
# SIMULATION CONFIGURATION CONTROLS
# ==========================================
SHOW_ANIMATION = True
SHOW_TELEMETRY = True
# ==========================================

def main():
    print("Initializing scenario...")
    missile, target, sim_config = Scenarios.create_davids_sling_scenario()
    logger = DataLogger()
    environment = SimulationEnvironment(missile, target, logger, sim_config)

    print(f"Running RK4 simulation (dt={sim_config.dt}s)...")
    environment.run()
    flight_data = environment.logger.get_arrays()

    # Store animation reference in a list to prevent Python's garbage collector from deleting it
    active_animations = []

    if SHOW_TELEMETRY:
        Visualizer.plot_telemetry(flight_data)
        
    if SHOW_ANIMATION:
        anim = Visualizer.animate_flight(flight_data)
        active_animations.append(anim)

    if SHOW_ANIMATION or SHOW_TELEMETRY:
        plt.show()

if __name__ == "__main__":
    main()