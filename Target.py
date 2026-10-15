import numpy as np

from Dataclasses import Derivative, State, TargetSpecs
from KinematicBodies.KinematicBody import KinematicBody


class Target(KinematicBody):
    def __init__(self, truth_state: State, guidance_func, specs=None):
        super().__init__(truth_state, guidance_func)
        self.specs = specs
        
    def get_derivatives(self, t, state, **kwargs):
        # Target only cares about its own state
        vx, vy = state.vel
        if self.guidance_func == None:
            ax, ay = 0.0, 0.0 
        elif self.specs == None:
            ax, ay = self.guidance_func(t, state)
        else:
            ax, ay = self.guidance_func(t, state, specs=self.specs)

        
        return Derivative(vel=np.array([vx, vy]), accel=np.array([ax, ay]))