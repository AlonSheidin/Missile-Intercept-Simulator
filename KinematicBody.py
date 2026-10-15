from abc import ABC, abstractmethod
from Dataclasses import Derivative, State


class KinematicBody(ABC):
    """
    Abstract base class for all physical entities in the simulation environment.
    """
    def __init__(self, truth_state: State, guidance_func):
        self.truth_state = truth_state
        self.guidance_func = guidance_func

    @abstractmethod
    def get_derivatives(self, t: float, state: State, **kwargs) -> Derivative:
        """
        Calculates and returns the Derivative (velocity and acceleration) for numerical integration.
        Reactive objects can extract external states from kwargs.
        """
        pass