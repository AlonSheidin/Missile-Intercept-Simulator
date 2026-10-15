import numpy as np
from Dataclasses import State


class KalmanFilter:
    """
    A 2D discrete-time Extended/Linear Kalman Filter for state estimation.
    Fuses noisy sensor measurements with a predictive kinematic physics model.
    """
    def __init__(self, initial_state: State, q_variance: float, r_variance: float, initial_uncertainty=500.0) -> None:
        # Initialize state as a 4x1 matrix: [x, y, vx, vy]^T
        self.X = np.array([initial_state.pos, initial_state.vel], dtype=float).reshape(4, 1)
        
        # Covariance Matrix (P): Represents filter uncertainty
        self.P = np.eye(4) * initial_uncertainty

        # Measurement matrix (H): Maps the state space to the measurement space (we only measure x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float) 

        self.I = np.eye(4)
        self.q_variance = q_variance     
        self.R = np.eye(2) * r_variance  # Sensor noise covariance

    def predict(self, dt: float, commanded_acceleration=None):
        """
        Phase 1: Predicts the new state based on system dynamics.
        
        Args:
            dt: Time step since the last prediction.
            commanded_acceleration: The known [ax, ay] applied to the system (Control Input).
                                    If None, defaults to a constant velocity model.
        """
        # Dynamic State Transition Matrix (F)
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0,  dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ], dtype=float)

        # Process Noise Covariance Matrix (Q)
        Q_discrete = np.array([
            [0.25 * dt**4, 0,            0.5 * dt**3, 0],
            [0,            0.25 * dt**4, 0,           0.5 * dt**3],
            [0.5 * dt**3,  0,            dt**2,       0],
            [0,            0.5 * dt**3,  0,           dt**2]
        ], dtype=float) * self.q_variance

        # State Extrapolation
        if commanded_acceleration is not None:
            # Control Matrix (B) translates acceleration into position/velocity changes
            B = np.array([
                [0.5 * dt**2, 0],
                [0,           0.5 * dt**2],
                [dt,          0],
                [0,           dt]
            ], dtype=float)
            u = np.array(commanded_acceleration, dtype=float).reshape(2, 1)
            self.X = np.dot(F, self.X) + np.dot(B, u)
        else:
            self.X = np.dot(F, self.X)

        # Covariance Extrapolation
        self.P = np.dot(np.dot(F, self.P), F.T) + Q_discrete

    def update(self, measurement: np.ndarray):
        """
        Phase 2: Corrects the prediction using the noisy sensor measurement.
        
        Args:
            measurement: A 1D array [x, y] from the sensor.
        """
        Z = np.array(measurement, dtype=float).reshape(2, 1)

        # 1. Innovation (Difference between measurement and prediction)
        y = Z - np.dot(self.H, self.X)

        # 2. Innovation Covariance
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R

        # 3. Kalman Gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # 4. State Update
        self.X = self.X + np.dot(K, y)

        # 5. Covariance Update
        self.P = np.dot((self.I - np.dot(K, self.H)), self.P)

    @property
    def current_estimate(self) -> np.ndarray:
        """Returns a flat 1D array [x, y, vx, vy] for easy state generation."""
        return self.X.flatten()