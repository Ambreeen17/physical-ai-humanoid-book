"""
Lab 4.1: 1D Kalman Filter - Starter Template

This is a template for the 1D Kalman Filter lab. Complete the TODOs
to implement a functional constant velocity Kalman filter.

Learning Objectives:
- Understand Kalman filter predict/update cycle
- Implement constant velocity motion model
- Apply filtering to 1D position estimation

Author: Robotics Lab Generator
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Optional


class KalmanFilter1D:
    """
    1D Kalman Filter for position and velocity estimation.

    State vector: x = [position, velocity]^T
    Motion model: Constant velocity
    Observation: Position only

    Attributes:
        Q: Process noise covariance
        R: Measurement noise covariance
        H: Observation matrix
    """

    def __init__(self, process_noise_std: float = 0.1,
                 measurement_noise_std: float = 1.0):
        """
        Initialize Kalman filter.

        Args:
            process_noise_std: Standard deviation of process noise (acceleration)
            measurement_noise_std: Standard deviation of measurement noise
        """
        # Process noise covariance Q
        # Q represents uncertainty in our motion model (acceleration noise)
        self.Q = np.array([[0, 0],
                          [0, process_noise_std**2]])

        # Measurement noise covariance R
        # R represents sensor noise (uncertainty in position measurements)
        self.R = measurement_noise_std**2

        # Observation matrix H: we measure position directly
        self.H = np.array([[1, 0]])

        print(f"Kalman Filter initialized with Q={self.Q.diagonal()}, R={self.R}")

    def predict(self, x_prev: np.ndarray, P_prev: np.ndarray,
                dt: float, u: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict step of Kalman filter.

        Computes the predicted state and covariance based on the motion model.

        Args:
            x_prev: Previous state estimate [position, velocity]^T
            P_prev: Previous state covariance matrix
            dt: Time step (seconds)
            u: Control input (not used in constant velocity model)

        Returns:
            Tuple of (predicted state, predicted covariance)
        """
        # ============================================================
        # TODO: Implement the predict step
        # ============================================================

        # Step 1: Compute the state transition matrix F for constant velocity
        # F = [[1, dt],
        #      [0,  1]]
        # F = np.array([[1, dt],
        #               [0, 1]])

        # YOUR CODE HERE - Uncomment and complete the above

        # Step 2: Compute control input matrix B (not needed for this model)
        # B = np.array([[0],
        #              [dt]])

        # Step 3: Compute predicted state
        # x_pred = F @ x_prev + B * u
        # x_pred = None  # YOUR CODE HERE

        # Step 4: Compute predicted covariance
        # P_pred = F @ P_prev @ F.T + Q
        # P_pred = None  # YOUR CODE HERE

        # Return predictions
        # return x_pred, P_pred

        pass  # Replace with actual return

    def update(self, x_pred: np.ndarray, P_pred: np.ndarray,
               z: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update step of Kalman filter.

        Updates the state estimate with a new measurement.

        Args:
            x_pred: Predicted state from predict step
            P_pred: Predicted covariance from predict step
            z: Measured position value

        Returns:
            Tuple of (updated state, updated covariance)
        """
        # ============================================================
        # TODO: Implement the update step
        # ============================================================

        # Step 1: Compute innovation (measurement residual)
        # y = z - H @ x_pred
        # y = None  # YOUR CODE HERE

        # Step 2: Compute innovation covariance
        # S = H @ P_pred @ H.T + R
        # S = None  # YOUR CODE HERE

        # Step 3: Compute Kalman gain
        # K = P_pred @ H.T @ inv(S)
        # K = None  # YOUR CODE HERE

        # Step 4: Update state estimate
        # x_hat = x_pred + K @ y
        # x_hat = None  # YOUR CODE HERE

        # Step 5: Update covariance estimate
        # P_hat = (I - K @ H) @ P_pred
        # P_hat = None  # YOUR CODE HERE

        # Return updated estimates
        # return x_hat, P_hat

        pass  # Replace with actual return


def generate_synthetic_data(n_samples: int = 200, dt: float = 0.1,
                            true_velocity: float = 2.0,
                            position_offset: float = 0.0,
                            measurement_noise_std: float = 1.0,
                            process_noise_std: float = 0.05) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate synthetic noisy position measurements.

    Args:
        n_samples: Number of time steps
        dt: Time step duration
        true_velocity: True constant velocity
        position_offset: Initial position
        measurement_noise_std: Std dev of measurement noise
        process_noise_std: Std dev of process noise (acceleration)

    Returns:
        times: Array of time values
        true_positions: True position at each time step
        measurements: Noisy position measurements
    """
    times = np.arange(n_samples) * dt

    # True positions (constant velocity motion)
    true_positions = position_offset + true_velocity * times

    # Add process noise (random acceleration)
    np.random.seed(42)  # For reproducibility
    acceleration_noise = np.random.normal(0, process_noise_std, n_samples)
    velocity_with_noise = true_velocity + np.cumsum(acceleration_noise * dt)
    true_positions = position_offset + np.cumsum(velocity_with_noise * dt)

    # Add measurement noise
    measurements = true_positions + np.random.normal(0, measurement_noise_std, n_samples)

    return times, true_positions, measurements


def run_kalman_filter_demo():
    """
    Demonstrate the Kalman filter with synthetic data.
    """
    # Filter parameters
    process_noise_std = 0.1   # Acceleration noise
    measurement_noise_std = 1.0  # Position measurement noise

    # Generate data
    times, true_positions, measurements = generate_synthetic_data(
        n_samples=200,
        dt=0.1,
        true_velocity=2.0,
        measurement_noise_std=measurement_noise_std,
        process_noise_std=process_noise_std
    )

    # Initialize filter
    kf = KalmanFilter1D(
        process_noise_std=process_noise_std,
        measurement_noise_std=measurement_noise_std
    )

    # Initialize state and covariance
    x = np.array([measurements[0], 0.0])  # Start with first measurement and zero velocity
    P = np.array([[measurement_noise_std**2, 0],
                  [0, 10]])  # High uncertainty in velocity

    # Storage for visualization
    estimates = []
    uncertainties = []

    # Run filter
    print("\nRunning Kalman filter...")
    for i, z in enumerate(measurements):
        # Predict
        x, P = kf.predict(x, P, dt=0.1)

        # Update
        x, P = kf.update(x, P, z)

        # Store results
        estimates.append(x.copy())
        uncertainties.append(np.sqrt(P[0, 0]))  # Position uncertainty (std dev)

    # Convert to arrays
    estimates = np.array(estimates)
    position_std = np.array(uncertainties)

    # Compute metrics
    position_errors = estimates[:, 0] - true_positions
    rmse = np.sqrt(np.mean(position_errors**2))

    print(f"\nResults:")
    print(f"  Final position estimate: {estimates[-1, 0]:.2f} m")
    print(f"  Final velocity estimate: {estimates[-1, 1]:.2f} m/s")
    print(f"  Position RMSE: {rmse:.3f} m")
    print(f"  Final uncertainty (1-sigma): {position_std[-1]:.3f} m")

    # ============================================================
    # Visualization
    # ============================================================
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    # Plot 1: Position estimates
    ax1 = axes[0]
    ax1.plot(times, true_positions, 'g-', linewidth=2, label='True Position')
    ax1.plot(times, measurements, 'b.', alpha=0.5, markersize=3, label='Measurements')
    ax1.plot(times, estimates[:, 0], 'r-', linewidth=2, label='Kalman Estimate')
    ax1.fill_between(times,
                     estimates[:, 0] - 2*position_std,
                     estimates[:, 0] + 2*position_std,
                     alpha=0.2, color='red', label='95% Confidence')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (m)')
    ax1.set_title('1D Kalman Filter: Position Estimation')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: Velocity estimates
    ax2 = axes[1]
    ax2.plot(times, np.full_like(times, 2.0), 'g--', linewidth=2, label='True Velocity')
    ax2.plot(times, estimates[:, 1], 'r-', linewidth=2, label='Estimated Velocity')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.set_title('1D Kalman Filter: Velocity Estimation')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('kalman_filter_results.png', dpi=150)
    plt.show()

    return rmse, estimates, position_std


if __name__ == "__main__":
    print("=" * 60)
    print("Lab 4.1: 1D Kalman Filter Demo")
    print("=" * 60)

    # Run the demo
    rmse, estimates, uncertainties = run_kalman_filter_demo()

    print("\n" + "=" * 60)
    print("Demo complete! Results saved to kalman_filter_results.png")
    print("=" * 60)
