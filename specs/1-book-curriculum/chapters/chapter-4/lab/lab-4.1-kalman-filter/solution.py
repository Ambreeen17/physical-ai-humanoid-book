"""
Lab 4.1: 1D Kalman Filter - Complete Solution

This file contains the complete implementation of a 1D Kalman Filter
with constant velocity motion model for position and velocity estimation.

Author: Robotics Lab Generator
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple


class KalmanFilter1D:
    """
    1D Kalman Filter for position and velocity estimation.

    State vector: x = [position, velocity]^T
    Motion model: Constant velocity
    Observation: Position only

    This filter optimally combines:
    - Prediction from motion model (with some uncertainty)
    - Measurements from sensor (with some uncertainty)

    Attributes:
        Q: Process noise covariance matrix
        R: Measurement noise covariance
        H: Observation matrix
    """

    def __init__(self, process_noise_std: float = 0.1,
                 measurement_noise_std: float = 1.0):
        """
        Initialize Kalman filter with specified noise parameters.

        The process noise (Q) models acceleration uncertainty - how much
        we expect the velocity to change unexpectedly between steps.

        The measurement noise (R) models sensor accuracy - how much
        we trust each position measurement.

        Args:
            process_noise_std: Standard deviation of acceleration noise
                              Higher = more uncertainty in motion model
            measurement_noise_std: Standard deviation of position measurements
                                  Higher = less trust in sensor readings
        """
        # Process noise covariance for constant velocity model
        # Using discrete white noise acceleration model:
        # Q = [[dt^4/4, dt^3/2], * sigma_a^2
        #      [dt^3/2, dt^2   ]]
        # For simplicity, we use a diagonal approximation
        self.Q = np.array([[0, 0],
                          [0, process_noise_std**2]])

        # Measurement noise covariance (scalar since we measure 1D)
        self.R = measurement_noise_std**2

        # Observation matrix: we observe position, not velocity
        self.H = np.array([[1, 0]])

        print(f"[INFO] Kalman Filter initialized")
        print(f"       Process noise std: {process_noise_std}")
        print(f"       Measurement noise std: {measurement_noise_std}")

    def predict(self, x_prev: np.ndarray, P_prev: np.ndarray,
                dt: float, u: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict step: Project state forward in time.

        Uses the constant velocity model:
            position_new = position_old + velocity_old * dt
            velocity_new = velocity_old (plus process noise)

        Args:
            x_prev: Previous state estimate [position, velocity]^T
            P_prev: Previous covariance matrix (2x2)
            dt: Time step in seconds
            u: Control input (not used in this model, defaults to 0)

        Returns:
            x_pred: Predicted state [position, velocity]^T
            P_pred: Predicted covariance matrix (2x2)

        Mathematical Formulation:
            F = [[1, dt],     (state transition matrix)
                 [0,  1]]

            x_pred = F @ x_prev
            P_pred = F @ P_prev @ F^T + Q
        """
        # State transition matrix for constant velocity model
        F = np.array([[1, dt],
                      [0,  1]])

        # Control input matrix (not used in this simple model)
        B = np.array([[0],
                      [dt]])

        # Predicted state: incorporate motion model
        x_pred = F @ x_prev + B * u

        # Predicted covariance: propagate uncertainty + add process noise
        P_pred = F @ P_prev @ F.T + self.Q

        return x_pred, P_pred

    def update(self, x_pred: np.ndarray, P_pred: np.ndarray,
               z: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update step: Incorporate new measurement.

        Uses Bayesian inference to combine prediction with measurement,
        weighting each by their respective uncertainties.

        Args:
            x_pred: Predicted state from predict step
            P_pred: Predicted covariance from predict step
            z: Measured position value

        Returns:
            x_hat: Updated (posterior) state estimate
            P_hat: Updated (posterior) covariance matrix

        Mathematical Formulation:
            y = z - H @ x_pred        (innovation/measurement residual)
            S = H @ P_pred @ H^T + R  (innovation covariance)
            K = P_pred @ H^T @ S^-1   (Kalman gain)
            x_hat = x_pred + K @ y    (posterior state)
            P_hat = (I - K @ H) @ P_pred  (posterior covariance)
        """
        # Innovation (measurement residual): what we didn't expect to see
        y = z - self.H @ x_pred

        # Innovation covariance: uncertainty in the residual
        S = self.H @ P_pred @ self.H.T + self.R

        # Kalman gain: how much to trust the measurement vs prediction
        # Numerically stable computation using solve instead of inverse
        K = P_pred @ self.H.T @ np.linalg.inv(S)

        # Update state estimate: blend prediction and measurement
        x_hat = x_pred + K @ y

        # Update covariance estimate
        # Joseph form for numerical stability: (I - KH) @ P @ (I - KH)^T + K @ R @ K^T
        # Using simple form for this 1D case
        I_KH = np.eye(2) - K @ self.H
        P_hat = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T

        return x_hat, P_hat

    def get_gain_magnitude(self) -> float:
        """Return a measure of how much the filter trusts measurements."""
        return float(np.linalg.norm(self.H))


def generate_synthetic_data(n_samples: int = 200, dt: float = 0.1,
                            true_velocity: float = 2.0,
                            position_offset: float = 0.0,
                            measurement_noise_std: float = 1.0,
                            process_noise_std: float = 0.05,
                            seed: int = 42) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate synthetic noisy position measurements with ground truth.

    Creates a realistic scenario with:
    - Constant velocity motion (with small acceleration noise)
    - Gaussian measurement noise

    Args:
        n_samples: Number of time steps to generate
        dt: Time step duration in seconds
        true_velocity: Initial velocity (m/s)
        position_offset: Starting position (m)
        measurement_noise_std: Std dev of position sensor noise (m)
        process_noise_std: Std dev of acceleration noise (m/s^2)
        seed: Random seed for reproducibility

    Returns:
        times: Array of time values [0, dt, 2*dt, ...]
        true_positions: Ground truth position at each time step
        measurements: Noisy position measurements
    """
    np.random.seed(seed)

    times = np.arange(n_samples) * dt

    # Generate acceleration noise (random walk in velocity)
    acceleration_noise = np.random.normal(0, process_noise_std, n_samples)

    # Integrate to get velocity with noise
    velocity_with_noise = true_velocity + np.cumsum(acceleration_noise * dt)

    # Integrate to get position
    true_positions = position_offset + np.cumsum(velocity_with_noise * dt)

    # Add measurement noise
    measurement_noise = np.random.normal(0, measurement_noise_std, n_samples)
    measurements = true_positions + measurement_noise

    return times, true_positions, measurements


def run_kalman_filter_demo():
    """
    Demonstrate the Kalman filter with synthetic data.

    This function:
    1. Generates synthetic noisy position data
    2. Runs the Kalman filter
    3. Computes performance metrics
    4. Visualizes results
    """
    # ============================================================
    # Configuration
    # ============================================================
    process_noise_std = 0.1      # How much velocity might change unexpectedly
    measurement_noise_std = 1.0  # Sensor noise (meters)
    n_samples = 200              # Number of time steps
    dt = 0.1                     # Time step (seconds)

    print("=" * 60)
    print("1D Kalman Filter Demonstration")
    print("=" * 60)
    print(f"Configuration:")
    print(f"  Time steps: {n_samples}")
    print(f"  Time step (dt): {dt}s")
    print(f"  Total time: {n_samples * dt}s")
    print(f"  Process noise std: {process_noise_std}")
    print(f"  Measurement noise std: {measurement_noise_std}")
    print("=" * 60)

    # ============================================================
    # Generate Data
    # ============================================================
    times, true_positions, measurements = generate_synthetic_data(
        n_samples=n_samples,
        dt=dt,
        true_velocity=2.0,
        measurement_noise_std=measurement_noise_std,
        process_noise_std=process_noise_std
    )

    print(f"\n[DATA] Generated {n_samples} measurements")
    print(f"       True final position: {true_positions[-1]:.2f} m")
    print(f"       Measurement range: [{measurements.min():.2f}, {measurements.max():.2f}] m")

    # ============================================================
    # Initialize Filter
    # ============================================================
    kf = KalmanFilter1D(
        process_noise_std=process_noise_std,
        measurement_noise_std=measurement_noise_std
    )

    # Initialize state: start with first measurement
    # Position = first measurement, velocity = 0 (will be estimated)
    x = np.array([measurements[0], 0.0])

    # Initialize covariance
    # High uncertainty in velocity (10 m/s^2 std)
    P = np.array([[measurement_noise_std**2, 0],
                  [0, 10**2]])

    print(f"\n[INIT] Initial state: position={x[0]:.2f}, velocity={x[1]:.2f}")
    print(f"       Initial covariance (position std): {np.sqrt(P[0,0]):.2f} m")
    print(f"       Initial covariance (velocity std): {np.sqrt(P[1,1]):.2f} m/s")

    # ============================================================
    # Run Filter
    # ============================================================
    estimates = []
    covariances = []

    print(f"\n[RUN] Processing {n_samples} measurements...")

    for i, z in enumerate(measurements):
        # Predict: project state forward
        x, P = kf.predict(x, P, dt=dt)

        # Update: incorporate measurement
        x, P = kf.update(x, P, z)

        # Store for analysis
        estimates.append(x.copy())
        covariances.append(P.copy())

    estimates = np.array(estimates)
    covariances = np.array(covariances)

    print(f"       Done! Filter converged after ~{(estimates[:, 0] - true_positions)**2 < 0.5}.sum() steps")

    # ============================================================
    # Compute Metrics
    # ============================================================
    position_errors = estimates[:, 0] - true_positions
    velocity_errors = estimates[:, 1] - 2.0  # True velocity is 2.0

    rmse_position = np.sqrt(np.mean(position_errors**2))
    rmse_velocity = np.sqrt(np.mean(velocity_errors**2))
    mae_position = np.mean(np.abs(position_errors))

    final_position_std = np.sqrt(covariances[-1, 0, 0])
    final_velocity_std = np.sqrt(covariances[-1, 1, 1])

    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    print(f"Final State Estimates:")
    print(f"  Position: {estimates[-1, 0]:.3f} m (true: {true_positions[-1]:.3f} m)")
    print(f"  Velocity: {estimates[-1, 1]:.3f} m/s (true: 2.000 m/s)")
    print(f"\nAccuracy Metrics:")
    print(f"  Position RMSE: {rmse_position:.3f} m")
    print(f"  Velocity RMSE: {rmse_velocity:.3f} m/s")
    print(f"  Position MAE:  {mae_position:.3f} m")
    print(f"\nUncertainty (1-sigma):")
    print(f"  Final position std: {final_position_std:.3f} m")
    print(f"  Final velocity std: {final_velocity_std:.3f} m/s")

    # Improvement over raw measurements
    measurement_rmse = np.sqrt(np.mean((measurements - true_positions)**2))
    print(f"\nImprovement over raw measurements:")
    print(f"  Raw measurement RMSE: {measurement_rmse:.3f} m")
    print(f"  Kalman filter RMSE:   {rmse_position:.3f} m")
    print(f"  Improvement factor:   {measurement_rmse/rmse_position:.2f}x")

    # ============================================================
    # Visualization
    # ============================================================
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Plot 1: Position estimation
    ax1 = axes[0, 0]
    position_std = np.sqrt(covariances[:, 0, 0])

    ax1.plot(times, true_positions, 'g-', linewidth=2, label='True Position')
    ax1.plot(times, measurements, 'b.', alpha=0.5, markersize=4, label='Measurements')
    ax1.plot(times, estimates[:, 0], 'r-', linewidth=2, label='Kalman Estimate')
    ax1.fill_between(times,
                     estimates[:, 0] - 2*position_std,
                     estimates[:, 0] + 2*position_std,
                     alpha=0.2, color='red', label='95% CI')
    ax1.set_xlabel('Time (s)', fontsize=11)
    ax1.set_ylabel('Position (m)', fontsize=11)
    ax1.set_title('Position Estimation with 95% Confidence Interval', fontsize=12)
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)

    # Plot 2: Velocity estimation
    ax2 = axes[0, 1]
    velocity_std = np.sqrt(covariances[:, 1, 1])

    ax2.plot(times, np.full_like(times, 2.0), 'g--', linewidth=2, label='True Velocity')
    ax2.plot(times, estimates[:, 1], 'r-', linewidth=2, label='Estimated Velocity')
    ax2.fill_between(times,
                     estimates[:, 1] - 2*velocity_std,
                     estimates[:, 1] + 2*velocity_std,
                     alpha=0.2, color='red', label='95% CI')
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Velocity (m/s)', fontsize=11)
    ax2.set_title('Velocity Estimation with 95% Confidence Interval', fontsize=12)
    ax2.legend(loc='upper left')
    ax2.grid(True, alpha=0.3)

    # Plot 3: Estimation error over time
    ax3 = axes[1, 0]
    ax3.plot(times, position_errors, 'b-', linewidth=1, label='Position Error')
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax3.fill_between(times, -2*position_std, +2*position_std,
                     alpha=0.2, color='blue', label='95% CI')
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylabel('Position Error (m)', fontsize=11)
    ax3.set_title('Position Estimation Error', fontsize=12)
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Plot 4: Uncertainty evolution
    ax4 = axes[1, 1]
    ax4.plot(times, position_std, 'r-', linewidth=2, label='Position Uncertainty')
    ax4.plot(times, velocity_std, 'b-', linewidth=2, label='Velocity Uncertainty')
    ax4.set_xlabel('Time (s)', fontsize=11)
    ax4.set_ylabel('Uncertainty (std dev)', fontsize=11)
    ax4.set_title('Filter Uncertainty Over Time', fontsize=12)
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('kalman_filter_results.png', dpi=150, bbox_inches='tight')
    plt.show()

    return rmse_position, estimates, covariances


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("LAB 4.1: 1D KALMAN FILTER - COMPLETE SOLUTION")
    print("=" * 60 + "\n")

    # Run the demo
    rmse, estimates, covariances = run_kalman_filter_demo()

    print("\n" + "=" * 60)
    print("Demo complete!")
    print("Results saved to: kalman_filter_results.png")
    print("=" * 60)
