"""
Lab 4.2: EKF Localization - Complete Solution

This file contains the complete implementation of an Extended Kalman Filter
for 2D robot localization with range-bearing landmark observations.

Author: Robotics Lab Generator
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Optional
import math


class EKFLocalization:
    """
    Extended Kalman Filter for 2D robot localization.

    This EKF implements:
    - Unicycle motion model for differential drive / Ackermann robots
    - Range-bearing observation model for landmark detection
    - Data association using Mahalanobis distance gating

    State: x = [x, y, theta]^T
    - x, y: Robot position in world frame (meters)
    - theta: Robot heading (radians)
    """

    def __init__(self, process_noise_std: Tuple[float, float, float] = (0.1, 0.1, 0.05),
                 measurement_noise_std: Tuple[float, float] = (0.3, 0.1)):
        """
        Initialize EKF localization with noise parameters.

        Args:
            process_noise_std: Std dev of [x, y, theta] process noise
                              Higher values = more trust in measurements
            measurement_noise_std: Std dev of [range, bearing] measurement noise
                                  Lower values = more trust in sensors
        """
        # Process noise covariance: models uncertainty in motion
        # - Translation noise: affects x, y
        # - Rotation noise: affects theta
        self.Q = np.diag([process_noise_std[0]**2,
                         process_noise_std[1]**2,
                         process_noise_std[2]**2])

        # Measurement noise covariance: models sensor uncertainty
        # - Range noise: std dev of range measurements
        # - Bearing noise: std dev of bearing measurements
        self.R = np.diag([measurement_noise_std[0]**2,
                         measurement_noise_std[1]**2])

        print(f"[INFO] EKF Localization initialized")
        print(f"       Process noise diag: {np.diag(self.Q)}")
        print(f"       Measurement noise diag: {np.diag(self.R)}")

    def motion_model(self, state: np.ndarray, control: np.ndarray,
                     dt: float) -> np.ndarray:
        """
        Unicycle motion model.

        Implements the exact solution for unicycle dynamics:
            dx/dt = v * cos(theta)
            dy/dt = v * sin(theta)
            dtheta/dt = omega

        Args:
            state: Current state [x, y, theta]
            control: Control input [v, omega]
            dt: Time step (seconds)

        Returns:
            New state after applying controls
        """
        x, y, theta = state
        v, omega = control

        # Handle small omega using limit or direct integration
        if abs(omega) < 1e-6:
            # Small omega approximation
            x_new = x + v * np.cos(theta) * dt
            y_new = y + v * np.sin(theta) * dt
            theta_new = theta + omega * dt
        else:
            # Exact solution using rotation radius
            R = v / omega  # Turning radius

            # Compute using trigonometric identities
            # This avoids accumulated errors from Euler integration
            x_new = x + R * (np.sin(theta + omega * dt) - np.sin(theta))
            y_new = y - R * (np.cos(theta + omega * dt) - np.cos(theta))
            theta_new = theta + omega * dt

        # Normalize angle to [-pi, pi]
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))

        return np.array([x_new, y_new, theta_new])

    def compute_motion_jacobian(self, state: np.ndarray, control: np.ndarray,
                                dt: float) -> np.ndarray:
        """
        Compute Jacobian of motion model w.r.t. state.

        For the unicycle model, this is the 3x3 matrix:
            F = d(f(x,u))/dx evaluated at current state

        Partial derivatives:
            d(x_new)/dx = 1
            d(x_new)/dy = 0
            d(x_new)/dtheta = -v * sin(theta) * dt

            d(y_new)/dx = 0
            d(y_new)/dy = 1
            d(y_new)/dtheta = v * cos(theta) * dt

            d(theta_new)/dx = 0
            d(theta_new)/dy = 0
            d(theta_new)/dtheta = 1

        Args:
            state: Current state [x, y, theta]
            control: Control input [v, omega]
            dt: Time step

        Returns:
            3x3 Jacobian matrix F
        """
        x, y, theta = state
        v, omega = control

        # Initialize Jacobian
        F = np.eye(3)

        # Off-diagonal elements
        F[0, 2] = -v * np.sin(theta) * dt
        F[1, 2] = v * np.cos(theta) * dt

        return F

    def predict(self, x_prev: np.ndarray, P_prev: np.ndarray,
                control: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        EKF predict step.

        Projects state and covariance forward using the motion model.

        Args:
            x_prev: Previous state estimate [x, y, theta]
            P_prev: Previous state covariance (3x3)
            control: Control input [v, omega]
            dt: Time step

        Returns:
            Predicted state and covariance
        """
        # Predict state using nonlinear motion model
        x_pred = self.motion_model(x_prev, control, dt)

        # Linearize around current estimate
        F = self.compute_motion_jacobian(x_prev, control, dt)

        # Propagate uncertainty using Jacobian
        P_pred = F @ P_prev @ F.T + self.Q

        return x_pred, P_pred

    def observation_model(self, state: np.ndarray, landmark: np.ndarray) -> np.ndarray:
        """
        Convert landmark position to range-bearing measurement.

        Args:
            state: Robot state [x, y, theta]
            landmark: Landmark position [lx, ly]

        Returns:
            Expected measurement [range, bearing]
        """
        x, y, theta = state
        lx, ly = landmark

        # Vector from robot to landmark
        dx = lx - x
        dy = ly - y

        # Range (distance)
        range_val = np.sqrt(dx**2 + dy**2)

        # Bearing angle relative to world frame
        bearing_world = np.arctan2(dy, dx)

        # Bearing relative to robot heading
        bearing_val = bearing_world - theta

        # Normalize to [-pi, pi]
        bearing_val = np.arctan2(np.sin(bearing_val), np.cos(bearing_val))

        return np.array([range_val, bearing_val])

    def compute_observation_jacobian(self, state: np.ndarray,
                                     landmark: np.ndarray) -> np.ndarray:
        """
        Compute Jacobian of observation model w.r.t. state.

        For range-bearing measurement z = h(x):
            z[0] = range = sqrt((lx-x)^2 + (ly-y)^2)
            z[1] = bearing = atan2(ly-y, lx-x) - theta

        Partial derivatives (chain rule):
            d(range)/dx = -(lx-x)/range = -dx/range
            d(range)/dy = -(ly-y)/range = -dy/range
            d(range)/dtheta = 0

            d(bearing)/dx = -(ly-y)/range^2 = -dy/range^2
            d(bearing)/dy = (lx-x)/range^2 = dx/range^2
            d(bearing)/dtheta = -1

        Args:
            state: Robot state [x, y, theta]
            landmark: Landmark position [lx, ly]

        Returns:
            2x3 Jacobian matrix H
        """
        x, y, theta = state
        lx, ly = landmark

        dx = lx - x
        dy = ly - y
        range_sq = dx**2 + dy**2
        range_val = np.sqrt(range_sq)

        # Avoid division by zero
        if range_val < 1e-10:
            return np.zeros((2, 3))

        # Build Jacobian row by row
        H = np.zeros((2, 3))

        # Row 0: derivatives of range
        H[0, 0] = -dx / range_val  # d(range)/dx
        H[0, 1] = -dy / range_val  # d(range)/dy
        H[0, 2] = 0.0              # d(range)/dtheta

        # Row 1: derivatives of bearing
        H[1, 0] = -dy / range_sq   # d(bearing)/dx
        H[1, 1] = dx / range_sq    # d(bearing)/dy
        H[1, 2] = -1.0             # d(bearing)/dtheta

        return H

    def update(self, x_pred: np.ndarray, P_pred: np.ndarray,
               z: np.ndarray, landmark: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        EKF update step with range-bearing measurement.

        Performs Bayesian update combining prediction with observation.

        Args:
            x_pred: Predicted state from predict step
            P_pred: Predicted covariance from predict step
            z: Measured [range, bearing]
            landmark: Known landmark position [lx, ly]

        Returns:
            Updated state and covariance
        """
        # Expected measurement
        z_pred = self.observation_model(x_pred, landmark)

        # Innovation (measurement residual)
        y = z - z_pred
        y[1] = np.arctan2(np.sin(y[1]), np.cos(y[1]))  # Normalize bearing

        # Observation Jacobian
        H = self.compute_observation_jacobian(x_pred, landmark)

        # Innovation covariance
        S = H @ P_pred @ H.T + self.R

        # Kalman gain (using solve for numerical stability)
        K = P_pred @ H.T @ np.linalg.inv(S)

        # Update state
        x_hat = x_pred + K @ y

        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(3) - K @ H
        P_hat = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T

        return x_hat, P_hat

    def data_association(self, x_pred: np.ndarray, P_pred: np.ndarray,
                         measurements: list, landmarks: np.ndarray,
                         gate_size: float = 5.0) -> list:
        """
        Associate measurements with landmarks using Mahalanobis distance.

        Uses nearest-neighbor with Mahalanobis distance gating to prevent
        incorrect associations.

        Args:
            x_pred: Predicted robot state
            P_pred: Predicted robot covariance
            measurements: List of [range, bearing] measurements
            landmarks: Array of known landmark positions (N x 2)
            gate_size: Maximum Mahalanobis distance threshold

        Returns:
            List of (measurement_idx, landmark_idx) tuples
        """
        associations = []
        used_landmarks = set()

        for m_idx, z in enumerate(measurements):
            best_landmark = None
            best_mahal = float('inf')

            for l_idx, landmark in enumerate(landmarks):
                if l_idx in used_landmarks:
                    continue

                # Expected measurement for this landmark
                z_pred = self.observation_model(x_pred, landmark)

                # Innovation
                y = z - z_pred
                y[1] = np.arctan2(np.sin(y[1]), np.cos(y[1]))

                # Jacobian and innovation covariance
                H = self.compute_observation_jacobian(x_pred, landmark)
                S = H @ P_pred @ H.T + self.R

                # Mahalanobis distance
                mahal = y.T @ np.linalg.inv(S) @ y

                if mahal < best_mahal and mahal < gate_size**2:
                    best_mahal = mahal
                    best_landmark = l_idx

            if best_landmark is not None:
                associations.append((m_idx, best_landmark))
                used_landmarks.add(best_landmark)

        return associations


def generate_synthetic_data(n_steps: int = 500,
                            dt: float = 0.1) -> Tuple[np.ndarray, list, np.ndarray]:
    """
    Generate synthetic robot motion and landmark observations.

    Creates a realistic scenario with:
    - Circular trajectory with varying angular velocity
    - Gaussian measurement noise on range and bearing
    - Multiple visible landmarks

    Args:
        n_steps: Number of time steps to simulate
        dt: Time step duration

    Returns:
        true_states: Ground truth robot states
        measurements: Observations at each step
        landmarks: Known landmark positions
    """
    np.random.seed(42)

    # Define landmarks in a realistic pattern
    landmarks = np.array([
        [2.0, 2.0],
        [6.0, 1.5],
        [8.0, 5.0],
        [4.0, 8.0],
        [1.0, 6.0],
        [5.0, 4.0],  # Central landmark
    ])

    # Robot parameters
    v_base = 0.5  # Base linear velocity

    # Generate true trajectory (lissajous-like pattern)
    true_states = []
    state = np.array([0.0, 0.0, 0.0])  # [x, y, theta]

    for i in range(n_steps):
        # Vary velocity and omega for interesting path
        omega = 0.3 + 0.1 * np.sin(i * 0.02)
        control = np.array([v_base, omega])

        # Apply exact motion model
        x, y, theta = state
        if abs(omega) < 1e-6:
            state[0] = x + v_base * np.cos(theta) * dt
            state[1] = y + v_base * np.sin(theta) * dt
            state[2] = theta + omega * dt
        else:
            R = v_base / omega
            state[0] = x + R * (np.sin(theta + omega * dt) - np.sin(theta))
            state[1] = y - R * (np.cos(theta + omega * dt) - np.cos(theta))
            state[2] = theta + omega * dt

        # Normalize theta
        state[2] = np.arctan2(np.sin(state[2]), np.cos(state[2]))

        true_states.append(state.copy())

    true_states = np.array(true_states)

    # Generate noisy measurements
    measurements = []
    range_noise_std = 0.3
    bearing_noise_std = 0.1

    for state in true_states:
        step_measurements = []
        for landmark in landmarks:
            dx = landmark[0] - state[0]
            dy = landmark[1] - state[1]
            range_val = np.sqrt(dx**2 + dy**2)
            bearing_val = np.arctan2(dy, dx) - state[2]

            # Add Gaussian noise
            range_val += np.random.normal(0, range_noise_std)
            bearing_val += np.random.normal(0, bearing_noise_std)
            bearing_val = np.arctan2(np.sin(bearing_val), np.cos(bearing_val))

            # Only observe landmarks within range
            if range_val < 12.0:
                step_measurements.append(np.array([range_val, bearing_val]))

        measurements.append(step_measurements)

    return true_states, measurements, landmarks


def run_ekf_demo():
    """
    Demonstrate EKF localization with synthetic data.

    This function:
    1. Generates synthetic robot motion and landmark observations
    2. Runs the EKF localization algorithm
    3. Computes performance metrics
    4. Visualizes results
    """
    print("=" * 60)
    print("EXTENDED KALMAN FILTER LOCALIZATION")
    print("=" * 60)

    # Generate synthetic data
    true_states, measurements, landmarks = generate_synthetic_data(n_steps=500)

    print(f"\n[DATA] Generated trajectory with {len(true_states)} steps")
    print(f"       {len(landmarks)} landmarks in environment")
    print(f"       Average measurements per step: {np.mean([len(m) for m in measurements]):.1f}")

    # Initialize EKF
    ekf = EKFLocalization(
        process_noise_std=(0.1, 0.1, 0.05),
        measurement_noise_std=(0.3, 0.1)
    )

    # Initialize state estimate at origin with moderate uncertainty
    x = np.array([0.0, 0.0, 0.0])
    P = np.diag([1.0, 1.0, 0.5])

    print(f"\n[INIT] Starting EKF with state: {x}")
    print(f"       Initial covariance diag: {np.diag(P)}")

    # Storage for analysis
    estimates = [x.copy()]
    covariances = [P.copy()]

    # Process each time step
    print(f"\n[RUN] Processing trajectory...")

    for i in range(len(true_states)):
        # Control input for this step
        v = 0.5
        omega = 0.3 + 0.1 * np.sin(i * 0.02)
        control = np.array([v, omega])

        # EKF Predict
        x, P = ekf.predict(x, P, control, dt=0.1)

        # EKF Update with visible landmarks
        if measurements[i]:
            # Associate measurements with landmarks
            associations = ekf.data_association(x, P, measurements[i], landmarks)

            for m_idx, l_idx in associations:
                z = measurements[i][m_idx]
                landmark = landmarks[l_idx]
                x, P = ekf.update(x, P, z, landmark)

        estimates.append(x.copy())
        covariances.append(P.copy())

    estimates = np.array(estimates)
    covariances = np.array(covariances)

    print(f"       Complete! Processed {len(true_states)} steps")

    # ============================================================
    # Compute Performance Metrics
    # ============================================================
    position_errors = np.sqrt((estimates[:-1, 0] - true_states[:, 0])**2 +
                              (estimates[:-1, 1] - true_states[:, 1])**2)
    heading_errors = np.abs(estimates[:-1, 2] - true_states[:, 2])

    # Normalize heading errors
    heading_errors = np.minimum(heading_errors, 2*np.pi - heading_errors)

    rmse_position = np.sqrt(np.mean(position_errors**2))
    rmse_heading = np.sqrt(np.mean(heading_errors**2))
    max_position_error = np.max(position_errors)

    # Uncertainty metrics
    avg_position_std = np.mean([np.sqrt(cov[0,0] + cov[1,1]) for cov in covariances])

    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    print(f"\nState Estimation:")
    print(f"  Final position: ({estimates[-1, 0]:.3f}, {estimates[-1, 1]:.3f}) m")
    print(f"  Final heading:  {estimates[-1, 2]:.3f} rad ({np.degrees(estimates[-1, 2]):.1f} deg)")
    print(f"\nTrue final state: ({true_states[-1, 0]:.3f}, {true_states[-1, 1]:.3f}) m")
    print(f"                  {true_states[-1, 2]:.3f} rad ({np.degrees(true_states[-1, 2]):.1f} deg)")

    print(f"\nAccuracy Metrics:")
    print(f"  Position RMSE: {rmse_position:.3f} m")
    print(f"  Heading RMSE:  {rmse_heading:.3f} rad ({np.degrees(rmse_heading):.1f} deg)")
    print(f"  Max position error: {max_position_error:.3f} m")
    print(f"\nUncertainty:")
    print(f"  Avg position std: {avg_position_std:.3f} m")
    print(f"  Final heading std: {np.sqrt(covariances[-1, 2, 2]):.3f} rad")

    # ============================================================
    # Visualization
    # ============================================================
    fig = plt.figure(figsize=(16, 10))

    # Plot 1: Trajectory comparison
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.plot(true_states[:, 0], true_states[:, 1], 'g-', linewidth=2, label='True Trajectory')
    ax1.plot(estimates[:, 0], estimates[:, 1], 'r-', linewidth=1.5, alpha=0.8, label='EKF Estimate')
    ax1.scatter(landmarks[:, 0], landmarks[:, 1], c='blue', s=120, marker='^',
                edgecolors='black', linewidth=1, label='Landmarks', zorder=5)
    ax1.scatter(true_states[0, 0], true_states[0, 1], c='green', s=200, marker='o',
                edgecolors='black', linewidth=2, label='Start', zorder=5)
    ax1.scatter(true_states[-1, 0], true_states[-1, 1], c='red', s=200, marker='x',
                linewidth=3, label='End (true)', zorder=5)
    ax1.scatter(estimates[-1, 0], estimates[-1, 1], c='orange', s=200, marker='x',
                linewidth=3, label='End (est)', zorder=5)

    # Draw some uncertainty ellipses
    for i in [0, 100, 200, 300, 400]:
        cov = covariances[i]
        ellipse = plot_uncertainty_ellipse(estimates[i, 0], estimates[i, 1], cov[:2, :2],
                                           ax=ax1, n_std=2.0, alpha=0.3)
        ax1.add_patch(ellipse)

    ax1.set_xlabel('X (m)', fontsize=11)
    ax1.set_ylabel('Y (m)', fontsize=11)
    ax1.set_title('EKF Localization: Trajectory with 2-Sigma Uncertainty', fontsize=12)
    ax1.legend(loc='upper left', fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # Plot 2: Position error over time
    ax2 = fig.add_subplot(2, 2, 2)
    time = np.arange(len(position_errors)) * 0.1
    ax2.plot(time, position_errors, 'b-', linewidth=1.5, label='Position Error')
    ax2.axhline(y=rmse_position, color='r', linestyle='--', label=f'RMSE = {rmse_position:.3f} m')
    ax2.fill_between(time, 0, position_errors, alpha=0.3)
    ax2.set_xlabel('Time (s)', fontsize=11)
    ax2.set_ylabel('Position Error (m)', fontsize=11)
    ax2.set_title('Position Estimation Error Over Time', fontsize=12)
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Plot 3: Uncertainty evolution
    ax3 = fig.add_subplot(2, 2, 3)
    position_stds = [np.sqrt(cov[0,0]) for cov in covariances]
    position_std_y = [np.sqrt(cov[1,1]) for cov in covariances]
    heading_stds = [np.sqrt(cov[2,2]) for cov in covariances]

    ax3.plot(time, position_stds, 'r-', linewidth=1.5, label='X uncertainty (std)')
    ax3.plot(time, position_std_y, 'b-', linewidth=1.5, label='Y uncertainty (std)')
    ax3.plot(time, heading_stds, 'g-', linewidth=1.5, label='Theta uncertainty (rad)')
    ax3.set_xlabel('Time (s)', fontsize=11)
    ax3.set_ylabel('Uncertainty (std dev)', fontsize=11)
    ax3.set_title('Filter Uncertainty Evolution', fontsize=12)
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Plot 4: Error distributions
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.hist(position_errors, bins=30, density=True, alpha=0.7, color='blue', edgecolor='black')
    ax4.axvline(x=rmse_position, color='r', linestyle='--', linewidth=2, label=f'RMSE = {rmse_position:.3f} m')
    ax4.axvline(x=np.mean(position_errors), color='orange', linestyle='--', linewidth=2,
                label=f'Mean = {np.mean(position_errors):.3f} m')
    ax4.set_xlabel('Position Error (m)', fontsize=11)
    ax4.set_ylabel('Density', fontsize=11)
    ax4.set_title('Error Distribution', fontsize=12)
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('ekf_localization_results.png', dpi=150, bbox_inches='tight')
    plt.show()


def plot_uncertainty_ellipse(x, y, cov, ax, n_std=2.0, **kwargs):
    """
    Plot 2D uncertainty ellipse from covariance matrix.

    Args:
        x, y: Center position
        cov: 2x2 covariance matrix
        ax: Matplotlib axis
        n_std: Number of standard deviations

    Returns:
        Ellipse patch
    """
    # Eigenvalue decomposition
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    # Sort by eigenvalue (largest first)
    order = eigenvalues.argsort()[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    # Compute angle of rotation
    angle = np.degrees(np.arctan2(*eigenvectors[:, 0][::-1]))

    # Compute ellipse width and height
    width, height = 2 * n_std * np.sqrt(eigenvalues)

    # Create ellipse patch
    from matplotlib.patches import Ellipse
    ellipse = Ellipse(xy=(x, y), width=width, height=height, angle=angle, **kwargs)
    ellipse.set_alpha(0.5)

    return ellipse


if __name__ == "__main__":
    print("\n" + "=" * 60)
    print("LAB 4.2: EXTENDED KALMAN FILTER LOCALIZATION")
    print("=" * 60 + "\n")

    run_ekf_demo()

    print("\n" + "=" * 60)
    print("Demo complete!")
    print("Results saved to: ekf_localization_results.png")
    print("=" * 60)
