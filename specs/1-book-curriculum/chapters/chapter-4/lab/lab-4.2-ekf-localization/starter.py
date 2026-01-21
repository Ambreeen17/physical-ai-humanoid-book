"""
Lab 4.2: EKF Localization - Starter Template

This is a template for the Extended Kalman Filter localization lab.
Complete the TODOs to implement EKF-based 2D robot localization.

Learning Objectives:
- Understand EKF for nonlinear state estimation
- Implement range-bearing landmark measurement models
- Linearize nonlinear models using Jacobians
- Handle data association for multiple landmarks

Author: Robotics Lab Generator
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Optional
import math


class EKFLocalization:
    """
    Extended Kalman Filter for 2D robot localization.

    State: x = [x, y, theta]^T
    - x, y: Robot position in world frame
    - theta: Robot heading (radians)

    The filter uses:
    - Unicycle motion model (velocity + angular velocity)
    - Range-bearing observations of known landmarks
    """

    def __init__(self, process_noise_std: Tuple[float, float, float] = (0.1, 0.1, 0.05),
                 measurement_noise_std: Tuple[float, float] = (0.3, 0.1)):
        """
        Initialize EKF localization.

        Args:
            process_noise_std: Std dev of [x, y, theta] process noise
            measurement_noise_std: Std dev of [range, bearing] measurement noise
        """
        # Process noise covariance (motion model uncertainty)
        self.Q = np.diag([process_noise_std[0]**2,
                         process_noise_std[1]**2,
                         process_noise_std[2]**2])

        # Measurement noise covariance (sensor uncertainty)
        self.R = np.diag([measurement_noise_std[0]**2,
                         measurement_noise_std[1]**2])

        print(f"[INFO] EKF Localization initialized")
        print(f"       Process noise: {process_noise_std}")
        print(f"       Measurement noise: {measurement_noise_std}")

    def motion_model(self, state: np.ndarray, control: np.ndarray,
                     dt: float) -> np.ndarray:
        """
        Unicycle motion model.

        Predicts new state given current state and control inputs.

        Args:
            state: Current state [x, y, theta]
            control: Control input [v, omega] (linear and angular velocity)
            dt: Time step in seconds

        Returns:
            Predicted state after applying controls

        Mathematical Model:
            x_new = x + v * cos(theta) * dt
            y_new = y + v * sin(theta) * dt
            theta_new = theta + omega * dt
        """
        x, y, theta = state
        v, omega = control

        # ============================================================
        # TODO: Implement the unicycle motion model
        # ============================================================

        # Handle small omega to avoid numerical issues
        # If omega is very small, use direct integration
        if abs(omega) < 1e-6:
            # YOUR CODE HERE
            # Compute new state without rotation
            # x_new = ...
            # y_new = ...
            # theta_new = ...
            pass
        else:
            # YOUR CODE HERE
            # Compute new state with proper rotation
            # Use the exact solution for unicycle model
            # R = v / omega
            # x_new = x + R * (sin(theta + omega*dt) - sin(theta))
            # y_new = y - R * (cos(theta + omega*dt) - cos(theta))
            # theta_new = theta + omega * dt
            pass

        # Return as column vector
        # return np.array([x_new, y_new, theta_new])
        pass  # Replace

    def compute_motion_jacobian(self, state: np.ndarray, control: np.ndarray,
                                dt: float) -> np.ndarray:
        """
        Compute Jacobian of motion model w.r.t. state.

        This is the matrix of partial derivatives:
        F[i,j] = d(state_new[i]) / d(state[j])

        Args:
            state: Current state [x, y, theta]
            control: Control input [v, omega]
            dt: Time step

        Returns:
            3x3 Jacobian matrix F

        Jacobian for unicycle model:
        F = [[1, 0, -v*sin(theta)*dt],
             [0, 1,  v*cos(theta)*dt],
             [0, 0,              1   ]]
        """
        x, y, theta = state
        v, omega = control

        # ============================================================
        # TODO: Compute the motion Jacobian
        # ============================================================

        # F = np.zeros((3, 3))
        # F[0, 0] = 1.0
        # F[1, 1] = 1.0
        # F[2, 2] = 1.0
        # F[0, 2] = -v * np.sin(theta) * dt
        # F[1, 2] = v * np.cos(theta) * dt

        # YOUR CODE HERE - Complete the Jacobian

        pass  # Replace

    def predict(self, x_prev: np.ndarray, P_prev: np.ndarray,
                control: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """
        EKF predict step.

        Args:
            x_prev: Previous state estimate [x, y, theta]
            P_prev: Previous state covariance (3x3)
            control: Control input [v, omega]
            dt: Time step

        Returns:
            Predicted state and covariance
        """
        # Predict state using motion model
        x_pred = self.motion_model(x_prev, control, dt)

        # Compute Jacobian
        F = self.compute_motion_jacobian(x_prev, control, dt)

        # Predict covariance: F @ P_prev @ F.T + Q
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

        Mathematical Model:
            range = sqrt((lx - x)^2 + (ly - y)^2)
            bearing = atan2(ly - y, lx - x) - theta
        """
        x, y, theta = state
        lx, ly = landmark

        # Distance from robot to landmark
        dx = lx - x
        dy = ly - y
        range_val = np.sqrt(dx**2 + dy**2)

        # Bearing angle (relative to robot heading)
        bearing_val = np.arctan2(dy, dx) - theta

        # Normalize bearing to [-pi, pi]
        while bearing_val > np.pi:
            bearing_val -= 2 * np.pi
        while bearing_val < -np.pi:
            bearing_val += 2 * np.pi

        return np.array([range_val, bearing_val])

    def compute_observation_jacobian(self, state: np.ndarray,
                                     landmark: np.ndarray) -> np.ndarray:
        """
        Compute Jacobian of observation model w.r.t. state.

        H[i,j] = d(measurement[i]) / d(state[j])

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

        # ============================================================
        # TODO: Compute the observation Jacobian
        # ============================================================

        # H = np.zeros((2, 3))

        # d_range/dx = -(lx - x) / range = -dx / range
        # d_range/dy = -(ly - y) / range = -dy / range
        # d_range/dtheta = 0

        # d_bearing/dx = -(ly - y) / range^2 = -dy / range^2
        # d_bearing/dy = (lx - x) / range^2 = dx / range^2
        # d_bearing/dtheta = -1

        # YOUR CODE HERE

        pass  # Replace

    def update(self, x_pred: np.ndarray, P_pred: np.ndarray,
               z: np.ndarray, landmark: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        EKF update step with range-bearing measurement.

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

        # Kalman gain (numerically stable)
        K = P_pred @ H.T @ np.linalg.inv(S)

        # Update state
        x_hat = x_pred + K @ y

        # Update covariance (Joseph form for stability)
        I_KH = np.eye(3) - K @ H
        P_hat = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T

        return x_hat, P_hat

    def data_association(self, x_pred: np.ndarray, P_pred: np.ndarray,
                         measurements: list, landmarks: np.ndarray,
                         gate_size: float = 5.0) -> list:
        """
        Associate measurements with landmarks using Mahalanobis distance.

        Args:
            x_pred: Predicted robot state
            P_pred: Predicted robot covariance
            measurements: List of [range, bearing] measurements
            landmarks: Array of known landmark positions (N x 2)
            gate_size: Maximum Mahalanobis distance for valid association

        Returns:
            List of (measurement_idx, landmark_idx) associations
        """
        associations = []
        used_landmarks = set()

        for m_idx, z in enumerate(measurements):
            best_landmark = None
            best_mahal = float('inf')

            for l_idx, landmark in enumerate(landmarks):
                if l_idx in used_landmarks:
                    continue

                # Compute expected measurement
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

    Returns:
        true_states: Array of true robot states
        measurements: List of measurements at each step
        landmarks: Array of landmark positions
    """
    np.random.seed(42)

    # Define landmarks (known positions)
    landmarks = np.array([
        [2.0, 2.0],
        [6.0, 1.5],
        [8.0, 5.0],
        [4.0, 8.0],
        [1.0, 6.0],
    ])

    # Robot parameters
    v = 0.5  # Linear velocity
    omega = 0.3  # Angular velocity (will be varied)

    # Generate true trajectory (circular motion)
    true_states = []
    state = np.array([0.0, 0.0, 0.0])  # [x, y, theta]

    for i in range(n_steps):
        # Vary omega to create interesting path
        omega_i = omega + 0.1 * np.sin(i * 0.05)
        control = np.array([v, omega_i])

        # Apply motion model (exact solution)
        x, y, theta = state
        if abs(omega_i) < 1e-6:
            state[0] = x + v * np.cos(theta) * dt
            state[1] = y + v * np.sin(theta) * dt
            state[2] = theta + omega_i * dt
        else:
            R = v / omega_i
            state[0] = x + R * (np.sin(theta + omega_i * dt) - np.sin(theta))
            state[1] = y - R * (np.cos(theta + omega_i * dt) - np.cos(theta))
            state[2] = theta + omega_i * dt

        # Normalize theta
        state[2] = np.arctan2(np.sin(state[2]), np.cos(state[2]))

        true_states.append(state.copy())

    true_states = np.array(true_states)

    # Generate measurements (with noise)
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

            # Add noise
            range_val += np.random.normal(0, range_noise_std)
            bearing_val += np.random.normal(0, bearing_noise_std)
            bearing_val = np.arctan2(np.sin(bearing_val), np.cos(bearing_val))

            # Only include visible landmarks (within range)
            if range_val < 10.0:
                step_measurements.append(np.array([range_val, bearing_val]))

        measurements.append(step_measurements)

    return true_states, measurements, landmarks


def run_ekf_demo():
    """
    Demonstrate EKF localization with synthetic data.
    """
    print("=" * 60)
    print("EKF Localization Demonstration")
    print("=" * 60)

    # Generate data
    true_states, measurements, landmarks = generate_synthetic_data(n_steps=500)

    # Initialize EKF
    ekf = EKFLocalization(
        process_noise_std=(0.1, 0.1, 0.05),
        measurement_noise_std=(0.3, 0.1)
    )

    # Initialize state estimate
    x = np.array([0.0, 0.0, 0.0])  # Start at origin
    P = np.diag([1.0, 1.0, 0.5])   # Initial uncertainty

    # Storage
    estimates = [x.copy()]
    covariances = [P.copy()]

    # Process each step
    for i in range(len(true_states)):
        # Get true state and control for this step
        state = true_states[i]
        v = 0.5
        omega = 0.3 + 0.1 * np.sin(i * 0.05)
        control = np.array([v, omega])

        # Predict
        x, P = ekf.predict(x, P, control, dt=0.1)

        # Update with all visible landmarks
        for z in measurements[i]:
            # Associate measurement with nearest landmark
            min_dist = float('inf')
            best_landmark = None
            for landmark in landmarks:
                z_pred = ekf.observation_model(x, landmark)
                dist = np.linalg.norm(z - z_pred)
                if dist < min_dist:
                    min_dist = dist
                    best_landmark = landmark

            if best_landmark is not None:
                x, P = ekf.update(x, P, z, best_landmark)

        estimates.append(x.copy())
        covariances.append(P.copy())

    estimates = np.array(estimates)
    covariances = np.array(covariances)

    # Compute errors
    position_errors = np.sqrt((estimates[:, 0] - true_states[:, 0])**2 +
                              (estimates[:, 1] - true_states[:, 1])**2)
    heading_errors = np.abs(estimates[:, 2] - true_states[:, 2])

    print(f"\nResults:")
    print(f"  Final position: ({estimates[-1, 0]:.2f}, {estimates[-1, 1]:.2f})")
    print(f"  Final heading: {estimates[-1, 2]:.2f} rad")
    print(f"  Mean position error: {np.mean(position_errors):.3f} m")
    print(f"  Max position error: {np.max(position_errors):.3f} m")
    print(f"  Mean heading error: {np.mean(heading_errors):.3f} rad")

    # Visualization
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # Plot trajectory
    ax1 = axes[0]
    ax1.plot(true_states[:, 0], true_states[:, 1], 'g-', linewidth=2, label='True Trajectory')
    ax1.plot(estimates[:, 0], estimates[:, 1], 'r-', linewidth=1, alpha=0.7, label='EKF Estimate')
    ax1.scatter(landmarks[:, 0], landmarks[:, 1], c='blue', s=100, marker='^',
                label='Landmarks', zorder=5)
    ax1.scatter(true_states[0, 0], true_states[0, 1], c='green', s=200, marker='o',
                label='Start', zorder=5)
    ax1.scatter(true_states[-1, 0], true_states[-1, 1], c='red', s=200, marker='x',
                label='End', zorder=5)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('EKF Localization: Trajectory')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    # Plot errors
    ax2 = axes[1]
    ax2.plot(position_errors, 'b-', linewidth=1, label='Position Error')
    ax2.plot(heading_errors, 'r-', linewidth=1, label='Heading Error (rad)')
    ax2.set_xlabel('Time Step')
    ax2.set_ylabel('Error')
    ax2.set_title('Localization Error Over Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('ekf_localization_results.png', dpi=150)
    plt.show()


if __name__ == "__main__":
    run_ekf_demo()
