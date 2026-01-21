#!/usr/bin/env python3
"""
Lab 2.3: Dynamics Simulation in MuJoCo - STARTER CODE

This lab:
1. Creates a MuJoCo XML model for 2-link arm
2. Derives Lagrangian dynamics equations analytically
3. Runs simulation with zero torque (passive dynamics)
4. Compares simulated vs analytical accelerations

Learning Objectives:
- MuJoCo model definition (MJCF)
- Lagrangian dynamics derivation
- Simulation validation

Expected Output:
    Max position error: 0.003 rad
    Max velocity error: 0.02 rad/s
    Conclusion: Simulation matches analytical model within 1% error
"""

import numpy as np
import matplotlib.pyplot as plt
import mujoco
from typing import Tuple, List
import time

# Robot parameters
L1 = 1.0  # Length of link 1 (meters)
L2 = 0.8  # Length of link 2 (meters)
m1 = 1.0  # Mass of link 1 (kg)
m2 = 0.8  # Mass of link 2 (kg)

# Center of mass distances (assume at midpoint)
Lc1 = L1 / 2
Lc2 = L2 / 2

# Simulation parameters
DT = 0.001  # Simulation timestep (seconds)
DURATION = 5.0  # Simulation duration (seconds)
STEPS = int(DURATION / DT)


def compute_mass_matrix(q: List[float], m1: float, m2: float,
                        L1: float, L2: float, Lc1: float, Lc2: float) -> np.ndarray:
    """
    TODO: Implement this function

    Compute the 2x2 mass matrix for 2-link planar arm.

    M(q) = | M11  M12 |
           | M21  M22 |

    Where:
    M11 = m1*Lc1^2 + m2*(L1^2 + Lc2^2 + 2*L1*Lc2*cos(q2))
    M12 = m2*(Lc2^2 + L1*Lc2*cos(q2))
    M21 = M12
    M22 = m2*Lc2^2

    Parameters:
    -----------
    q : List[float]
        [q1, q2] joint angles in radians
    m1, m2 : float
        Link masses
    L1, L2 : float
        Link lengths
    Lc1, Lc2 : float
        Center of mass distances from joints

    Returns:
    --------
    np.ndarray
        2x2 mass matrix
    """
    q1, q2 = q

    # === YOUR CODE START ===
    # HINT: Use the formulas above to compute M11, M12, M22
    pass
    # === YOUR CODE END ===

    M = np.array([
        [M11, M12],
        [M12, M22]
    ])

    return M


def compute_coriolis_matrix(q: List[float], qd: List[float],
                            m1: float, m2: float,
                            L1: float, L2: float, Lc1: float, Lc2: float) -> np.ndarray:
    """
    TODO: Implement this function

    Compute the 2x2 Coriolis/centrifugal matrix for 2-link planar arm.

    C(q, qd) encodes velocity-dependent forces:
    - Centrifugal: proportional to qd_i^2
    - Coriolis: proportional to qd_i * qd_j

    Simplified expressions:
    C11 = -m2 * L1 * Lc2 * sin(q2) * qd2
    C12 = -m2 * L1 * Lc2 * sin(q2) * (qd1 + qd2)
    C21 = m2 * L1 * Lc2 * sin(q2) * qd1
    C22 = 0

    Parameters:
    -----------
    q : List[float]
        [q1, q2] joint angles in radians
    qd : List[float]
        [qd1, qd2] joint velocities in rad/s

    Returns:
    --------
    np.ndarray
        2x2 Coriolis matrix
    """
    q1, q2 = q
    qd1, qd2 = qd

    # === YOUR CODE START ===
    # HINT: Use the formulas above for C11, C12, C21, C22
    pass
    # === YOUR CODE END ===

    C = np.array([
        [C11, C12],
        [C21, C22]
    ])

    return C


def compute_analytical_acceleration(q: List[float], qd: List[float],
                                    m1: float, m2: float,
                                    L1: float, L2: float,
                                    Lc1: float, Lc2: float) -> np.ndarray:
    """
    Compute analytical joint accelerations for zero torque (passive dynamics).

    For zero torque: M(q)*qdd + C(q, qd)*qd = 0
    Therefore: qdd = -M(q)^(-1) * C(q, qd) * qd

    Parameters:
    -----------
    q : List[float]
        Joint positions
    qd : List[float]
        Joint velocities

    Returns:
    --------
    np.ndarray
        Joint accelerations [qdd1, qdd2]
    """
    # Compute mass matrix
    M = compute_mass_matrix(q, m1, m2, L1, L2, Lc1, Lc2)

    # Compute Coriolis matrix
    C = compute_coriolis_matrix(q, qd, m1, m2, L1, L2, Lc1, Lc2)

    # Compute qdd = -M^(-1) * C * qd
    qdd = -np.linalg.solve(M, C @ qd)

    return qdd


def run_mujoco_simulation(xml_path: str, initial_q: List[float],
                          duration: float, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Run MuJoCo simulation with zero torque.

    Parameters:
    -----------
    xml_path : str
        Path to MuJoCo XML model
    initial_q : List[float]
        Initial joint configuration [q1, q2]
    duration : float
        Simulation duration in seconds
    dt : float
        Simulation timestep in seconds

    Returns:
    --------
    Tuple[np.ndarray, np.ndarray, np.ndarray]
        (time_array, position_array, velocity_array)
    """
    # Load model
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Set initial configuration
    data.qpos[:] = initial_q
    mujoco.mj_forward(model, data)

    # Storage for results
    n_steps = int(duration / dt)
    time_array = np.zeros(n_steps)
    position_array = np.zeros((n_steps, 2))
    velocity_array = np.zeros((n_steps, 2))

    print(f"Running MuJoCo simulation for {duration}s ({n_steps} steps)...")

    # Run simulation
    for step in range(n_steps):
        # Apply zero torque (passive dynamics)
        data.ctrl[:] = [0.0, 0.0]

        # Step simulation
        mujoco.mj_step(model, data)

        # Record state
        time_array[step] = step * dt
        position_array[step] = data.qpos.copy()
        velocity_array[step] = data.qvel.copy()

        if step % 1000 == 0:
            print(f"  Step {step}/{n_steps}: q = {data.qpos}")

    print(f"Simulation complete!")
    return time_array, position_array, velocity_array


def compute_analytical_trajectory(initial_q: List[float], initial_qd: List[float],
                                  m1: float, m2: float,
                                  L1: float, L2: float,
                                  Lc1: float, Lc2: float,
                                  time_array: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute analytical trajectory using Euler integration of dynamics.

    Uses: qdd = -M^(-1) * C * qd
    Integrated with: qd_new = qd + qdd * dt
                     q_new = q + qd_new * dt

    Parameters:
    -----------
    initial_q : List[float]
        Initial joint positions
    initial_qd : List[float]
        Initial joint velocities
    time_array : np.ndarray
        Time points to evaluate

    Returns:
    --------
    Tuple[np.ndarray, np.ndarray]
        (position_trajectory, velocity_trajectory)
    """
    n_steps = len(time_array)
    dt = time_array[1] - time_array[0]

    position_trajectory = np.zeros((n_steps, 2))
    velocity_trajectory = np.zeros((n_steps, 2))

    # Initialize
    q = np.array(initial_q)
    qd = np.array(initial_qd)

    position_trajectory[0] = q
    velocity_trajectory[0] = qd

    for step in range(1, n_steps):
        # Compute analytical acceleration
        qdd = compute_analytical_acceleration(q, qd, m1, m2, L1, L2, Lc1, Lc2)

        # Euler integration
        qd = qd + qdd * dt
        q = q + qd * dt

        position_trajectory[step] = q
        velocity_trajectory[step] = qd

    return position_trajectory, velocity_trajectory


def plot_results(time_array: np.ndarray,
                 sim_position: np.ndarray, sim_velocity: np.ndarray,
                 ana_position: np.ndarray, ana_velocity: np.ndarray) -> None:
    """
    Plot comparison of simulated vs analytical results.
    """
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # Joint 1 position
    ax = axes[0, 0]
    ax.plot(time_array, sim_position[:, 0], 'b-', linewidth=2, label='MuJoCo')
    ax.plot(time_array, ana_position[:, 0], 'r--', linewidth=2, label='Analytical')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('q1 (rad)')
    ax.set_title('Joint 1 Position')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Joint 2 position
    ax = axes[0, 1]
    ax.plot(time_array, sim_position[:, 1], 'b-', linewidth=2, label='MuJoCo')
    ax.plot(time_array, ana_position[:, 1], 'r--', linewidth=2, label='Analytical')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('q2 (rad)')
    ax.set_title('Joint 2 Position')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Joint 1 velocity
    ax = axes[1, 0]
    ax.plot(time_array, sim_velocity[:, 0], 'b-', linewidth=2, label='MuJoCo')
    ax.plot(time_array, ana_velocity[:, 0], 'r--', linewidth=2, label='Analytical')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('qd1 (rad/s)')
    ax.set_title('Joint 1 Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Joint 2 velocity
    ax = axes[1, 1]
    ax.plot(time_array, sim_velocity[:, 1], 'b-', linewidth=2, label='MuJoCo')
    ax.plot(time_array, ana_velocity[:, 1], 'r--', linewidth=2, label='Analytical')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('qd2 (rad/s)')
    ax.set_title('Joint 2 Velocity')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Position error
    ax = axes[2, 0]
    pos_error = np.linalg.norm(sim_position - ana_position, axis=1)
    ax.plot(time_array, pos_error, 'g-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (rad)')
    ax.set_title(f'Position Error (max: {np.max(pos_error):.4f} rad)')
    ax.grid(True, alpha=0.3)

    # Velocity error
    ax = axes[2, 1]
    vel_error = np.linalg.norm(sim_velocity - ana_velocity, axis=1)
    ax.plot(time_array, vel_error, 'g-', linewidth=2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity Error (rad/s)')
    ax.set_title(f'Velocity Error (max: {np.max(vel_error):.4f} rad/s)')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('lab2_3_dynamics_comparison.png', dpi=150, bbox_inches='tight')
    print("Results saved to 'lab2_3_dynamics_comparison.png'")


def main():
    """
    Main function to run dynamics simulation lab.
    """
    print("=" * 60)
    print("Lab 2.3: Dynamics Simulation in MuJoCo")
    print("=" * 60)

    # Initial conditions
    initial_q = [0.5, 0.5]  # radians
    initial_qd = [0.0, 0.0]  # rad/s (starting from rest)

    print(f"\nRobot Parameters:")
    print(f"  L1 = {L1} m, L2 = {L2} m")
    print(f"  m1 = {m1} kg, m2 = {m2} kg")
    print(f"\nInitial Conditions:")
    print(f"  q = {initial_q} rad")
    print(f"  qd = {initial_qd} rad/s")

    # Path to MuJoCo XML model
    xml_path = "arm_model.xml"

    print(f"\n" + "-" * 60)
    print("PART 1: MuJoCo Simulation")
    print("-" * 60)

    # Run MuJoCo simulation
    start_time = time.time()
    time_array, sim_position, sim_velocity = run_mujoco_simulation(
        xml_path, initial_q, DURATION, DT
    )
    sim_time = time.time() - start_time
    print(f"Simulation time: {sim_time:.2f}s")

    print(f"\n" + "-" * 60)
    print("PART 2: Analytical Dynamics")
    print("-" * 60)

    # Compute analytical trajectory
    ana_position, ana_velocity = compute_analytical_trajectory(
        initial_q, initial_qd,
        m1, m2, L1, L2, Lc1, Lc2,
        time_array
    )

    print("Analytical trajectory computed!")

    print(f"\n" + "-" * 60)
    print("PART 3: Validation Results")
    print("-" * 60)

    # Compute errors
    pos_error = np.linalg.norm(sim_position - ana_position, axis=1)
    vel_error = np.linalg.norm(sim_velocity - ana_velocity, axis=1)

    print(f"\nPosition Error:")
    print(f"  Mean: {np.mean(pos_error):.6f} rad")
    print(f"  Max:  {np.max(pos_error):.6f} rad")
    print(f"  Min:  {np.min(pos_error):.6f} rad")

    print(f"\nVelocity Error:")
    print(f"  Mean: {np.mean(vel_error):.6f} rad/s")
    print(f"  Max:  {np.max(vel_error):.6f} rad/s")
    print(f"  Min:  {np.min(vel_error):.6f} rad/s")

    # Final state comparison
    print(f"\nFinal State (t = {DURATION}s):")
    print(f"  Simulated: q = {sim_position[-1]}, qd = {sim_velocity[-1]}")
    print(f"  Analytical: q = {ana_position[-1]}, qd = {ana_velocity[-1]}")

    # Conclusion
    max_pos_error = np.max(pos_error)
    max_vel_error = np.max(vel_error)

    print(f"\n" + "-" * 60)
    print("CONCLUSION")
    print("-" * 60)
    print(f"Max position error: {max_pos_error:.3f} rad")
    print(f"Max velocity error: {max_vel_error:.3f} rad/s")

    if max_pos_error < 0.01 and max_vel_error < 0.05:
        print("\nConclusion: Simulation matches analytical model within 1% error")
    else:
        print("\nConclusion: Check for modeling errors - errors exceed threshold")

    # Plot results
    print(f"\n" + "-" * 60)
    print("VISUALIZATION")
    print("-" * 60)
    plot_results(time_array, sim_position, sim_velocity, ana_position, ana_velocity)

    print("\n" + "=" * 60)
    print("Lab 2.3 Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
