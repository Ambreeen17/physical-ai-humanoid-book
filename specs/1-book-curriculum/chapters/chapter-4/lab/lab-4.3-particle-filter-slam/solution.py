"""
Lab 4.3: Particle Filter SLAM - Complete Solution

GMapping-style Rao-Blackwellized Particle Filter implementation for
simultaneous localization and mapping with occupancy grids.

Author: Robotics Lab Generator
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional
from dataclasses import dataclass, field
import time


@dataclass
class OccupancyGrid:
    """
    2D occupancy grid map for storing environmental information.

    Uses log-odds representation for efficient Bayesian updates:
    - Positive values: occupied cells
    - Negative values: free cells
    - Near zero: unknown/uncharted

    Attributes:
        origin: World coordinates of cell [0, 0] (bottom-left corner)
        resolution: Size of each cell in meters
        width, height: Grid dimensions in number of cells
        data: 2D array storing log-odds values
    """
    origin: Tuple[float, float] = (0.0, 0.0)
    resolution: float = 0.1
    width: int = 200
    height: int = 200
    data: np.ndarray = field(default_factory=lambda: np.zeros((200, 200)))

    def world_to_cell(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coordinates to cell indices."""
        cx = int((wx - self.origin[0]) / self.resolution)
        cy = int((wy - self.origin[1]) / self.resolution)
        return cx, cy

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        """Convert cell indices to world coordinates (returns cell center)."""
        wx = self.origin[0] + (cx + 0.5) * self.resolution
        wy = self.origin[1] + (cy + 0.5) * self.resolution
        return wx, wy

    def is_valid(self, cx: int, cy: int) -> bool:
        """Check if cell coordinates are within grid bounds."""
        return 0 <= cx < self.width and 0 <= cy < self.height


class ParticleFilterSLAM:
    """
    GMapping-style Rao-Blackwellized Particle Filter SLAM.

    This implementation uses the RBPF algorithm where:
    1. Each particle represents a robot pose hypothesis
    2. Each particle maintains its own map estimate
    3. Particles are weighted by measurement likelihood
    4. Resampling prevents particle depletion

    Key components:
    - Odometry motion model with noise parameters
    - Beam model for range measurements
    - Systematic resampling for efficient weight-based selection
    - Occupancy grid mapping with inverse sensor model
    """

    def __init__(self, num_particles: int = 100,
                 motion_noise_std: Tuple[float, float, float] = (0.1, 0.1, 0.05),
                 measurement_noise_std: float = 0.5,
                 resolution: float = 0.1, map_size: float = 20.0,
                 resample_threshold: float = 0.5):
        """
        Initialize particle filter SLAM system.

        Args:
            num_particles: Number of particles in filter (more = better, slower)
            motion_noise_std: Std dev for motion noise in [x, y, theta]
            measurement_noise_std: Std dev for range measurements
            resolution: Map resolution (meters per cell)
            map_size: Total map size in meters (square map)
            resample_threshold: Fraction of particles below which to resample
        """
        self.num_particles = num_particles
        self.motion_noise_std = motion_noise_std
        self.measurement_noise_std = measurement_noise_std
        self.resample_threshold = resample_threshold

        # Motion model noise parameters (from GMapping paper)
        # These control how much we trust odometry vs. assume slippage
        self.alpha1 = 0.05   # Noise for rotation1
        self.alpha2 = 0.05   # Noise for translation
        self.alpha3 = 0.05   # Noise for rotation2
        self.alpha4 = 0.05   # Odometry noise scaling

        # Map parameters
        self.resolution = resolution
        self.map_size = map_size
        self.grid_size = int(map_size / resolution)

        # Initialize particles and weights
        self.particles: List[dict] = []
        self.weights: np.ndarray = np.ones(num_particles) / num_particles

        # Create particles with poses and maps
        for _ in range(num_particles):
            self.particles.append({
                'pose': np.array([0.0, 0.0, 0.0]),  # [x, y, theta]
                'map': self._create_empty_map()
            })

        print(f"[INFO] Particle Filter SLAM initialized")
        print(f"       Particles: {num_particles}")
        print(f"       Map size: {map_size}m x {map_size}m ({self.grid_size}x{self.grid_size} cells)")
        print(f"       Resolution: {resolution}m/cell")

    def _create_empty_map(self) -> OccupancyGrid:
        """Create an empty occupancy grid."""
        return OccupancyGrid(
            origin=(-self.map_size/2, -self.map_size/2),
            resolution=self.resolution,
            width=self.grid_size,
            height=self.grid_size
        )

    def sample_normal(self, mean: float, std: float) -> float:
        """Sample from Gaussian distribution."""
        return mean + np.random.normal(0, std)

    def sample_motion_model_odometry(self, prev_pose: np.ndarray,
                                     odometry: Tuple[float, float, float],
                                     particle_pose: np.ndarray) -> np.ndarray:
        """
        Sample new pose from odometry-based motion model.

        Implements the GMapping motion model which samples from a Gaussian
        centered on the noisy odometry measurement.

        Args:
            prev_pose: Previous pose from odometry [x, y, theta]
            odometry: Measured odometry (rot1, trans, rot2)
            particle_pose: Current particle pose [x, y, theta]

        Returns:
            New pose sampled from motion model

        Mathematical Model:
            noisy_rot1 ~ N(rot1, alpha1*rot1^2 + alpha2*trans^2)
            noisy_trans ~ N(trans, alpha3*trans^2 + alpha4*(rot1+rot2)^2)
            noisy_rot2 ~ N(rot2, alpha1*rot2^2 + alpha2*trans^2)
        """
        rot1, trans, rot2 = odometry

        # Compute noise parameters based on motion magnitude
        rot1_var = self.alpha1 * rot1**2 + self.alpha2 * trans**2
        trans_var = self.alpha3 * trans**2 + self.alpha4 * (rot1 + rot2)**2
        rot2_var = self.alpha1 * rot2**2 + self.alpha2 * trans**2

        # Sample noisy odometry
        noisy_rot1 = rot1 + self.sample_normal(0, np.sqrt(max(0, rot1_var)))
        noisy_trans = trans + self.sample_normal(0, np.sqrt(max(0, trans_var)))
        noisy_rot2 = rot2 + self.sample_normal(0, np.sqrt(max(0, rot2_var)))

        # Apply motion to particle pose
        x, y, theta = particle_pose

        # Compute new pose
        new_x = x + noisy_trans * np.cos(theta + noisy_rot1)
        new_y = y + noisy_trans * np.sin(theta + noisy_rot1)
        new_theta = theta + noisy_rot1 + noisy_rot2

        # Normalize angle
        new_theta = np.arctan2(np.sin(new_theta), np.cos(new_theta))

        return np.array([new_x, new_y, new_theta])

    def beam_range_finder_model(self, measured_range: float, expected_range: float,
                                max_range: float = 10.0) -> float:
        """
        Compute likelihood of a single range measurement.

        Implements the beam model with four components:
        1. Correct range with Gaussian noise
        2. Short range (unexpected obstacles)
        3. Max range (no return)
        4. Random measurements

        Args:
            measured_range: Actual sensor reading
            expected_range: Range to nearest obstacle from map
            max_range: Maximum sensor range

        Returns:
            Likelihood p(z | x, m) in (0, 1]
        """
        # Model parameters
        z_hit = 0.8      # Probability of correct measurement
        z_short = 0.1    # Probability of short measurement (unexpected obstacle)
        z_max = 0.05     # Probability of max range (no echo)
        z_rand = 0.05    # Probability of random measurement
        sigma = self.measurement_noise_std

        # Clamp expected range to avoid numerical issues
        expected_range = min(expected_range, max_range)

        # Compute probabilities for each component
        p = 0.0

        # 1. Correct measurement (Gaussian)
        if expected_range < max_range:
            p += z_hit * (1.0 / (sigma * np.sqrt(2 * np.pi))) * \
                np.exp(-(measured_range - expected_range)**2 / (2 * sigma**2))

        # 2. Short measurement (unexpected obstacle)
        if measured_range < expected_range:
            p += z_short * (2.0 / expected_range) * (1 - measured_range / expected_range)

        # 3. Max range (no echo)
        if measured_range >= max_range * 0.95:
            p += z_max

        # 4. Random measurement
        p += z_rand / max_range

        return max(p, 1e-50)  # Prevent underflow

    def compute_measurement_likelihood(self, ranges: np.ndarray, angles: np.ndarray,
                                       particle: dict) -> float:
        """
        Compute likelihood of full laser scan given particle pose and map.

        Assumes independence between beams (naive Bayes).

        Args:
            ranges: Array of range measurements from laser
            angles: Array of beam angles relative to robot heading
            particle: Dictionary with 'pose' and 'map'

        Returns:
            Likelihood of measurement under this particle
        """
        log_likelihood = 0.0
        pose = particle['pose']
        grid_map = particle['map']
        max_range = np.max(ranges)

        for r, angle in zip(ranges, angles):
            # Skip invalid measurements
            if r < 0.01 or r > max_range * 0.95:
                continue

            # Compute expected range from ray casting
            expected_range = self.ray_cast(grid_map, pose, angle, max_range)

            # Compute beam likelihood (add in log space)
            p = self.beam_range_finder_model(r, expected_range, max_range)
            log_likelihood += np.log(p)

            # Early termination for very low likelihood
            if log_likelihood < -50:
                return 1e-50

        return np.exp(log_likelihood)

    def ray_cast(self, grid_map: OccupancyGrid, pose: np.ndarray,
                 angle: float, max_range: float) -> float:
        """
        Cast ray through occupancy grid to find range to obstacle.

        Uses ray marching with step size = resolution/2 for accuracy.

        Args:
            grid_map: Occupancy grid to cast through
            pose: Robot pose [x, y, theta]
            angle: Beam angle relative to robot heading
            max_range: Maximum range to cast

        Returns:
            Range to first occupied cell, or max_range if none
        """
        # Compute beam direction in world frame
        beam_angle = pose[2] + angle
        dx = np.cos(beam_angle)
        dy = np.sin(beam_angle)

        # Start from robot position
        x, y = pose[0], pose[1]

        # Step size for ray marching (half resolution for accuracy)
        step = self.resolution / 2.0
        max_steps = int(max_range / step)

        for _ in range(max_steps):
            x += dx * step
            y += dy * step

            cx, cy = grid_map.world_to_cell(x, y)

            if not grid_map.is_valid(cx, cy):
                return max_range

            # Check for occupied cell (positive log-odds)
            if grid_map.data[cy, cx] > 0.2:
                # Compute actual range to this cell
                return np.sqrt((x - pose[0])**2 + (y - pose[1])**2)

        return max_range

    def update_maps(self, ranges: np.ndarray, angles: np.ndarray):
        """
        Update occupancy grid for each particle using new laser scan.

        Implements inverse sensor model:
        - Cells along ray (before hit): log-odds decrease (free)
        - Cell at hit: log-odds increase (occupied)

        Args:
            ranges: Laser scan ranges
            angles: Laser scan angles
        """
        log_odds_free = -0.3   # Log-odds for free cell
        log_odds_occ = 0.6     # Log-odds for occupied cell
        max_range = np.max(ranges)

        for particle in self.particles:
            pose = particle['pose']
            grid_map = particle['map']

            for r, angle in zip(ranges, angles):
                if r >= max_range * 0.95 or r < 0.01:
                    continue

                # Compute hit point in world coordinates
                beam_angle = pose[2] + angle
                hit_x = pose[0] + r * np.cos(beam_angle)
                hit_y = pose[1] + r * np.sin(beam_angle)

                # Update grid cells along ray
                self._update_grid_along_ray(grid_map, pose, (hit_x, hit_y),
                                            log_odds_free, log_odds_occ)

    def _update_grid_along_ray(self, grid_map: OccupancyGrid, pose: np.ndarray,
                               hit_point: Tuple[float, float],
                               log_odds_free: float, log_odds_occ: float):
        """
        Update grid cells along a ray using Bresenham's algorithm.

        Uses efficient line drawing to mark cells along ray.

        Args:
            grid_map: Grid to update
            pose: Robot pose
            hit_point: Ray hit point in world coordinates
            log_odds_free: Value to add for free cells
            log_odds_occ: Value to add for occupied cell
        """
        x0, y0 = grid_map.world_to_cell(pose[0], pose[1])
        x1, y1 = grid_map.world_to_cell(hit_point[0], hit_point[1])

        # Bresenham's line algorithm for efficient ray casting
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if grid_map.is_valid(x0, y0):
                # Compute range from robot to this cell
                cell_wx, cell_wy = grid_map.cell_to_world(x0, y0)
                dist = np.sqrt((cell_wx - pose[0])**2 + (cell_wy - pose[1])**2)

                hit_wx, hit_wy = hit_point
                hit_dist = np.sqrt((hit_wx - pose[0])**2 + (hit_wy - pose[1])**2)

                # Check if we've reached hit point
                if dist >= hit_dist - self.resolution:
                    # Update hit cell as occupied
                    grid_map.data[y0, x0] = min(1.0, max(-1.0,
                                            grid_map.data[y0, x0] + log_odds_occ))
                    break

                # Update cells as free (before hit)
                grid_map.data[y0, x0] = min(1.0, max(-1.0,
                                        grid_map.data[y0, x0] + log_odds_free))

            if x0 == x1 and y0 == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def systematic_resample(self, weights: np.ndarray) -> np.ndarray:
        """
        Perform systematic resampling (low variance sampling).

        This algorithm is more efficient than multinomial resampling
        and has lower variance in the number of offspring.

        Algorithm:
        1. Compute cumulative sum of normalized weights
        2. Sample starting point u0 ~ Uniform(0, 1/N)
        3. Select particles at positions u0 + (k-1)/N for k = 1..N

        Args:
            weights: Particle weights (will be normalized in-place)

        Returns:
            Indices of selected particles
        """
        N = len(weights)

        # Normalize weights
        weights = np.array(weights)
        weights += 1e-30  # Prevent numerical issues
        weights /= np.sum(weights)

        # Compute cumulative sum
        cumsum = np.cumsum(weights)

        # Generate systematic samples
        # Start with uniform random offset in [0, 1/N)
        u0 = np.random.uniform(0, 1.0 / N)

        # Positions to sample at
        u = u0 + np.arange(N) / N

        # Select particles using cumulative sum
        indices = np.zeros(N, dtype=int)
        j = 0
        for i in range(N):
            while cumsum[j] < u[i]:
                j += 1
            indices[i] = j

        return indices

    def resample(self):
        """Resample particles based on weights when effective N drops."""
        # Compute effective number of particles
        Neff = 1.0 / np.sum(self.weights**2)

        # Resampling threshold
        if Neff < self.num_particles * self.resample_threshold:
            # Perform resampling
            indices = self.systematic_resample(self.weights)

            # Create new particle set (copy to avoid aliasing issues)
            new_particles = []
            for i in indices:
                new_particle = {
                    'pose': self.particles[i]['pose'].copy(),
                    'map': self._create_empty_map()
                }
                # Copy map data
                new_particle['map'].data[:] = self.particles[i]['map'].data[:]
                new_particles.append(new_particle)

            self.particles = new_particles
            self.weights = np.ones(self.num_particles) / self.num_particles

            print(f"[RESAMPLE] Neff = {Neff:.1f} < {self.num_particles * self.resample_threshold:.0f}")
            print(f"           Resampled to maintain particle diversity")

    def predict(self, odometry: Tuple[float, float, float]):
        """
        Prediction step: Sample new poses from motion model.

        For each particle, sample a new pose from the motion model
        distribution p(x_t | x_{t-1}, u_t).

        Args:
            odometry: (rot1, trans, rot2) odometry measurement
        """
        for particle in self.particles:
            prev_pose = particle['pose']
            particle['pose'] = self.sample_motion_model_odometry(
                prev_pose, odometry, prev_pose
            )

    def update(self, ranges: np.ndarray, angles: np.ndarray):
        """
        Update step: Compute weights and update maps.

        Args:
            ranges: Laser scan ranges
            angles: Laser scan angles
        """
        # Compute weight for each particle
        for i, particle in enumerate(self.particles):
            likelihood = self.compute_measurement_likelihood(ranges, angles, particle)
            self.weights[i] *= likelihood

        # Normalize weights
        self.weights += 1e-50  # Prevent underflow
        self.weights /= np.sum(self.weights)

        # Update maps
        self.update_maps(ranges, angles)

        # Resample if needed
        self.resample()

    def get_best_pose(self) -> np.ndarray:
        """Get pose from highest-weight particle."""
        best_idx = np.argmax(self.weights)
        return self.particles[best_idx]['pose'].copy()

    def get_map(self) -> np.ndarray:
        """Get occupancy grid from highest-weight particle."""
        best_idx = np.argmax(self.weights)
        return self.particles[best_idx]['map'].data.copy()

    def get_particle_stats(self) -> dict:
        """Get statistics about particle distribution."""
        poses = np.array([p['pose'] for p in self.particles])

        # Compute effective number
        Neff = 1.0 / np.sum(self.weights**2)

        return {
            'mean_pose': np.average(poses, weights=self.weights, axis=0),
            'cov_pose': np.cov(poses.T, aweights=self.weights),
            'effective_n': Neff,
            'max_weight': np.max(self.weights),
            'pose_spread': np.mean(np.std(poses, axis=0))
        }


def generate_slam_data(n_steps: int = 500) -> Tuple[List, List]:
    """
    Generate synthetic SLAM data with circular trajectory.

    Creates realistic odometry and laser scan data in a simple
    environment with landmark obstacles.

    Args:
        n_steps: Number of time steps to generate

    Returns:
        odometry_list: List of (rot1, trans, rot2) tuples
        scan_list: List of (ranges, angles) tuples
    """
    np.random.seed(42)

    odometry_list = []
    scan_list = []

    # Create environment with obstacles
    landmarks = [
        (2.0, 2.0), (6.0, 1.5), (8.0, 5.0),
        (4.0, 8.0), (1.0, 6.0), (5.0, 4.0),
        (3.0, 3.0), (7.0, 6.0), (9.0, 2.0)
    ]

    # Robot motion parameters
    v_base = 0.3  # Base linear velocity
    dt = 0.1      # Time step

    # True pose
    true_pose = np.array([0.0, 0.0, 0.0])

    for i in range(n_steps):
        # Vary motion to create interesting path
        omega = 0.25 + 0.08 * np.sin(i * 0.02)

        # Compute odometry from true motion
        dtheta = omega * dt
        dtrans = v_base * dt

        # Odometry measurement
        rot1 = dtheta
        trans = dtrans
        rot2 = dtheta

        odometry_list.append((rot1, trans, rot2))

        # Update true pose (exact integration)
        true_pose[0] += trans * np.cos(true_pose[2] + rot1)
        true_pose[1] += trans * np.sin(true_pose[2] + rot1)
        true_pose[2] = np.arctan2(np.sin(true_pose[2] + dtheta),
                                   np.cos(true_pose[2] + dtheta))

        # Generate laser scan
        num_beams = 30
        angles = np.linspace(-np.pi/2, np.pi/2, num_beams)
        ranges = []

        for angle in angles:
            beam_angle = true_pose[2] + angle
            min_range = 10.0  # Max sensor range

            # Find closest landmark in beam direction
            for lx, ly in landmarks:
                dx = lx - true_pose[0]
                dy = ly - true_pose[1]
                range_to_landmark = np.sqrt(dx**2 + dy**2)

                # Check if landmark is in beam direction
                landmark_angle = np.arctan2(dy, dx)
                angle_diff = ((landmark_angle - beam_angle + np.pi) % (2*np.pi)) - np.pi

                if abs(angle_diff) < 0.08:  # Beam width
                    if range_to_landmark < min_range:
                        min_range = range_to_landmark

            # Add measurement noise
            if min_range >= 10.0:
                # No obstacle detected
                range_val = 10.0 + np.random.normal(0, 0.1)
            else:
                range_val = min_range + np.random.normal(0, 0.1)

            ranges.append(max(0.1, min(10.0, range_val)))

        scan_list.append((np.array(ranges), angles))

    return odometry_list, scan_list


def run_slam_demo():
    """
    Demonstrate particle filter SLAM with synthetic data.

    This function:
    1. Generates synthetic odometry and laser scan data
    2. Runs the particle filter SLAM algorithm
    3. Visualizes results including map, particles, and convergence
    """
    print("=" * 70)
    print("PARTICLE FILTER SLAM (GMapping-style)")
    print("=" * 70)

    # Generate synthetic data
    print("\n[DATA] Generating synthetic SLAM data...")
    odometry_list, scan_list = generate_slam_data(n_steps=500)
    print(f"       Generated {len(odometry_list)} steps of data")

    # Initialize SLAM system
    print("\n[INIT] Initializing particle filter...")
    slam = ParticleFilterSLAM(
        num_particles=100,
        motion_noise_std=(0.1, 0.1, 0.05),
        measurement_noise_std=0.5,
        resolution=0.1,
        map_size=20.0,
        resample_threshold=0.5
    )

    # Run SLAM
    print("\n[RUN] Running particle filter SLAM...")
    start_time = time.time()

    for i, (odom, scan) in enumerate(zip(odometry_list, scan_list)):
        # Prediction: sample from motion model
        slam.predict(odom)

        # Update: compute weights and update maps
        ranges, angles = scan
        slam.update(ranges, angles)

        if (i + 1) % 100 == 0:
            stats = slam.get_particle_stats()
            print(f"       Step {i+1}/{len(odometry_list)} | "
                  f"Neff: {stats['effective_n']:.1f} | "
                  f"Max weight: {stats['max_weight']:.3f}")

    elapsed_time = time.time() - start_time
    print(f"\n       Completed in {elapsed_time:.2f} seconds")

    # Get results
    best_pose = slam.get_best_pose()
    occ_grid = slam.get_map()
    stats = slam.get_particle_stats()

    # ============================================================
    # Print Results
    # ============================================================
    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)
    print(f"\nBest Pose Estimate: ({best_pose[0]:.3f}, {best_pose[1]:.3f}, {best_pose[2]:.3f} rad)")
    print(f"Mean Pose: ({stats['mean_pose'][0]:.3f}, {stats['mean_pose'][1]:.3f}, "
          f"{stats['mean_pose'][2]:.3f} rad)")
    print(f"\nParticle Statistics:")
    print(f"  Effective particles: {stats['effective_n']:.1f}")
    max_w = np.max(slam.weights)
    print(f"  Max weight: {max_w:.4f}")
    print(f"  Pose spread: {stats['pose_spread']:.3f} m")

    # ============================================================
    # Visualization
    # ============================================================
    fig = plt.figure(figsize=(18, 12))

    # Plot 1: Occupancy Grid Map
    ax1 = fig.add_subplot(2, 2, 1)
    im = ax1.imshow(occ_grid, cmap='gray_r', origin='lower',
                    extent=[-10, 10, -10, 10], vmin=-1, vmax=1)
    ax1.set_xlabel('X (m)', fontsize=11)
    ax1.set_ylabel('Y (m)', fontsize=11)
    ax1.set_title('Built Occupancy Grid Map', fontsize=12)
    plt.colorbar(im, ax=ax1, label='Log-odds')

    # Plot 2: Particle Cloud
    ax2 = fig.add_subplot(2, 2, 2)
    particle_poses = np.array([p['pose'] for p in slam.particles])

    # Color particles by weight
    colors = slam.weights
    scatter = ax2.scatter(particle_poses[:, 0], particle_poses[:, 1],
                          c=colors, cmap='viridis', s=30, alpha=0.7)
    ax2.scatter(best_pose[0], best_pose[1], c='red', s=200, marker='x',
                linewidths=3, label='Best pose', zorder=10)
    ax2.set_xlabel('X (m)', fontsize=11)
    ax2.set_ylabel('Y (m)', fontsize=11)
    ax2.set_title(f'Particle Cloud (N={slam.num_particles})', fontsize=12)
    ax2.legend()
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)
    plt.colorbar(scatter, ax=ax2, label='Weight')

    # Plot 3: Weight Distribution
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.hist(slam.weights, bins=20, edgecolor='black', alpha=0.7)
    ax3.axvline(x=1.0/slam.num_particles, color='r', linestyle='--',
                label=f'Uniform = {1.0/slam.num_particles:.4f}')
    ax3.set_xlabel('Weight', fontsize=11)
    ax3.set_ylabel('Count', fontsize=11)
    ax3.set_title('Particle Weight Distribution', fontsize=12)
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Plot 4: Map with best particle's trajectory
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.imshow(occ_grid, cmap='gray_r', origin='lower',
               extent=[-10, 10, -10, 10], vmin=-1, vmax=1)

    # Get trajectory from best particle
    best_idx = np.argmax(slam.weights)
    trajectory = []

    # Reconstruct trajectory by simulating motion model
    temp_pose = np.array([0.0, 0.0, 0.0])
    for odom in odometry_list:
        temp_pose = slam.sample_motion_model_odometry(temp_pose, odom, temp_pose)
        trajectory.append(temp_pose.copy())

    trajectory = np.array(trajectory)
    ax4.plot(trajectory[:, 0], trajectory[:, 1], 'r-', linewidth=1,
             alpha=0.7, label='Estimated trajectory')
    ax4.scatter(best_pose[0], best_pose[1], c='blue', s=100, marker='o',
                label='Final pose', zorder=10)
    ax4.set_xlabel('X (m)', fontsize=11)
    ax4.set_ylabel('Y (m)', fontsize=11)
    ax4.set_title('Estimated Trajectory on Map', fontsize=12)
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('particle_filter_slam_results.png', dpi=150, bbox_inches='tight')
    plt.show()


if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("LAB 4.3: PARTICLE FILTER SLAM (GMapping-style)")
    print("=" * 70 + "\n")

    run_slam_demo()

    print("\n" + "=" * 70)
    print("Demo complete!")
    print("Results saved to: particle_filter_slam_results.png")
    print("=" * 70)
