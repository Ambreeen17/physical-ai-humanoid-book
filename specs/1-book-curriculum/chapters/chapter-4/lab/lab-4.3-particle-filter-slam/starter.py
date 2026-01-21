"""
Lab 4.3: Particle Filter SLAM - Starter Template

This is a template for a GMapping-style Particle Filter SLAM implementation.
Complete the TODOs to implement Rao-Blackwellized particle filtering with
occupancy grid mapping.

Learning Objectives:
- Understand particle filters for non-Gaussian state estimation
- Implement importance sampling with motion and measurement models
- Apply resampling techniques (systematic, stratified)
- Build occupancy grid maps online

Author: Robotics Lab Generator
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional
from dataclasses import dataclass, field


@dataclass
class OccupancyGrid:
    """
    2D occupancy grid map.

    Attributes:
        origin: World coordinates of cell [0, 0]
        resolution: Size of each cell (meters)
        width, height: Grid dimensions in cells
        data: Log-odds values (positive = occupied, negative = free)
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
        """Convert cell indices to world coordinates."""
        wx = self.origin[0] + (cx + 0.5) * self.resolution
        wy = self.origin[1] + (cy + 0.5) * self.resolution
        return wx, wy

    def is_valid(self, cx: int, cy: int) -> bool:
        """Check if cell is within grid bounds."""
        return 0 <= cx < self.width and 0 <= cy < self.height


class ParticleFilterSLAM:
    """
    GMapping-style Particle Filter SLAM implementation.

    Uses Rao-Blackwellized Particle Filter:
    - Each particle represents a robot pose hypothesis
    - Each particle maintains its own map estimate
    - Resampling based on measurement likelihood
    """

    def __init__(self, num_particles: int = 100,
                 motion_noise_std: Tuple[float, float, float] = (0.1, 0.1, 0.05),
                 measurement_noise_std: float = 0.5,
                 resolution: float = 0.1, map_size: float = 20.0):
        """
        Initialize particle filter SLAM.

        Args:
            num_particles: Number of particles in filter
            motion_noise_std: Std dev for [x, y, theta] motion noise
            measurement_noise_std: Std dev for range measurements
            resolution: Map resolution (meters per cell)
            map_size: Map size in meters
        """
        self.num_particles = num_particles
        self.motion_noise_std = motion_noise_std
        self.measurement_noise_std = measurement_noise_std

        # Motion model parameters (MappingG default)
        self.alpha1 = 0.05  # Rotation noise
        self.alpha2 = 0.05  # Translation noise
        self.alpha3 = 0.05  # Rotation noise
        self.alpha4 = 0.05  # Translation noise (odometry)

        # Initialize particles
        self.particles: List[dict] = []
        self.weights: np.ndarray = np.ones(num_particles) / num_particles

        # Map parameters
        self.resolution = resolution
        self.map_size = map_size
        self.grid_size = int(map_size / resolution)

        # Create initial map for each particle
        for _ in range(num_particles):
            self.particles.append({
                'pose': np.array([0.0, 0.0, 0.0]),  # [x, y, theta]
                'map': OccupancyGrid(
                    origin=(-map_size/2, -map_size/2),
                    resolution=resolution,
                    width=self.grid_size,
                    height=self.grid_size
                )
            })

        print(f"[INFO] Particle Filter SLAM initialized")
        print(f"       Particles: {num_particles}")
        print(f"       Map size: {map_size}m x {map_size}m")
        print(f"       Resolution: {resolution}m/cell")

    def sample_normal(self, mean: float, std: float) -> float:
        """Sample from Gaussian distribution."""
        return mean + np.random.normal(0, std)

    def sample_motion_model_odometry(self, prev_pose: np.ndarray,
                                     odometry: Tuple[float, float, float],
                                     particle_pose: np.ndarray) -> np.ndarray:
        """
        Sample new pose from odometry-based motion model.

        Args:
            prev_pose: Previous pose from odometry [x, y, theta]
            odometry: Measured odometry (rot1, trans, rot2)
            particle_pose: Current particle pose [x, y, theta]

        Returns:
            New pose sampled from motion model

        GMapping Motion Model:
        - Sample rotation1 with noise: alpha1 * |rot1| + alpha2 * trans
        - Sample translation with noise: alpha3 * trans + alpha4 * (|rot1| + |rot2|)
        - Sample rotation2 with noise: alpha1 * |rot2| + alpha2 * trans
        """
        rot1, trans, rot2 = odometry

        # ============================================================
        # TODO: Implement odometry motion model sampling
        # ============================================================

        # Add noise to odometry measurements
        # noisy_rot1 = rot1 + self.sample_normal(0, self.alpha1 * abs(rot1) + self.alpha2 * trans)
        # noisy_trans = trans + self.sample_normal(0, self.alpha3 * trans + self.alpha4 * (abs(rot1) + abs(rot2)))
        # noisy_rot2 = rot2 + self.sample_normal(0, self.alpha1 * abs(rot2) + self.alpha2 * trans)

        # Apply motion to particle pose
        # YOUR CODE HERE - Apply noisy odometry to particle_pose

        pass  # Replace

    def beam_range_finder_model(self, measured_range: float, expected_range: float,
                                max_range: float = 10.0) -> float:
        """
        Compute likelihood of range measurement.

        Beam model with three components:
        1. Short range: Gaussian around expected
        2. Max range: Uniform (no echo)
        3. Unexpected objects: Small uniform probability

        Args:
            measured_range: Actual sensor reading
            expected_range: Range to nearest obstacle
            max_range: Maximum sensor range

        Returns:
            Likelihood p(z | x, m)
        """
        z_short = 0.1  # Probability of short reading
        z_max = 0.05   # Probability of max reading
        z_rand = 0.05  # Probability of random reading

        sigma = self.measurement_noise_std

        if measured_range >= max_range * 0.95:
            # Max range reading
            return z_max + z_rand
        elif measured_range < expected_range:
            # Short reading (unexpected obstacle)
            return z_rand
        else:
            # Gaussian around expected range
            p = (1.0 / (sigma * np.sqrt(2 * np.pi))) * \
                np.exp(-(measured_range - expected_range)**2 / (2 * sigma**2))
            return p * (1 - z_max - z_rand - z_short) + z_short

    def compute_measurement_likelihood(self, ranges: np.ndarray, angles: np.ndarray,
                                       particle: dict) -> float:
        """
        Compute likelihood of scan given particle pose and map.

        Args:
            ranges: Laser scan ranges
            angles: Laser scan angles
            particle: Particle with pose and map

        Returns:
            Likelihood of measurement under this particle
        """
        likelihood = 1.0
        pose = particle['pose']
        grid_map = particle['map']
        max_range = np.max(ranges)

        for i, (r, angle) in enumerate(zip(ranges, angles)):
            # Skip invalid measurements
            if r < 0.01 or r > max_range * 0.95:
                continue

            # Compute expected range from ray casting
            expected_range = self.ray_cast(grid_map, pose, angle, max_range)

            # Compute beam likelihood
            p = self.beam_range_finder_model(r, expected_range, max_range)
            likelihood *= p

            # Prevent underflow
            if likelihood < 1e-50:
                likelihood = 1e-50
                break

        return likelihood

    def ray_cast(self, grid_map: OccupancyGrid, pose: np.ndarray,
                 angle: float, max_range: float) -> float:
        """
        Cast ray through occupancy grid.

        Args:
            grid_map: Occupancy grid
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

        # Step size (half resolution for accuracy)
        step = self.resolution / 2.0

        # Cast ray
        for r in np.arange(0, max_range, step):
            x += dx * step
            y += dy * step

            cx, cy = grid_map.world_to_cell(x, y)

            if not grid_map.is_valid(cx, cy):
                return max_range

            # Check for occupied cell (log-odds > 0)
            if grid_map.data[cy, cx] > 0:
                return r

        return max_range

    def update_maps(self, ranges: np.ndarray, angles: np.ndarray):
        """
        Update occupancy grid for each particle using scan.

        Inverse sensor model:
        - Cells along ray (before hit): log-odds decrease (free)
        - Cell at hit: log-odds increase (occupied)
        """
        log_odds_free = -0.2  # Log-odds for free cell
        log_odds_occ = 0.3    # Log-odds for occupied cell
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

                # Cast ray and update cells
                self._update_grid_along_ray(grid_map, pose, (hit_x, hit_y),
                                            angle, r, log_odds_free, log_odds_occ)

    def _update_grid_along_ray(self, grid_map: OccupancyGrid, pose: np.ndarray,
                               hit_point: Tuple[float, float], angle: float,
                               max_r: float, log_odds_free: float, log_odds_occ: float):
        """
        Update grid cells along a ray using Bresenham's algorithm.

        Args:
            grid_map: Occupancy grid to update
            pose: Robot pose
            hit_point: Ray hit point in world coords
            angle: Beam angle
            max_r: Maximum range of beam
            log_odds_free: Value for free cells
            log_odds_occ: Value for occupied cell
        """
        x0, y0 = grid_map.world_to_cell(pose[0], pose[1])
        x1, y1 = grid_map.world_to_cell(hit_point[0], hit_point[1])

        # Bresenham's line algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if grid_map.is_valid(x0, y0):
                # Don't update beyond hit point
                dist = np.sqrt((x0 * self.resolution)**2 + (y0 * self.resolution)**2)
                if dist >= max_r:
                    break

                # Update free cells (before hit)
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

        # Update hit cell (occupied)
        if grid_map.is_valid(x1, y1):
            grid_map.data[y1, x1] = min(1.0, max(-1.0,
                                    grid_map.data[y1, x1] + log_odds_occ))

    def systematic_resample(self, weights: np.ndarray) -> np.ndarray:
        """
        Perform systematic resampling.

        Algorithm:
        1. Compute cumulative sum of normalized weights
        2. Sample starting point u0 ~ Uniform(0, 1/N)
        3. Select particles at positions u0 + (k-1)/N

        Args:
            weights: Particle weights (will be normalized)

        Returns:
            Indices of selected particles
        """
        N = len(weights)

        # ============================================================
        # TODO: Implement systematic resampling
        # ============================================================

        # Normalize weights
        # weights = weights / np.sum(weights)

        # Compute cumulative sum
        # cumsum = np.cumsum(weights)

        # Generate systematic samples
        # u0 = np.random.uniform(0, 1.0/N)
        # u = u0 + np.arange(N) / N
        # indices = np.zeros(N, dtype=int)

        # YOUR CODE HERE - Select particles using cumsum

        pass  # Replace

    def resample(self):
        """Resample particles based on weights."""
        # Compute effective number of particles
        Neff = 1.0 / np.sum(self.weights**2)

        # Resampling threshold (typically N/2)
        if Neff < self.num_particles / 2:
            # Perform resampling
            indices = self.systematic_resample(self.weights)

            # Create new particle set
            new_particles = []
            for i in indices:
                new_particles.append({
                    'pose': self.particles[i]['pose'].copy(),
                    'map': self.particles[i]['map']  # Share reference
                })

            self.particles = new_particles
            self.weights = np.ones(self.num_particles) / self.num_particles

            print(f"[RESAMPLE] Effective particles: {Neff:.1f} -> {self.num_particles}")

    def predict(self, odometry: Tuple[float, float, float]):
        """
        Prediction step: Sample new poses from motion model.

        Args:
            odometry: (rot1, trans, rot2) from odometry
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
        self.weights += 1e-30  # Prevent division by zero
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
        """Get combined map (majority vote from particles)."""
        # Use map from highest-weight particle
        best_idx = np.argmax(self.weights)
        return self.particles[best_idx]['map'].data.copy()


def generate_slam_data(n_steps: int = 500) -> Tuple[List, List]:
    """
    Generate synthetic SLAM data (odometry and laser scans).

    Returns:
        odometry_list: List of (rot1, trans, rot2) tuples
        scan_list: List of (ranges, angles) tuples
    """
    np.random.seed(42)

    odometry_list = []
    scan_list = []

    # Create environment with some obstacles
    landmarks = [
        (2.0, 2.0), (6.0, 1.5), (8.0, 5.0),
        (4.0, 8.0), (1.0, 6.0), (5.0, 4.0)
    ]

    # Robot motion parameters
    v = 0.3
    omega = 0.2
    dt = 0.1

    pose = np.array([0.0, 0.0, 0.0])

    for i in range(n_steps):
        # Vary motion
        omega_i = omega + 0.05 * np.sin(i * 0.03)

        # Compute odometry
        theta = pose[2]
        dtheta = omega_i * dt
        dtrans = v * dt

        rot1 = dtheta
        trans = dtrans
        rot2 = dtheta

        odometry_list.append((rot1, trans, rot2))

        # Update true pose
        pose[0] += trans * np.cos(theta + rot1)
        pose[1] += trans * np.sin(theta + rot1)
        pose[2] = (pose[2] + dtheta) % (2 * np.pi)

        # Generate laser scan
        num_beams = 20
        angles = np.linspace(-np.pi/2, np.pi/2, num_beams)
        ranges = []

        for angle in angles:
            beam_angle = pose[2] + angle
            min_range = 10.0

            for landmark in landmarks:
                dx = landmark[0] - pose[0]
                dy = landmark[1] - pose[1]
                range_to_landmark = np.sqrt(dx**2 + dy**2)

                # Check if landmark is in beam direction
                landmark_angle = np.arctan2(dy, dx)
                angle_diff = ((landmark_angle - beam_angle + np.pi) % (2*np.pi)) - np.pi

                if abs(angle_diff) < 0.1:  # Within beam width
                    min_range = min(min_range, range_to_landmark)

            # Add noise
            range_val = min_range + np.random.normal(0, 0.1)
            ranges.append(max(0.1, range_val))

        scan_list.append((np.array(ranges), angles))

    return odometry_list, scan_list


def run_slam_demo():
    """Demonstrate particle filter SLAM."""
    print("=" * 60)
    print("Particle Filter SLAM Demonstration")
    print("=" * 60)

    # Generate data
    odometry_list, scan_list = generate_slam_data(n_steps=500)

    print(f"\n[DATA] Generated {len(odometry_list)} steps")

    # Initialize SLAM
    slam = ParticleFilterSLAM(
        num_particles=100,
        motion_noise_std=(0.1, 0.1, 0.05),
        measurement_noise_std=0.5,
        resolution=0.1,
        map_size=20.0
    )

    # Run SLAM
    print(f"\n[RUN] Running particle filter SLAM...")

    for i, (odom, scan) in enumerate(zip(odometry_list, scan_list)):
        # Prediction
        slam.predict(odom)

        # Update
        ranges, angles = scan
        slam.update(ranges, angles)

        if i % 100 == 0:
            print(f"       Step {i}/{len(odometry_list)}")

    print(f"       Complete!")

    # Get results
    best_pose = slam.get_best_pose()
    occ_grid = slam.get_map()

    # Visualization
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # Plot 1: Occupancy grid
    ax1 = axes[0]
    ax1.imshow(occ_grid, cmap='gray_r', origin='lower',
               extent=[-10, 10, -10, 10])
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('Occupancy Grid Map')

    # Plot 2: Particle cloud
    ax2 = axes[1]
    particle_poses = np.array([p['pose'] for p in slam.particles])
    ax2.scatter(particle_poses[:, 0], particle_poses[:, 1], c='blue', s=10, alpha=0.5)
    ax2.scatter(best_pose[0], best_pose[1], c='red', s=100, marker='x')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Particle Cloud (100 particles)')
    ax2.axis('equal')
    ax2.grid(True, alpha=0.3)

    # Plot 3: Weight distribution
    ax3 = axes[2]
    ax3.hist(slam.weights, bins=20, edgecolor='black')
    ax3.set_xlabel('Weight')
    ax3.set_ylabel('Count')
    ax3.set_title('Particle Weight Distribution')
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('particle_filter_slam_results.png', dpi=150)
    plt.show()


if __name__ == "__main__":
    run_slam_demo()
