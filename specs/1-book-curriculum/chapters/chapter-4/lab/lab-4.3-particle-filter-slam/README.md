# Lab 4.3: Particle Filter SLAM - GMapping-Style Implementation

## Learning Objectives

By the end of this lab, you will be able to:
- Understand particle filters for non-Gaussian state estimation
- Implement importance sampling with motion and measurement models
- Apply resampling techniques (systematic, stratified)
- Build a GMapping-style occupancy grid map online
- Handle particle depletion and convergence assessment

## Prerequisites

- Completed Labs 4.1 and 4.2 (Kalman filters)
- Python 3.8+
- numpy, matplotlib, scipy
- Understanding of Bayes filters and importance sampling

## Time Estimate: 75 minutes

---

## Theory Overview

### Why Particle Filters?

- **Non-Gaussian distributions**: Particle filters can represent arbitrary posteriors
- **Nonlinear systems**: No linearization required
- **Multi-modal beliefs**: Can represent multiple hypotheses (e.g., kidnapped robot)

### Rao-Blackwellized Particle Filter (RBPF)

GMapping uses RBPF which:
1. Uses particles to represent robot pose distribution
2. Rao-Blackwellizes: computes map given each pose hypothesis analytically
3. Approximates p(x_{1:t}, m | z_{1:t}, u_{1:t}) as particles

### Algorithm Overview

```
for each particle i:
    1. Sample new pose from motion model: x_t^(i) ~ p(x_t | x_{t-1}^(i), u_t)
    2. Compute particle weight: w_t^(i) = w_{t-1}^(i) * p(z_t | x_t^(i), m)
    3. Update map for each particle: m_t^(i) = update(m_{t-1}^(i), x_t^(i), z_t)

Resample particles based on weights
```

### Key Components

**1. Motion Model (Odometry-based)**
```
x_t = x_{t-1} + (1/alpha1*rot1 + alpha2*trans)*cos(theta + alpha1*rot1)
y_t = y_{t-1} + (1/alpha1*rot1 + alpha2*trans)*sin(theta + alpha1*rot1)
theta_t = theta_{t-1} + alpha1*rot1 + alpha3*trans + noise
```

**2. Measurement Model (Beam Model)**
```
p(z_t | x_t, m) = product over beams of p(z_t^k | x_t, m)
```

**3. Resampling**
- Systematic resampling (efficient, low variance)
- Stratified resampling
- Residual resampling

---

## Lab Setup

```bash
cd lab-4.3-particle-filter-slam
pip install numpy matplotlib scipy
```

---

## Exercise Tasks

### Task 1: Implement Odometry Motion Model

```python
def sample_motion_model(odometry: tuple, prev_pose: np.ndarray) -> np.ndarray:
    """
    Sample new pose from odometry motion model.

    Args:
        odometry: (rot1, trans, rot2) from dead reckoning
        prev_pose: Previous pose [x, y, theta]

    Returns:
        New pose sampled from motion model
    """
    # YOUR CODE HERE
    # 1. Compute odometry deltas with noise
    # 2. Apply rotation and translation
    # 3. Sample from Gaussian noise models
    pass
```

### Task 2: Implement Beam Measurement Model

```python
def beam_range_finder_model(beam: np.ndarray, pose: np.ndarray,
                            occupancy_map: OccupancyGrid) -> float:
    """
    Compute likelihood of beam given pose and map.

    Uses the beam model:
    - Short range: high certainty
    - Long range: uncertainty grows
    - Max range: "missing" measurements

    Returns:
        Likelihood p(z | x, m)
    """
    # YOUR CODE HERE
    # 1. Cast ray through map
    # 2. Compute expected range
    # 3. Compute measurement likelihood
    pass
```

### Task 3: Implement Systematic Resampling

```python
def systematic_resample(weights: np.ndarray) -> np.ndarray:
    """
    Perform systematic resampling based on particle weights.

    Algorithm:
    1. Compute cumulative sum of weights
    2. Sample starting point u0 ~ Uniform(0, 1/N)
    3. Select particles at positions u0 + (k-1)/N

    Returns:
        Array of new particle indices
    """
    # YOUR CODE HERE
    pass
```

### Task 4: Implement Map Update

```python
def update_occupancy_grid(map: OccupancyGrid, pose: np.ndarray,
                          ranges: np.ndarray, angles: np.ndarray) -> OccupancyGrid:
    """
    Update occupancy grid with new laser scan.

    Uses inverse sensor model:
    - Cells along ray before hit: p = 0 (free)
    - Cell at hit: p = 1 (occupied)
    - Cells after hit: p = 0 (unknown/no information)
    """
    # YOUR CODE HERE
    pass
```

### Task 5: Run GMapping-Style SLAM

```bash
python solution.py
```

---

## Expected Output

```
Particle Filter SLAM Results:
============================
Number of particles: 100
Final position: (4.87, 3.12, 1.54) m/rad
Position RMSE: 0.31 m
Map size: 20m x 20m (200 x 200 cells)
Resolution: 0.1 m/cell

Output files:
  - slam_trajectory.png
  - slam_map.png
  - particle_convergence.png
```

---

## Verification Checklist

- [ ] Motion model samples realistic poses from odometry
- [ ] Measurement model correctly weights particles
- [ ] Resampling preserves particle diversity
- [ ] Map builds correctly around landmarks
- [ ] Particle cloud converges to true trajectory
- [ ] Handles kidnapped robot scenario

---

## Extensions (Optional)

1. **FastSLAM 2.0**: Add proposal distribution using measurement
2. **Adaptive Resampling**: Only resample when effective N drops
3. **Loop Closure**: Add scan matching for global consistency
4. **Real Data**: Test with ROS bag file data

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| All particles collapse | Increase process noise, reduce measurement noise |
| Particles spread too far | Decrease motion noise, increase resampling threshold |
| Map has artifacts | Check ray casting and inverse sensor model |
| Slow execution | Reduce number of particles, use ray casting optimization |

---

## Files

- `README.md` - This file
- `starter.py` - Template with TODOs to complete
- `solution.py` - Complete working implementation
- `Dockerfile.lab` - Container for reproducibility
