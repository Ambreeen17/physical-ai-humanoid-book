# Chapter 4: Kalman Filters and State Estimation

## Personalized for: Intermediate Learners

**Profile Summary:** Comfortable with linear algebra and probability; wants practical implementation skills alongside theory; prefers Python for experiments.

---

### Quick Navigation

- **Core sections** (essential): 1-5
- **[OPTIONAL] sections**: 6 (particle filter deep-dive)
- **Suggested labs**: Lab B, Lab C

---

## 1. The Kalman Filter Framework

The Kalman Filter is an **optimal recursive estimator** for linear systems with Gaussian noise. It maintains the mean and covariance of a Gaussian belief over the state.

### System Model

```
State evolution:     x_{k} = F_k * x_{k-1} + B_k * u_k + w_k
Measurement:         z_k = H_k * x_k + v_k

Where:
  w_k ~ N(0, Q_k)   process noise
  v_k ~ N(0, R_k)   measurement noise
```

### The Two-Step Process

```
Time Update (Predict):
  x_pred = F * x + B * u
  P_pred = F * P * F^T + Q

Measurement Update (Correct):
  S = H * P_pred * H^T + R
  K = P_pred * H^T * S^{-1}   (Kalman Gain)
  x = x_pred + K * (z - H * x_pred)
  P = (I - K * H) * P_pred
```

### Why Optimal? The Math Behind It

The Kalman Gain minimizes the expected squared error:
```
K* = argmin_K E[||x - x_estimate(K)||^2]
```

Resulting in the closed-form solution above, which yields the **minimum mean squared error (MMSE)** estimate.

---

## 2. Multi-Dimensional Kalman Filter

### Matrix Forms in 2D Tracking

Consider tracking a robot with position (px, py) and velocity (vx, vy):

```python
import numpy as np

# State: [px, py, vx, vy]
state_dim = 4
x = np.zeros(state_dim)

# State transition (constant velocity model)
dt = 0.1  # time step
F = np.array([
    [1, 0, dt, 0],
    [0, 1, 0, dt],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# Measurement matrix (we only measure position)
H = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0]
])
```

### Noise Covariance Matrices

```python
# Process noise: velocity uncertainty
q = 0.1  # velocity noise magnitude
Q = np.array([
    [dt**4/4, 0, dt**3/2, 0],
    [0, dt**4/4, 0, dt**3/2],
    [dt**3/2, 0, dt**2, 0],
    [0, dt**3/2, 0, dt**2]
]) * q**2

# Measurement noise
R = np.array([
    [0.5, 0],
    [0, 0.5]
])  # 0.5m standard deviation per axis
```

---

## 3. Extended Kalman Filter (EKF)

For **non-linear systems**, we linearize using Taylor expansion around the current estimate.

### Linearization

```
f(x) ≈ f(x_hat) + F * (x - x_hat)
h(x) ≈ h(x_hat) + H * (x - x_hat)

Where:
  F = ∂f/∂x  evaluated at x_hat (state Jacobian)
  H = ∂h/∂x  evaluated at x_hat (measurement Jacobian)
```

### EKF Algorithm

```python
def ekf_predict(x, P, F, Q):
    x_pred = F @ x
    P_pred = F @ P @ F.T + Q
    return x_pred, P_pred

def ekf_update(x_pred, P_pred, z, H, R):
    S = H @ P_pred @ H.T + R
    K = P_pred @ H.T @ np.linalg.inv(S)

    y = z - H @ x_pred  # innovation
    x = x_pred + K @ y
    P = (np.eye(len(x)) - K @ H) @ P_pred
    return x, P
```

### Example: 2D Robot with Bearing Measurements

```python
# Bearing measurement: z = atan2(py - ry, px - rx)
# Jacobian: d/dx[atan2] at measurement point
H = np.array([
    [-dy / r**2, dx / r**2, 0, 0]  # derivative w.r.t. position
])
# where r = sqrt(dx^2 + dy^2)
```

---

## 4. Unscented Kalman Filter (UKF)

The UKF avoids Jacobian computation by using **sigma points** to propagate the distribution.

### Sigma Point Selection

```python
def sigma_points(x, P, alpha=1e-3, beta=2, kappa=0):
    n = len(x)
    lam = alpha**2 * (n + kappa) - n

    # 2n + 1 sigma points
    points = np.zeros((2*n + 1, n))
    weights = np.zeros(2*n + 1)

    points[0] = x
    weights[0] = lam / (n + lam)

    sqrt_P = np.linalg.cholesky((n + lam) * P)
    for i in range(n):
        points[i + 1] = x + sqrt_P[:, i]
        points[i + 1 + n] = x - sqrt_P[:, i]
        weights[i + 1] = weights[i + 1 + n] = 1 / (2 * (n + lam))

    return points, weights
```

### UKF Prediction and Update

```python
def ukf_predict(x, P, f, Q, *f_args):
    points, weights = sigma_points(x, P)

    # Propagate sigma points
    points_pred = np.array([f(p, *f_args) for p in points])

    # Weighted mean and covariance
    x_pred = np.sum(weights * points_pred.T, axis=1)
    P_pred = Q.copy()
    for i, w in enumerate(weights):
        diff = points_pred[i] - x_pred
        P_pred += w * np.outer(diff, diff)

    return x_pred, P_pred, points_pred

def ukf_update(x_pred, P_pred, points_pred, z, h, R, *h_args):
    points, weights = sigma_points(x_pred, P_pred)

    # Project sigma points to measurement space
    z_pred = np.array([h(p, *h_args) for p in points])
    z_mean = np.sum(weights * z_pred.T, axis=1)

    # Innovation covariance
    S = R.copy()
    for i, w in enumerate(weights):
        diff = z_pred[i] - z_mean
        S += w * np.outer(diff, diff)

    # Cross-covariance
    Pxz = np.zeros((len(x_pred), len(z_mean)))
    for i, w in enumerate(weights):
        diff_x = points_pred[i] - x_pred
        diff_z = z_pred[i] - z_mean
        Pxz += w * np.outer(diff_x, diff_z)

    # Kalman gain and update
    K = Pxz @ np.linalg.inv(S)
    y = z - z_mean
    x = x_pred + K @ y
    P = P_pred - K @ S @ K.T

    return x, P
```

### EKF vs UKF Comparison

| Aspect | EKF | UKF |
|--------|-----|-----|
| Jacobian required | Yes | No |
| Accuracy | First-order | Second-order (typically) |
| Computational cost | O(n^2) | O(n^3) for Cholesky |
| Handles severe non-linearity | Poor | Good |

---

## 5. Particle Filters

For **highly non-Gaussian** or **multi-modal** distributions, particle filters use sequential Monte Carlo methods.

### Basic Particle Filter

```python
class ParticleFilter:
    def __init__(self, n_particles, state_dim):
        self.n = n_particles
        self.particles = np.random.randn(n, state_dim)
        self.weights = np.ones(n) / n

    def predict(self, motion_model, Q, *args):
        noise = np.random.randn(self.n, self.particles.shape[1]) @ np.sqrt(Q)
        self.particles = motion_model(self.particles, *args) + noise

    def update(self, measurement_model, R, z):
        likelihoods = np.zeros(self.n)
        for i in range(self.n):
            likelihoods[i] = self._gaussian_pdf(
                z - measurement_model(self.particles[i]), R)
        self.weights *= likelihoods
        self.weights /= np.sum(self.weights)

    def resample(self):
        indices = np.random.choice(
            self.n, self.n, p=self.weights, replace=True)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.n) / self.n

    def estimate(self):
        return np.sum(self.weights * self.particles.T, axis=1)

    def _gaussian_pdf(self, residual, R):
        return np.exp(-0.5 * residual @ np.linalg.inv(R) @ residual) / \
               np.sqrt((2 * np.pi)**len(residual) * np.linalg.det(R))
```

### Resampling Strategies

```python
# Systematic resampling (more efficient)
def systematic_resample(weights):
    n = len(weights)
    cumsum = np.cumsum(weights)
    positions = (np.arange(n) + np.random.random()) / n

    indices = np.zeros(n, dtype=int)
    j = 0
    for i in range(n):
        while cumsum[j] < positions[i]:
            j += 1
        indices[i] = j
    return indices
```

---

## 6. [OPTIONAL] Advanced Particle Filter Topics

*This section covers sophisticated techniques for production systems.*

### Rao-Blackwellized Particle Filter

Marginalizes analytically tractable parts of the state:
```
p(x_{1:k}, theta | z_{1:k}) = p(theta | z_{1:k}) * p(x_{1:k} | theta, z_{1:k})
```
Useful when some state variables are linear-Gaussian.

### FastSLAM

Uses particles to track robot pose, with EKF updates for each landmark:
```
p(s_{1:k}, m | z_{1:k}, u_{1:k}) ≈ sum_i [delta(s_{1:k}^i) * prod_j p(m_j | s_{1:k}^i, z_{1:k})]
```

---

### Alternative Lab Options

1. **Lab A: 2D Robot Tracking with GPS** - Implement a 2D Kalman filter to track a robot using noisy GPS measurements. Visualize uncertainty ellipses and compare against dead reckoning.

2. **Lab B: EKF vs UKF on Circular Trajectory** - Track a robot moving in a circle using bearing-only measurements. Compare EKF and UKF accuracy as non-linearity increases.

3. **Lab C: Multi-Sensor Fusion** - Fuse GPS + IMU data for robust robot localization. Implement an indirect Kalman filter (error-state formulation).

4. **Lab D: Particle Filter for Robot Localization** - Implement AMCL-style particle filter for global localization in a known map. Handle kidnapped robot scenario.

---

### Optional Deep-Dives

- **Indirect/Error-State Kalman Filter** - Tracks perturbations from a nominal trajectory rather than absolute states. Essential for systems with significant rotations.
- **Invariant Kalman Filter** - Preserves geometric structure for Lie group systems (SO(3), SE(3)).

---

### Recommended Sequence

1. **Now:** Lab A (2D KF) builds implementation skills
2. **Next:** Lab B (EKF vs UKF) compares approximation methods
3. **After:** Lab C (sensor fusion) for practical applications
4. **Future:** Lab D (particle filter) for non-Gaussian scenarios

---

### Quick Reference

```
KF: Linear systems, Gaussian noise
     - O(n^3) predict, O(n^2) update

EKF: Non-linear, differentiable
     - Requires Jacobians
     - Linearization errors

UKF: Non-linear, non-differentiable
     - No Jacobians needed
     - Better for severe non-linearity

PF: Multi-modal, arbitrary distributions
     - Scales with particles (not state dim)
     - Prone to sample degeneracy
```
