# Lab 4.2: EKF Localization - Landmark Detection and Nonlinear Models

## Learning Objectives

By the end of this lab, you will be able to:
- Understand Extended Kalman Filter (EKF) for nonlinear state estimation
- Implement range-bearing landmark measurement models
- Linearize nonlinear motion and observation models using Jacobians
- Apply EKF to 2D robot localization with known landmarks
- Handle data association for multiple landmark measurements

## Prerequisites

- Completed Lab 4.1 (1D Kalman Filter)
- Python 3.8+
- numpy, matplotlib
- Understanding of basic trigonometry and coordinate transformations

## Time Estimate: 60 minutes

---

## Theory Overview

### Why Extended Kalman Filter?

Standard Kalman Filter only works for **linear** systems:
```
x_{k+1} = F * x_k + B * u_k     (linear state transition)
z_k = H * x_k                   (linear observation)
```

Real robots have **nonlinear** dynamics:
- Motion: velocity controls affect position via trigonometric functions
- Sensing: landmarks give range-bearing measurements

EKF solves this by **linearizing** around the current estimate using Taylor series.

### Linearization via Jacobians

For a nonlinear function f(x):
```
f(x) ≈ f(x0) + J_f(x0) * (x - x0)
```

Where J_f is the **Jacobian matrix** of partial derivatives.

### Robot State and Motion Model

**State**: `x = [x, y, theta]^T` (2D position + heading)

**Motion Model** (unicycle):
```
x_{k} = x_{k-1} + v * cos(theta) * dt
y_{k} = y_{k-1} + v * sin(theta) * dt
theta_{k} = theta_{k-1} + omega * dt
```

**Motion Jacobian F_motion**:
```
[[1, 0, -v*sin(theta)*dt],
 [0, 1,  v*cos(theta)*dt],
 [0, 0,         1        ]]
```

### Observation Model

For a landmark at `(lx, ly)` observed from robot at `(rx, ry, theta)`:
```
range = sqrt((lx - rx)^2 + (ly - ry)^2)
bearing = atan2(ly - ry, lx - rx) - theta
```

**Observation Jacobian H** (partial derivatives w.r.t. robot state):
```
d_range/dx = -(lx - rx) / range
d_range/dy = -(ly - ry) / range
d_range/dtheta = 0

d_bearing/dx = -(ly - ry) / range^2
d_bearing/dy = (lx - rx) / range^2
d_bearing/dtheta = -1
```

---

## Lab Setup

### Installation

```bash
cd lab-4.2-ekf-localization
pip install numpy matplotlib scipy
```

---

## Exercise Tasks

### Task 1: Implement Motion Model

Complete the motion model in `starter.py`:

```python
def motion_model(state: np.ndarray, control: np.ndarray,
                 dt: float) -> np.ndarray:
    """
    Unicycle motion model.

    Args:
        state: Current state [x, y, theta]
        control: Control input [v, omega]
        dt: Time step

    Returns:
        Predicted state after applying controls
    """
    x, y, theta = state
    v, omega = control

    # YOUR CODE HERE
    # Compute new x, y, theta using trigonometric motion model
    # Handle omega near zero to avoid division issues
    pass
```

### Task 2: Compute Motion Jacobian

```python
def compute_motion_jacobian(state: np.ndarray, control: np.ndarray,
                            dt: float) -> np.ndarray:
    """
    Compute Jacobian of motion model w.r.t. state.

    Returns:
        3x3 Jacobian matrix F where:
        F[i,j] = d(state_new[i]) / d(state[j])
    """
    x, y, theta = state
    v, omega = control

    # YOUR CODE HERE
    # Compute partial derivatives for each element
    pass
```

### Task 3: Implement Observation Model

```python
def observation_model(state: np.ndarray, landmark: np.ndarray) -> np.ndarray:
    """
    Convert landmark position to range-bearing measurement.

    Args:
        state: Robot state [x, y, theta]
        landmark: Landmark position [lx, ly]

    Returns:
        Measurement [range, bearing]
    """
    # YOUR CODE HERE
    # Compute range and bearing from robot to landmark
    pass
```

### Task 4: Implement EKF Update

Complete the update step handling range-bearing measurements:

```python
def ekf_update(x_pred: np.ndarray, P_pred: np.ndarray,
               z: np.ndarray, landmark: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    EKF update step with range-bearing measurement.

    Args:
        x_pred: Predicted state
        P_pred: Predicted covariance
        z: Measurement [range, bearing]
        landmark: Landmark position [lx, ly]

    Returns:
        Updated state and covariance
    """
    # YOUR CODE HERE
    # 1. Compute expected measurement (h(x))
    # 2. Compute Jacobian H
    # 3. Compute innovation and its covariance
    # 4. Compute Kalman gain
    # 5. Update state and covariance
    pass
```

### Task 5: Run EKF Localization

```bash
python solution.py
```

This will simulate a robot moving in a environment with landmarks and visualize the EKF localization results.

---

## Key Concepts

### Data Association

When multiple landmarks are visible, we must decide which measurement corresponds to which landmark:

1. **Nearest Neighbor**: Associate measurement with closest landmark (in innovation space)
2. **Mahalanobis Distance**: Account for uncertainty in the match
3. **Joint Compatibility**: Consider multiple measurements together

### Process Noise Tuning

For unicycle model, process noise typically models:
- Wheel slippage
- Uneven terrain
- Control inaccuracies

```python
Q = np.diag([0.1, 0.1, 0.05])**2  # [x_noise, y_noise, theta_noise]
```

### Measurement Noise

```python
R = np.diag([0.5, 0.1])**2  # [range_noise, bearing_noise]
```

---

## Expected Output

Running `solution.py` should show:
1. Robot trajectory with true and estimated positions
2. Landmark positions and observations
3. Uncertainty ellipses at key points
4. Console output with:
   ```
   Final position: (4.95, 3.02) m
   Final heading: 1.57 rad
   Position RMSE: 0.23 m
   Heading RMSE: 0.05 rad
   ```

---

## Verification Checklist

- [ ] Motion model correctly handles unicycle dynamics
- [ ] Jacobians correctly computed using chain rule
- [ ] Observation model correctly converts Cartesian to polar
- [ ] EKF update produces valid (positive definite) covariance
- [ ] Filter converges as robot moves and observes landmarks
- [ ] Uncertainty grows in unobserved directions

---

## Extensions (Optional)

1. **Unscented Kalman Filter (UKF)**: Implement UKF and compare with EKF
2. **Unknown Landmark Init**: Implement SLAM to initialize new landmarks
3. **Robust Data Association**: Implement MHT (Multiple Hypothesis Tracking)
4. **RGB-D Integration**: Add depth sensor measurements

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Filter diverges | Check Jacobian signs; increase R |
| Uncertainty too small | Increase Q (process noise) |
| Large heading errors | Check bearing angle wrapping (-pi to pi) |
| NaN in computations | Check for zero division in range computation |

---

## Files

- `README.md` - This file
- `starter.py` - Template with TODOs to complete
- `solution.py` - Complete working implementation
- `Dockerfile.lab` - Container for reproducibility
