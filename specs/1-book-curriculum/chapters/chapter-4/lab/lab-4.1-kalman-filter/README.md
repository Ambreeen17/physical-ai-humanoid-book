# Lab 4.1: 1D Kalman Filter - Constant Velocity Model

## Learning Objectives

By the end of this lab, you will be able to:
- Understand the predict-update cycle of Kalman filters
- Implement a constant velocity motion model
- Apply Kalman filtering to 1D position/velocity estimation
- Tune process and measurement noise covariance matrices
- Visualize filter performance and uncertainty bounds

## Prerequisites

- Python 3.8+
- numpy
- matplotlib
- Basic understanding of linear algebra (matrices, vectors)

## Time Estimate: 45 minutes

---

## Theory Overview

### Kalman Filter Basics

The Kalman filter is an optimal recursive estimator for linear systems with Gaussian noise. It maintains:
- **State estimate**: `x_hat` - our best guess of the true state
- **Uncertainty**: `P` - covariance matrix representing our confidence

### The Two-Step Cycle

**1. Prediction (Predict)**
```
x_pred = F * x_prev + B * u
P_pred = F * P_prev * F^T + Q
```

**2. Update (Correct)**
```
K = P_pred * H^T * (H * P_pred * H^T + R)^(-1)
x_hat = x_pred + K * (z - H * x_pred)
P_hat = (I - K * H) * P_pred
```

### 1D Constant Velocity Model

For a 1D system tracking position and velocity:
- **State vector**: `x = [position, velocity]^T`
- **State transition**:
  ```
  x_k = F * x_{k-1}
  F = [[1, dt],
       [0,  1]]
  ```
- **Observation**: We measure position directly, so `H = [1, 0]`

---

## Lab Setup

### Installation

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install numpy matplotlib
```

### Verify Installation

```bash
python -c "import numpy; import matplotlib; print('Dependencies OK')"
```

---

## Exercise Tasks

### Task 1: Implement the Prediction Step

Complete the `predict` function in `starter.py`:

```python
def predict(self, x_prev: np.ndarray, P_prev: np.ndarray,
            dt: float, u: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """
    Predict step of Kalman filter.

    Args:
        x_prev: Previous state estimate [position, velocity]
        P_prev: Previous covariance matrix
        dt: Time step
        u: Control input (optional, for this model it's 0)

    Returns:
        x_pred: Predicted state
        P_pred: Predicted covariance
    """
    # YOUR CODE HERE
    # 1. Compute state transition matrix F based on dt
    # 2. Compute predicted state: x_pred = F @ x_prev + B * u
    # 3. Compute predicted covariance: P_pred = F @ P_prev @ F.T + Q
    pass
```

### Task 2: Implement the Update Step

Complete the `update` function:

```python
def update(self, x_pred: np.ndarray, P_pred: np.ndarray,
           z: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Update step of Kalman filter.

    Args:
        x_pred: Predicted state from predict step
        P_pred: Predicted covariance from predict step
        z: Measured position

    Returns:
        x_hat: Updated (posterior) state estimate
        P_hat: Updated (posterior) covariance
    """
    # YOUR CODE HERE
    # 1. Compute Kalman gain: K = P_pred * H.T @ inv(H @ P_pred @ H.T + R)
    # 2. Compute innovation: y = z - H @ x_pred
    # 3. Update state: x_hat = x_pred + K @ y
    # 4. Update covariance: P_hat = (I - K @ H) @ P_pred
    pass
```

### Task 3: Run and Analyze

```bash
cd lab-4.1-kalman-filter
python solution.py
```

This will:
- Generate synthetic noisy position measurements
- Apply the Kalman filter
- Plot true vs estimated position with uncertainty bounds
- Print RMSE and other metrics

---

## Key Concepts to Understand

### Process Noise Covariance (Q)

`Q` models uncertainty in our motion model. For constant velocity:
```python
Q = [[dt^4/4, dt^3/2],
     [dt^3/2, dt^2]] * sigma_a^2
```

Where `sigma_a` is the standard deviation of acceleration noise.

### Measurement Noise Covariance (R)

`R` models sensor noise. Since we measure position:
```python
R = sigma_measurement^2
```

### Tuning Tips

- **High Q (process noise)**: Filter trusts measurements more, responds quickly to changes
- **Low Q (process noise)**: Filter trusts model more, smooths out noise
- **High R (measurement noise)**: Filter is skeptical of measurements, smooths more
- **Low R (measurement noise)**: Filter trusts measurements, responds quickly

---

## Expected Output

When running `solution.py`, you should see:
1. A plot comparing true position, measurements, and Kalman estimate
2. Uncertainty bands (2-sigma) around the estimate
3. Console output with metrics like:
   ```
   Final position estimate: 95.2 m
   Final velocity estimate: 1.01 m/s
   RMSE: 0.45 m
   ```

---

## Verification Checklist

- [ ] `predict` function correctly computes F matrix for any dt
- [ ] `update` function correctly computes Kalman gain
- [ ] State and covariance updates use proper matrix operations
- [ ] Filter converges to true state within reasonable time
- [ ] Uncertainty decreases when measurements are consistent

---

## Extensions (Optional)

1. **Constant Acceleration Model**: Extend state to [position, velocity, acceleration]
2. **Adaptive Q/R**: Automatically estimate noise covariances from data
3. **Multiple Sensors**: Fuse GPS and IMU measurements

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Filter diverges | Increase R (measurement noise) or decrease Q |
| Filter too slow | Decrease R or increase Q |
| Position estimate grows without bound | Check F matrix implementation |
| Covariance becomes negative | Use `np.linalg.solve` instead of `inv()` for numerical stability |

---

## Files

- `README.md` - This file
- `starter.py` - Template with TODOs to complete
- `solution.py` - Complete working implementation
- `Dockerfile.lab` - Container for reproducibility
