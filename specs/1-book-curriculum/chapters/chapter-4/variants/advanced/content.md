# Chapter 4: Advanced State Estimation Techniques

## Personalized for: Advanced Learners

**Profile Summary:** Strong linear algebra background; familiar with KF/EKF; ready for information-form filters, incremental smoothing, and multi-state constraints.

---

### Quick Navigation

- **Core sections** (essential): 1-3, 5-6
- **[OPTIONAL] sections**: 4 (remedial matrix KF recap)
- **Suggested labs**: Lab C (MSCKF), Lab D (iSAM2)

---

## 1. Information Filter: The Dual Form

While the Kalman Filter maintains the **covariance matrix (P)**, the Information Filter maintains its inverse: the **information matrix (Omega = P^{-1})** and **information vector (xi)**.

### Information Filter Equations

**Prediction:**
```
Omega_pred = (F * Omega^{-1} * F^T + Q)^{-1}
xi_pred = Omega_pred * F * Omega^{-1} * x
```

**Update:**
```
S = H * Omega_pred^{-1} * H^T + R
K = Omega_pred^{-1} * H^T * S^{-1}

Omega_new = Omega_pred + H^T * R^{-1} * H
xi_new = xi_pred + H^T * R^{-1} * z
```

### When Information Form Shines

1. **Sparse updates** - Adding measurements only affects local information matrix entries
2. **Many measurements** - O(N^3) vs O(N^2) per update for dense systems
3. **Map merging** - Information matrices add linearly for pose graph fusion

### Computational Tradeoffs

| Operation | Covariance Form | Information Form |
|-----------|-----------------|------------------|
| Predict | O(n^3) | O(n^3) |
| Update (one measurement) | O(n^3) | O(m^2) where m << n |
| Measurement marginalization | Hard | Easy |

### The Bir-Euler Connection

Information matrices relate to the log-partition function:
```
I(x) = -log p(z|x) + const
```

This connects to **maximum entropy** distributions and **dual representations** in exponential families.

---

## 2. iSAM2: Incremental Smoothing and Mapping

iSAM2 provides **real-time incremental inference** for graph-based SLAM using the **Bayes Tree** data structure.

### Bayes Tree Structure

```
Bayes Tree = Chow-Liu tree over the information matrix
- Parent pointers encode conditional independencies
- Cliques capture coupled variables
- Enables O(k^2) updates for k changes
```

### The QR-Based Update Trick

Instead of recomputing the full factorization, iSAM2 tracks **column updates**:
```
Let A be the linearized system: Ax = b

When A changes by delta_A:
- Only affected columns re-factorized
- Usegivens rotations for numerical stability
- Incremental QR: R = R + delta_R
```

### Variable Reordering andfill-in

The **approximate minimum degree (AMD)** ordering heuristic minimizes fill-in during Cholesky factorization:

```
Original ordering:          AMD ordering:
[A B C]  →  fill-in: X      [A C B]  →  no fill-in
[D E F]                      [D F E]
[G H I]                      [G I H]
```

### Connection to Gaussian Belief Propagation

iSAM2's Bayes Tree is equivalent to **Loopy Belief Propagation** on a chordal graph. The clique messages are:
```
Message m_{i→j}(x_j) = ∫ p(x_i) * ∏_{k∈N(i)\j} m_{k→i}(x_i) dx_i
```

---

## 3. Multi-State Constraint Kalman Filter (MSCKF)

MSCKF performs **visual-inertial odometry** by tracking features across multiple camera poses withoutFeature initialization.

### MSCKF vs EKF-SLAM Comparison

| Aspect | EKF-SLAM | MSCKF |
|--------|----------|-------|
| Feature state | Included in state vector | Marginalized out |
| Number of features | Limited (~100) | Can handle 1000+ |
| Computational cost | O(n^3) where n includes features | O(m^2) for m camera poses |
| Consistency | Prone to linearization errors | Better consistency |

### MSCKF Algorithm Steps

1. **Propagate IMU state** using noise-driven dynamics
2. **Add camera poses** to state when sliding window fills
3. **Perform EKF update** for each feature track:
   - Formulate constraints from all observations
   - Jacobian linearization around current estimates
   - Update only camera poses (feature marginalized)
4. **Prune old poses** and their measurements

### The MSCKF Update Derivation

For a feature observed from N camera poses:

**Measurement model:**
```
z_ij = h(x_i, f_j) + v_ij
```

**Stacked linearized system:**
```
H_ij * delta_x_i + G_ij * delta_f_j = r_ij + v_ij
```

**Marginalize feature:**
```
A = H_ij * P_ii * H_ij^T + R_ij
B = H_ij * P_ii * G_ij^T
C = G_ij * P_ff * G_ij^T + I * sigma_f^2

Effective update: (H_ij * P_ii * H_ij^T - B * C^{-1} * B^T) * delta_x_i = residual
```

### Observability and Consistency

MSCKF preserves **observability properties** by using the **first-estimate Jacobian** formulation, preventing spurious information gain that plagues standard EKF-SLAM.

The unobservable directions are:
- 3 DOF global translation
- 1 DOF rotation about gravity

---

## 4. [REMEDIAL] 1D Kalman Filter Basics

*Skippable - basic material for review*

In 1D, the Kalman filter simplifies to scalar operations:

```
Prediction:
  x_pred = F * x + B * u
  P_pred = F * P * F^T + Q

Update:
  K = P_pred * H^T * (H * P_pred * H^T + R)^{-1}
  x = x_pred + K * (z - H * x_pred)
  P = (I - K * H) * P_pred
```

---

## 5. Unified Framework: The Square-Root Filters

All Kalman variants can be expressed using **matrix square roots**:

```
Covariance form:    P = U * U^T  (Cholesky factor)
Information form:   Omega = L^T * L  (inverse Cholesky)

Square-Root Filter benefits:
- Numerical stability (positive semi-definiteness guaranteed)
- Smoother convergence
- Better conditioned for ill-conditioned problems
```

### Joseph Form Update (Numerically Stable)

For the update step with guaranteed symmetry and positive semi-definiteness:

```
U_new = (I - K * H) * U

Or using QR decomposition:
[K; U] = qr([R^{1/2} * H; U])
K = K(1:n, :)' / R
```

---

## 6. Current Research Frontiers

### Information Filter Extensions
- **Sparse iSAM2** - Exploiting structure in large-scale SLAM graphs
- **Incremental SVD** for information matrix updates
- **Parallelization** using Gauss-Seidel relaxation

### MSCKF Enhancements
- **MSCKF 2.0** - Robust feature initialization and outlier rejection
- **Sliding window bundle adjustment** equivalence proofs
- **Deep learning integration** for learned measurement models

### Open Problems
1. **Non-Gaussian noise** - Handling heavy-tailed sensor errors
2. **Continuous-time** trajectory estimation
3. **Decentralized** multi-robot fusion with privacy

---

### Alternative Lab Options

1. **Lab C: MSCKF Implementation** - Implement a visual-inertial MSCKF with synthetic IMU and camera data. Compare trajectory accuracy against ground truth at various noise levels.

2. **Lab D: iSAM2 on Toro Dataset** - Implement iSAM2 on the TORO benchmark. Compare against g2o and explore the effect of different variable ordering strategies.

3. **Lab E: Square-Root Filter Comparison** - Implement both covariance and square-root forms. Measure numerical conditioning on ill-conditioned problems (e.g., large state dimension, small measurement rates).

4. **Lab F: Information Filter Fusion** - Implement a distributed SLAM system where two robots fuse their local information matrices via an information consensus protocol.

---

### Optional Deep-Dives

- **Rauch-Tung-Striebel (RTS) Smoother** - The Rauch-Tung-Striebel (RTS) smoother provides a backward pass to improve state estimates using all measurements. It computes the optimal smoothed estimate by combining forward-filtered states with backward-predicted trajectories. The key insight is that smoothing incorporates future information, reducing uncertainty compared to filtering alone. This approach is particularly valuable in batch processing scenarios like map optimization and trajectory reconstruction.

- **Error-State Kalman Filters** - Error-state (or indirect) Kalman filters track perturbations from a nominal trajectory rather than absolute states. This formulation linearizes around the known nominal trajectory, improving numerical stability and separating large motions from small corrections. The technique is essential for systems with significant rotations where standard Kalman filters can suffer from singularity issues.

---

### Recommended Sequence

1. **Now:** Lab C (MSCKF) to ground theory in implementation
2. **Parallel:** Lab D (iSAM2) for graph-based perspective
3. **Next:** Explore RTS smoother for smoothing applications
4. **Advanced:** Investigate continuous-time trajectory estimation

---

### Key Mathematical References

```
Information Filter:
  Omega = P^{-1}, xi = Omega * x

iSAM2:
  Bayes Tree = Chow-Liu tree over Omega
  Update cost: O(k^2) for k changes

MSCKF:
  Feature marginalized update
  Clamp-based outlier rejection
  First-estimate Jacobian for consistency
```
