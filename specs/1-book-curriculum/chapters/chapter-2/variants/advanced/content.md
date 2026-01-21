# Chapter 2: Kinematics and Dynamics of Robot Manipulators

**Variant**: Advanced
**Difficulty Score**: 8/10
**Estimated Reading Time**: 180 minutes

**Chapter Overview**: This chapter develops the mathematical foundations for describing and predicting robot motion at a graduate-level depth. We begin with the geometry of robot configurations and advance through forward kinematics using both Denavit-Hartenberg and modern screw theory approaches. Inverse kinematics is treated with both analytical and numerical methods, including global optimization perspectives. The second half introduces Lagrangian dynamics with full derivations, followed by the Newton-Euler recursive formulation for real-time computation. We conclude with screw theory representation, the Product of Exponentials formula, and optimal control formulations for trajectory planning.

**Learning Objectives**: After completing this chapter, you will be able to derive forward and inverse kinematics using multiple formulations (DH, PoE, screw theory), implement the Newton-Euler algorithm for O(n) dynamics computation, analyze singularities using the Jacobian and understand their topological implications, derive Lagrangian dynamics equations from first principles, apply screw theory to robot kinematics, formulate optimal control problems for trajectory generation, and critically evaluate research literature on contact-rich manipulation.

**Prerequisites**: Chapter 1, multivariable calculus, linear algebra at the level of Strang's "Linear Algebra and Its Applications," differential equations, graduate-level programming in Python/C++, and familiarity with optimization theory.

---

## 2.1 Joint Spaces and Configuration Spaces

### Topological Foundations

The mathematical study of robot configurations requires understanding the topological structure of configuration spaces. For an n-DOF robot with revolute joints, the configuration space is the n-dimensional torus $T^n = S^1 \times S^1 \times \cdots \times S^1$, embedded in $\mathbb{R}^{2n}$ through the exponential map $\theta_i \mapsto (\cos\theta_i, \sin\theta_i)$.

**Fundamental Groups and Homotopy**: The fundamental group $\pi_1(T^n) \cong \mathbb{Z}^n$ captures the winding numbers of closed loops in configuration space. This has practical implications for motion planning—trajectories can be classified by how many times they wind around each joint's circular topology.

### Configuration Space Obstacles

When we introduce collision constraints, the configuration space becomes $C_{free} = C \setminus C_{obs}$, where $C_{obs}$ is the configuration space obstacle region. For rigid body obstacles, $C_{obs}$ is constructed via configuration space sweep:

$$
C_{obs} = \{ q \in C \mid A(q) \cap B \neq \emptyset \}
$$

where $A(q)$ is the robot in configuration $q$ and $B$ is the obstacle region.

> **Research Note**: The complexity of configuration space obstacles grows exponentially with DOF. For 6-DOF arms, exact $C_{obs}$ computation is intractable; sampling-based methods (PRM*, RRT*) provide probabilistic completeness guarantees. See Kavraki et al. (1996) and Karaman & Frazzoli (2011).

### Workspace Analysis: Reachable and Dexterous Workspace

The **reachable workspace** $\mathcal{W}_R$ is the set of all end-effector positions reachable with at least one orientation:

$$
\mathcal{W}_R = \{ p \in \mathbb{R}^3 \mid \exists q \in C, \exists R \in SO(3), \ FK(q) = (R, p) \}
$$

The **dexterous workspace** $\mathcal{W}_D$ requires reachability with arbitrary orientation:

$$
\mathcal{W}_D = \{ p \in \mathbb{R}^3 \mid \forall R \in SO(3), \exists q \in C, \ FK(q) = (R, p) \}
$$

For the Unitree G1's 6-DOF arm, $\mathcal{W}_D \subsetneq \mathcal{W}_R$, with $\mathcal{W}_D$ typically being a smaller region near the robot's torso due to wrist orientation limitations at workspace boundaries.

### Jacobian Analysis and Singularities

The geometric Jacobian $J(q)$ maps joint velocities to end-effector twist:

$$
V_{ee} = \begin{bmatrix} v_{ee} \\ \omega_{ee} \end{bmatrix} = J(q) \dot{q}
$$

Singularities occur when $\text{rank}(J(q)) < 6$. The **singular value decomposition** provides deeper insight:

$$
J(q) = U \Sigma V^T = U \begin{bmatrix} \Sigma_r & 0 \\ 0 & 0 \end{bmatrix} V^T
$$

where $\Sigma_r$ contains the non-zero singular values $\sigma_1 \geq \sigma_2 \geq \cdots \geq \sigma_r > 0$. The null space dimension equals $6 - \text{rank}(J)$.

**Dexterity Metrics**: The **manipulability index** (Yoshikawa, 1985) quantifies manipulability:

$$
\mu(q) = \sqrt{\det(J(q)J(q)^T)}
$$

At singularities, $\mu(q) = 0$. The **isotropy index** relates worst-case to best-case manipulability:

$$
\kappa = \frac{\sigma_{\min}}{\sigma_{\max}}
$$

### Summary and Key Takeaways

- $C$-space topology (torus for revolute joints) fundamentally constrains motion planning
- Configuration space obstacles have exponential complexity in DOF
- Workspace decomposition into reachable and dexterous regions guides task planning
- Jacobian SVD analysis reveals singularity structure and manipulability

---

## 2.2 Forward Kinematics

### Homogeneous Transformations in SE(3)

The special Euclidean group SE(3) comprises all rigid body transformations:

$$
SE(3) = \left\{ \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} \mid R \in SO(3), t \in \mathbb{R}^3 \right\}
$$

$SO(3)$ is the special orthogonal group of 3x3 rotation matrices with determinant +1.

### Denavit-Hartenberg Convention: Detailed Treatment

The DH convention (Denavit & Hartenberg, 1955) assigns frames using four geometric parameters:

| Parameter | Definition | Physical Interpretation |
|-----------|------------|------------------------|
| $a_i$ | Link length | Distance from $z_{i-1}$ to $z_i$ along $x_i$ |
| $d_i$ | Link offset | Distance from $x_{i-1}$ to $x_i$ along $z_{i-1}$ |
| $\theta_i$ | Joint angle | Rotation about $z_{i-1}$ from $x_{i-1}$ to $x_i$ |
| $\alpha_i$ | Link twist | Rotation about $x_i$ from $z_{i-1}$ to $z_i$ |

**Standard vs. Modified DH**: The standard convention attaches frame $i$ to the output of joint $i$, while the modified convention attaches it to the input. This affects the transformation multiplication order:

- **Standard DH**: $T_i^{i-1} = Rot(z, \theta_i) \cdot Trans(z, d_i) \cdot Trans(x, a_i) \cdot Rot(x, \alpha_i)$
- **Modified DH**: $T_i^{i-1} = Rot(x, \alpha_i) \cdot Trans(x, a_i) \cdot Rot(z, \theta_i) \cdot Trans(z, d_i)$

### Screw Theory and the Product of Exponentials

An alternative formulation represents each joint as a screw motion:

$$
T = e^{[S_1]\theta_1} e^{[S_2]\theta_2} \cdots e^{[S_n]\theta_n} T(0)
$$

where $[S]$ is the se(3) matrix representation of screw $S$, and $T(0)$ is the home configuration.

**Screw Representation**: A screw $S = (s, s_0)$ consists of:
- Direction $s \in \mathbb{R}^3$ (unit vector along joint axis)
- Moment $s_0 \in \mathbb{R}^3$ (moment about origin)

The se(3) matrix is:

$$
[S] = \begin{bmatrix} [s]_\times & s_0 \\ 0 & 0 \end{bmatrix}
$$

where $[s]_\times$ is the skew-symmetric cross-product matrix.

**Exponential Map**: For rotation by $\theta$ about axis $s$ through point $p$:

$$
e^{[S]\theta} = \begin{bmatrix} I + \sin\theta [s]_\times + (1-\cos\theta)[s]_\times^2 & (I\theta + (1-\cos\theta)[s]_\times + (\theta - \sin\theta)[s]_\times^2) p \\ 0 & 1 \end{bmatrix}
$$

### Forward Kinematics Implementation for Unitree G1

```python
import numpy as np
from scipy.linalg import expm

def skew(v):
    """Skew-symmetric matrix from vector."""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])

def screw_to_se3(s, theta, p=None):
    """
    Convert screw and angle to SE(3) exponential.

    S = (s, s0) where s is direction, s0 is moment.
    """
    if p is None:
        s0 = np.cross(s, np.zeros(3))  # Through origin
    else:
        s0 = np.cross(s, p)

    S = np.zeros((4, 4))
    S[:3, :3] = skew(s)
    S[:3, 3] = s0

    return expm(theta * S)

def forward_kinematics_poe(S, theta, T_home):
    """
    Product of Exponentials forward kinematics.

    S: n x 6 array of screw coordinates (one per row)
    theta: n x 1 array of joint angles
    T_home: 4x4 home configuration
    """
    T = T_home.copy()
    for i in range(len(theta)):
        S_i = np.zeros((4, 4))
        S_i[:3, :3] = skew(S[i, :3])
        S_i[:3, 3] = S[i, 3:]
        T = T @ expm(theta[i] * S_i)
    return T
```

### Comparative Analysis: DH vs. PoE

| Aspect | Denavit-Hartenberg | Product of Exponentials |
|--------|-------------------|------------------------|
| Frame attachment | Required at each joint | Single home frame only |
| Parameter count | 4n | 6n |
| Singularity handling | Explicit DH parameters | Implicit in screw definitions |
| Calibration sensitivity | High | Lower |
| Research applications | Standard robotics | Modern geometric control |

> **Research Note**: Murray et al. (1994) established the mathematical foundations of screw theory in robotics. Recent work by Martin &配置空間分析 (2020) demonstrates advantages of PoE for calibration-agnostic kinematic modeling.

### Summary and Key Takeaways

- SE(3) provides the mathematical group structure for rigid transformations
- DH convention systematically assigns frames with 4 parameters per joint
- Screw theory and PoE offer calibration-robust alternatives
- Choice of formulation depends on application requirements

---

## 2.3 Inverse Kinematics

### Problem Formulation

The inverse kinematics problem seeks $q$ such that $FK(q) = T_d$. This is solving:

$$
T_1(\theta_1) T_2(\theta_2) \cdots T_n(\theta_n) = T_d
$$

Nonlinearity and non-uniqueness make this challenging.

### Analytical Solutions: Geometric Approach

For the 2-DOF planar arm, we derive closed-form solutions using the law of cosines:

Given target $(x, y)$ and link lengths $L_1, L_2$:

$$
\cos\theta_2 = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}
$$

This yields two solutions $\theta_2 = \pm \arccos(\cdot)$, corresponding to elbow-up and elbow-down configurations. Then:

$$
\theta_1 = \atan2(y, x) - \atan2(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2)
$$

### Numerical Methods: Optimization Perspective

For 6-DOF arms, we formulate IK as optimization:

$$
\min_q \| FK(q) - T_d \|^2_W
$$

subject to joint limits $q_{min} \leq q \leq q_{max}$.

**Levenberg-Marquardt Damped Least Squares**:

$$
\Delta q = (J^T W J + \lambda^2 I)^{-1} J^T W e
$$

where $e$ is the pose error and $\lambda$ is the damping parameter.

### Advanced Topics: Global and Optimization-Based IK

**Continuation Methods**: Parameterize the IK problem through homotopy to find multiple solutions (Diaz-Calderon et al., 2005).

**Bayesian Optimization**: For high-DOF systems, BO-IK uses probabilistic models to guide search (Berkenkamp et al., 2016).

**Neural IK Networks**: Learning-based approaches (e.g., Tsai & Docan, 2021) achieve real-time performance but require training data.

### Implementation: Damped Least Squares

```python
import numpy as np
from scipy.linalg import solve_sylvester

class NumericalIK:
    """
    Damped Least Squares inverse kinematics with null-space projection.
    """

    def __init__(self, fk_func, n_joints, lambda_damping=0.1):
        self.fk_func = fk_func
        self.n = n_joints
        self.lambda_damping = lambda_damping
        self.joint_limits = np.array([[-np.pi, np.pi]] * n_joints)

    def compute_jacobian(self, q, delta=1e-6):
        """Numerical Jacobian using central differences."""
        J = np.zeros((6, self.n))
        T0, _, _ = self.fk_func(q)
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]

        for i in range(self.n):
            q_plus = q.copy()
            q_plus[i] += delta
            q_minus = q.copy()
            q_minus[i] -= delta

            T_plus, _, _ = self.fk_func(q_plus)
            T_minus, _, _ = self.fk_func(q_minus)

            # Linear velocity
            J[:3, i] = (T_plus[:3, 3] - T_minus[:3, 3]) / (2 * delta)

            # Angular velocity (via axis-angle)
            R_diff = T_plus[:3, :3] @ T_minus[:3, :3].T
            rot_axis = self._rotation_to_axis_angle(R_diff)
            J[3:, i] = rot_axis / (2 * delta)

        return J

    def _rotation_to_axis_angle(self, R):
        """Convert rotation matrix to axis-angle representation."""
        trace = np.trace(R)
        if trace > 2.9:  # Near identity
            return np.zeros(3)
        elif trace < -0.99:  # 180 degree rotation
            # Handle edge case
            return np.array([np.sqrt((R[0,0]+1)/2),
                           np.sqrt((R[1,1]+1)/2),
                           np.sqrt((R[2,2]+1)/2)]) * np.pi
        else:
            angle = np.arccos((trace - 1) / 2)
            axis = np.array([R[2,1] - R[1,2],
                           R[0,2] - R[2,0],
                           R[1,0] - R[0,1]]) / (2 * np.sin(angle))
            return axis * angle

    def solve(self, T_target, q_init, max_iter=100, tol=1e-4):
        """
        Solve IK using damped least squares with null-space projection.
        """
        q = np.array(q_init)

        for iteration in range(max_iter):
            T_current, _, _ = self.fk_func(q)

            # Compute pose error
            e_pos = T_target[:3, 3] - T_current[:3, 3]
            R_error = T_target[:3, :3] @ T_current[:3, :3].T
            e_rot = self._rotation_to_axis_angle(R_error)
            e = np.concatenate([e_pos, e_rot])

            if np.linalg.norm(e) < tol:
                return q, True, iteration

            # Compute Jacobian and DLS step
            J = self.compute_jacobian(q)
            J JT = J @ J.T
            delta_q = J.T @ solve_sylvester(J JT, self.lambda_damping**2 * np.eye(6), e)

            # Null-space projection for joint limit avoidance
            J_pinv = np.linalg.pinv(J)
            null_projector = np.eye(self.n) - J_pinv @ J

            # Gradient of quadratic joint limit penalty
            q_center = (self.joint_limits[:, 0] + self.joint_limits[:, 1]) / 2
            limit_gradient = (q - q_center) / ((self.joint_limits[:, 1] - self.joint_limits[:, 0]) / 2)**2

            q = q + delta_q + null_projector @ limit_gradient * 0.01

            # Clamp to joint limits
            q = np.clip(q, self.joint_limits[:, 0], self.joint_limits[:, 1])

        return q, False, max_iter
```

### Summary and Key Takeaways

- Analytical IK provides exact solutions with geometric insight
- Numerical IK handles general manipulators via optimization
- Damped least squares balances convergence and stability
- Null-space projection enables secondary objectives (limit avoidance)

---

## 2.4 Lagrangian Dynamics

### Energy-Based Formulation

The Lagrangian $L = T - V$ captures system energy. The Euler-Lagrange equation:

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right) - \frac{\partial L}{\partial q_i} = \tau_i
$$

generates equations of motion without constraint forces.

### Kinetic Energy and the Mass Matrix

For link $k$ with mass $m_k$, inertia tensor $I_k$, and Jacobian $J_k(q)$:

$$
T = \frac{1}{2} \sum_k \dot{q}^T J_k^T m_k J_k \dot{q} + \frac{1}{2} \sum_k \omega_k^T I_k \omega_k
$$

The mass matrix is:

$$
M(q) = \sum_k J_{v,k}^T m_k J_{v,k} + J_{\omega,k}^T I_k J_{\omega,k}
$$

### Potential Energy and Gravity

For constant gravitational field $g$:

$$
V(q) = -\sum_k m_k g^T p_{com,k}(q)
$$

where $p_{com,k}(q)$ is the center of mass position of link $k$.

### Derivation: 2-Link Planar Arm

Consider a 2-link arm with parameters $(L_1, L_2, m_1, m_2, I_1, I_2)$.

**Kinetic Energy**:

Link 1: $T_1 = \frac{1}{2} I_1 \dot{q}_1^2 + \frac{1}{2} m_1 v_{c1}^2$

Link 2: $T_2 = \frac{1}{2} I_2 (\dot{q}_1 + \dot{q}_2)^2 + \frac{1}{2} m_2 v_{c2}^2$

Computing $v_{c1}$ and $v_{c2}$ and collecting terms yields:

$$
T = \frac{1}{2} \dot{q}^T \begin{bmatrix} M_{11} & M_{12} \\ M_{21} & M_{22} \end{bmatrix} \dot{q}
$$

where:

$$
\begin{aligned}
M_{11} &= m_1 l_{c1}^2 + m_2(l_1^2 + l_{c2}^2 + 2l_1 l_{c2}\cos q_2) + I_1 + I_2 \\
M_{12} &= m_2(l_{c2}^2 + l_1 l_{c2}\cos q_2) + I_2 \\
M_{22} &= m_2 l_{c2}^2 + I_2
\end{aligned}
$$

**Potential Energy**:

$$
V = -m_1 g l_{c1} \cos q_1 - m_2 g (l_1 \cos q_1 + l_{c2} \cos(q_1 + q_2))
$$

**Lagrangian Equations**: Applying Euler-Lagrange to $L = T - V$ yields:

$$
\begin{aligned}
M_{11}\ddot{q}_1 + M_{12}\ddot{q}_2 - m_2 l_1 l_{c2}\sin q_2 (2\dot{q}_1\dot{q}_2 + \dot{q}_2^2) + (m_1 l_{c1} + m_2 l_1)g\sin q_1 + m_2 g l_{c2}\sin(q_1+q_2) &= \tau_1 \\
M_{12}\ddot{q}_1 + M_{22}\ddot{q}_2 + m_2 l_1 l_{c2}\sin q_2 \dot{q}_1^2 + m_2 g l_{c2}\sin(q_1+q_2) &= \tau_2
\end{aligned}
$$

### Canonical Robot Dynamics Equation

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau
$$

**Key Properties**:

1. **Symmetry**: $M(q) = M(q)^T$
2. **Positive Definiteness**: $x^T M(q) x > 0$ for all $x \neq 0$
3. **Skew-Symmetry**: $\dot{M}(q) - 2C(q, \dot{q})$ is skew-symmetric
4. **Linearity in Parameters**: $\tau = Y(q, \dot{q}, \ddot{q})\theta$ for parameter vector $\theta$

> **Research Note**: The linearity property enables adaptive control and parameter estimation. See Slotine & Li (1991) for adaptive controller design.

### Summary and Key Takeaways

- Lagrangian mechanics provides constraint-free equations of motion
- Mass matrix M(q) encodes configuration-dependent inertia
- Skew-symmetry of $\dot{M} - 2C$ underlies passivity proofs
- Parameter linearity enables adaptive and learning-based control

---

## 2.5 Newton-Euler Formulation

### Recursive Algorithm Structure

The Newton-Euler formulation computes dynamics in O(n) time via two passes:

**Forward Pass (Base to Tip)**:
1. Compute angular velocities: $\omega_i = R_{i-1}^i \omega_{i-1} + \dot{q}_i z_{i-1}$
2. Compute angular accelerations: $\alpha_i = R_{i-1}^i \alpha_{i-1} + \ddot{q}_i z_{i-1} + \omega_{i-1} \times \dot{q}_i z_{i-1}$
3. Compute linear accelerations: $a_i = R_{i-1}^i a_{i-1} + \alpha_i \times p_i^{i-1} + \omega_i \times (\omega_i \times p_i^{i-1})$

**Backward Pass (Tip to Base)**:
1. Compute forces: $f_i = R_i^{i+1} f_{i+1} + m_i(a_i + g)$
2. Compute torques: $n_i = R_i^{i+1} n_{i+1} + p_{com,i}^i \times f_i + I_i \alpha_i + \omega_i \times (I_i \omega_i)$
3. Joint torque: $\tau_i = n_i^T z_{i-1}$

### Implementation: O(n) Recursive Newton-Euler

```python
import numpy as np

class NewtonEuler:
    """
    Recursive Newton-Euler algorithm for robot dynamics.
    O(n) complexity for n-DOF manipulator.
    """

    def __init__(self, dh_params, masses, inertias, com_offsets):
        """
        dh_params: list of [a, d, alpha] for each joint
        masses: list of link masses
        inertias: list of 3x3 inertia tensors (about COM, in link frame)
        com_offsets: list of COM positions in link frame
        """
        self.n = len(dh_params)
        self.dh_params = np.array(dh_params)
        self.masses = np.array(masses)
        self.inertias = np.array(inertias)
        self.com_offsets = np.array(com_offsets)
        self.g = np.array([0, 0, -9.81])  # Gravity vector

    def rotation_matrix(self, theta, axis):
        """Generate rotation matrix for given axis and angle."""
        c, s = np.cos(theta), np.sin(theta)
        if axis == 'z':
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        elif axis == 'y':
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        elif axis == 'x':
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    def skew(self, v):
        """Skew-symmetric cross-product matrix."""
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])

    def compute_inverse_dynamics(self, q, qd, qdd):
        """
        Compute joint torques given positions, velocities, accelerations.

        Args:
            q: joint positions (radians)
            qd: joint velocities (rad/s)
            qdd: joint accelerations (rad/s^2)

        Returns:
            tau: joint torques
        """
        n = self.n
        R = [np.eye(3) for _ in range(n + 1)]  # Rotations
        p = [np.zeros(3) for _ in range(n + 1)]  # Positions
        omega = [np.zeros(3) for _ in range(n + 1)]  # Angular velocities
        alpha = [np.zeros(3) for _ in range(n + 1)]  # Angular accelerations
        a = [np.zeros(3) for _ in range(n + 1)]  # Linear accelerations
        f = [np.zeros(3) for _ in range(n + 1)]  # Forces
        n_torque = [np.zeros(3) for _ in range(n + 1)]  # Torques

        # Forward pass
        omega[0] = np.zeros(3)
        alpha[0] = np.zeros(3)
        a[0] = self.g

        for i in range(n):
            # Extract DH parameters
            a_i, d_i, alpha_i = self.dh_params[i]
            theta_i = q[i]

            # Transformation from frame i to i+1
            R_i = self.rotation_matrix(theta_i, 'z') @ \
                  self.rotation_matrix(d_i, 'z') @ \
                  self.rotation_matrix(a_i, 'x') @ \
                  self.rotation_matrix(alpha_i, 'x')
            R[i+1] = R[i] @ R_i

            # Joint axis (z-axis of previous frame)
            z_prev = R[i, :, 2]

            # Update velocities and accelerations
            omega[i+1] = R[i].T @ omega[i] + qd[i] * z_prev
            alpha[i+1] = R[i].T @ alpha[i] + qdd[i] * z_prev + \
                        self.skew(R[i].T @ omega[i]) @ (qd[i] * z_prev)

            # Link COM position and acceleration
            p_com_i = np.array([a_i/2, 0, d_i])  # Simplified: COM at link midpoint
            a[i+1] = R[i].T @ a[i] + self.skew(alpha[i+1]) @ p_com_i + \
                    self.skew(omega[i+1]) @ (self.skew(omega[i+1]) @ p_com_i)

        # Backward pass
        for i in range(n - 1, -1, -1):
            a_i, d_i, alpha_i = self.dh_params[i]
            z_prev = R[i, :, 2]
            p_com_i = np.array([a_i/2, 0, d_i])

            # Force balance
            f[i] = R[i+1] @ f[i+1] + self.masses[i] * (a[i+1] + self.g)

            # Torque balance
            n_torque[i] = R[i+1] @ n_torque[i] + \
                         self.skew(p_com_i) @ f[i] + \
                         self.inertias[i] @ alpha[i+1] + \
                         self.skew(omega[i+1]) @ (self.inertias[i] @ omega[i+1])

        # Extract joint torques (projection onto joint axis)
        tau = np.zeros(n)
        for i in range(n):
            z_prev = R[i, :, 2]
            tau[i] = n_torque[i] @ z_prev

        return tau

    def compute_mass_matrix(self, q):
        """
        Compute the n x n mass matrix M(q) using RNEA with unit accelerations.
        """
        n = self.n
        M = np.zeros((n, n))

        for i in range(n):
            # Unit acceleration at joint i, zero elsewhere
            qdd = np.zeros(n)
            qdd[i] = 1.0

            # Compute torques with zero velocity and gravity compensation
            qd = np.zeros(n)
            tau = self.compute_inverse_dynamics(q, qd, qdd)
            M[:, i] = tau

        return M

    def compute_coriolis_gravity(self, q, qd):
        """
        Compute C(q, qd)*qd + g(q) using RNEA with zero acceleration.
        """
        qdd = np.zeros(self.n)
        return self.compute_inverse_dynamics(q, qd, qdd)
```

### Complexity Analysis

| Method | Complexity | Memory | Use Case |
|--------|------------|--------|----------|
| Lagrangian (naive) | O(n^3) | O(n^2) | Symbolic derivation |
| Lagrangian (efficient) | O(n^2) | O(n^2) | Precomputation |
| Newton-Euler | O(n) | O(n) | Real-time control |
| Articulated Body | O(n) | O(n) | Multiple queries |

### Summary and Key Takeaways

- Newton-Euler achieves O(n) complexity via forward/backward recursion
- RNEA (Recursive Newton-Euler Algorithm) is standard in real-time controllers
- Mass matrix computed via n calls to RNEA with unit accelerations
- Popular libraries (Pinocchio, Drake, MuJoCo) implement optimized versions

---

## 2.6 Simulation of Multi-Body Dynamics

### Physics Engine Comparison

| Engine | Formulation | Contact Model | Research Use |
|--------|-------------|---------------|--------------|
| MuJoCo | Convex optimization | Elastic, pyramidal | Manipulation, locomotion |
| Drake | Trajectory optimization | ICP, SCCP | Planning, control |
| PyBullet | LCP (Lemke) | Spring-damper | ML, RL |
| DART | Featherstone (ABA) | LCP | Grasping, biomechanics |

### MuJoCo Implementation

```python
import mujoco
import numpy as np

class MuJoCoSimulator:
    """
    MuJoCo-based simulator for robot dynamics and control.
    """

    def __init__(self, xml_path, control_freq=500):
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        self.nu = self.model.nu  # Number of actuators
        self.nq = self.model.nq  # Configuration dimension

    def reset(self, q0=None, qd0=None):
        """Reset simulation to initial state."""
        mujoco.mj_resetData(self.model, self.data)
        if q0 is not None:
            self.data.qpos[:] = q0
        if qd0 is not None:
            self.data.qvel[:] = qd0
        mujoco.mj_forward(self.model, self.data)

    def step(self, tau=None, q_des=None, kp=100, kd=10):
        """
        Advance simulation by one timestep.

        If tau is None, uses PD control to track q_des.
        """
        if tau is None and q_des is not None:
            # PD control
            tau = kp * (q_des - self.data.qpos[:self.nu]) - \
                  kd * self.data.qvel[:self.nu]

        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)
        return self.get_state()

    def get_state(self):
        """Return current joint state."""
        return {
            'q': self.data.qpos[:self.nu].copy(),
            'qd': self.data.qvel[:self.nu].copy(),
            'tau': self.data.ctrl[:self.nu].copy(),
            'time': self.data.time
        }

    def compute_dynamics(self, q, qd):
        """
        Compute M(q)*qdd + C(q,qd)*qd + g(q) using MuJoCo's internal solver.
        """
        # Set state
        self.data.qpos[:self.nu] = q
        self.data.qvel[:self.nu] = qd
        self.data.qacc[:] = 0
        mujoco.mj_forward(self.model, self.data)

        # Extract terms
        M = np.zeros((self.nu, self.nu))
        mujoco.mj_fullM(self.model, M, self.data.qM)

        g = self.data.qfrc_bias.copy()

        return M, g

    def inverse_dynamics(self, q, qd, qdd):
        """
        Compute joint torques for given accelerations.
        """
        self.data.qpos[:self.nu] = q
        self.data.qvel[:self.nu] = qd
        self.data.qacc[:self.nu] = qdd
        mujoco.mj_forward(self.model, self.data)
        return self.data.qfrc_inverse.copy()
```

### Validation and Verification

**Analytical Validation**: Compare simulation against closed-form dynamics:

```python
def validate_simulation(simulator, q_test, qd_test, params):
    """
    Compare MuJoCo dynamics against analytical Lagrangian model.
    """
    # Analytical model
    M_analytical = compute_mass_matrix_lagrangian(q_test, params)
    g_analytical = compute_gravity_lagrangian(q_test, params)

    # MuJoCo model
    M_mujoco, g_mujoco = simulator.compute_dynamics(q_test, qd_test)

    # Compare
    M_error = np.linalg.norm(M_analytical - M_mujoco) / np.linalg.norm(M_analytical)
    g_error = np.linalg.norm(g_analytical - g_mujoco) / (np.linalg.norm(g_analytical) + 1e-6)

    return M_error, g_error
```

### Summary and Key Takeaways

- Physics engines provide validated multi-body dynamics
- MuJoCo's convex contact formulation enables stable simulation
- Always validate simulation against analytical models
- Domain randomization bridges sim-to-real gaps

---

## 2.7 Screw Theory and Product of Exponentials (Advanced)

### Mathematical Foundations

Screw theory provides a unified language for kinematics and dynamics. A screw $S$ in $\mathbb{R}^3$ is characterized by:

- Direction $s \in \mathbb{R}^3$ (unit vector along the screw axis)
- Point $q \in \mathbb{R}^3$ (any point on the axis)

The moment is $s_0 = q \times s$. A screw is $S = (s; s_0) \in \mathbb{R}^6$.

### Exponential Coordinates for SE(3)

The exponential map $\exp: \mathfrak{se}(3) \to SE(3)$ is:

$$
\exp([S]\theta) = \begin{bmatrix} I + \sin\theta [s]_\times + (1-\cos\theta)[s]_\times^2 & (I\theta + (1-\cos\theta)[s]_\times + (\theta - \sin\theta)[s]_\times^2) q \\ 0 & 1 \end{bmatrix}
$$

### Lie Group Structure

SE(3) is a Lie group with Lie algebra $\mathfrak{se}(3)$:

- **Group operation**: Matrix multiplication
- **Identity**: 4x4 identity matrix
- **Inverse**: $T^{-1} = \begin{bmatrix} R^T & -R^T t \\ 0 & 1 \end{bmatrix}$
- **Adjoint action**: $Ad_T(S) = \begin{bmatrix} R & [t]_\times R \\ 0 & R \end{bmatrix} S$

### PoE Formula Derivation

For a robot with home configuration $T(0)$ and joint screws $S_1, ..., S_n$:

$$
T(\theta) = e^{[S_1]\theta_1} e^{[S_2]\theta_2} \cdots e^{[S_n]\theta_n} T(0)
$$

**Advantages over DH**:
1. No frame attachment required at each joint
2. More robust to calibration errors
3. Naturally handles parallel mechanisms
4. Extends to closed chains via constraint screws

### Implementation Example

```python
import numpy as np
from scipy.linalg import expm

class ProductOfExponentials:
    """
    Forward kinematics using the Product of Exponentials formula.
    """

    def __init__(self, S, T_home):
        """
        S: n x 6 array of screw coordinates
        T_home: 4x4 home configuration
        """
        self.S = S  # Screws in se(3) form
        self.T_home = T_home

    def screw_to_se3(self, s, theta):
        """Convert screw to SE(3) exponential."""
        S = np.zeros((4, 4))
        S[:3, :3] = self.skew(s[:3])
        S[:3, 3] = s[3:]
        return expm(theta * S)

    def skew(self, v):
        """Skew-symmetric matrix."""
        return np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])

    def forward_kinematics(self, theta):
        """Compute FK using PoE formula."""
        T = self.T_home.copy()
        for i in range(len(theta)):
            S_i = np.zeros((4, 4))
            S_i[:3, :3] = self.skew(self.S[i, :3])
            S_i[:3, 3] = self.S[i, 3:]
            T = expm(theta[i] * S_i) @ T  # Pre-multiplication for body-fixed
        return T

    def geometric_jacobian(self, theta):
        """
        Compute geometric Jacobian at configuration theta.
        J = [J_1, J_2, ..., J_n] where J_i = Ad_{e^[S_1]...e^[S_{i-1}]}(S_i)
        """
        n = len(theta)
        J = np.zeros((6, n))
        T_current = np.eye(4)

        for i in range(n):
            # Compute adjoint transformation for accumulated transforms
            R = T_current[:3, :3]
            t = T_current[:3, 3]

            S_i_body = np.zeros(6)
            S_i_body[:3] = R @ self.S[i, :3]
            S_i_body[3:] = R @ self.S[i, 3:] + np.cross(t, R @ self.S[i, :3])

            J[:, i] = S_i_body

            # Update T_current
            S_i = np.zeros((4, 4))
            S_i[:3, :3] = self.skew(self.S[i, :3])
            S_i[:3, 3] = self.S[i, 3:]
            T_current = expm(theta[i] * S_i) @ T_current

        return J
```

> **Research Note**: Recent advances in screw theory include applications to parallel mechanisms (Gosselin & Angeles, 1990), wire-driven robots (Ming & Higuchi, 1994), and compliant mechanisms (Howell & Midha, 1994).

### Summary and Key Takeaways

- Screw theory provides geometric foundations for robot kinematics
- PoE formula offers calibration-robust forward kinematics
- Lie group structure enables geometric control design
- Adjoint transformations relate body-fixed and spatial velocities

---

## 2.8 Optimal Control for Trajectory Planning

### Problem Formulation

Trajectory planning as optimal control:

$$
\min_{q(t), \tau(t)} \int_0^T L(q, \dot{q}, \tau, t) dt + \Phi(q(T))
$$

subject to:
- $\dot{q} = f(q, \tau)$ (dynamics)
- $q(0) = q_0, q(T) = q_T$ (boundary conditions)
- $q_{min} \leq q(t) \leq q_{max}$ (joint limits)
- $|\tau(t)| \leq \tau_{max}$ (torque limits)

### Cost Functions

**Minimum Time**: $L = 1$ (time-optimal trajectories)

**Minimum Energy**: $L = \tau^T \tau$ (energy-optimal)

**Quadratic Tracking**: $L = (q - q_d)^T Q (q - q_d) + \dot{q}^T R \dot{q}$

### Solution Approaches

**Direct Transcription**: Discretize and convert to nonlinear programming:

```python
import numpy as np
from scipy.optimize import minimize

class DirectTranscription:
    """
    Direct collocation for trajectory optimization.
    """

    def __init__(self, dynamics_func, n, N, T):
        self.dynamics = dynamics_func
        self.n = n
        self.N = N  # Number of segments
        self.T = T  # Total time
        self.dt = T / N

    def discretize(self, z0):
        """
        z0: initial guess [q_0, q_1, ..., q_N]
        """
        N, n = self.N, self.n
        z = np.zeros((N + 1) * n)
        for i in range(N + 1):
            z[i*n:(i+1)*n] = z0
        return z

    def dynamics_constraints(self, z):
        """Enforce dynamics at collocation points."""
        N, n, dt = self.N, self.n, self.dt
        constraints = []

        for i in range(N):
            q_i = z[i*n:(i+1)*n]
            q_ip1 = z[(i+1)*n:(i+2)*n]

            # Midpoint integration
            q_mid = (q_i + q_ip1) / 2
            qd_mid = self.dynamics(q_mid)  # Assuming torque = 0 for kinematic planning

            # Defect constraint
            defect = q_ip1 - q_i - dt * qd_mid
            constraints.extend(defect)

        return np.array(constraints)

    def optimize(self, q_init, q_final):
        """Solve trajectory optimization."""
        N, n = self.N, self.n

        # Initial guess
        z0 = self.discretize(q_init)

        # Constraints
        constraints = [{'type': 'eq', 'fun': self.dynamics_constraints}]

        # Boundary conditions
        def boundary_constraints(z):
            return np.array([z[:n] - q_init, z[-n:] - q_final])
        constraints.append({'type': 'eq', 'fun': boundary_constraints})

        # Joint limits (simplified: just bounds on variables)
        bounds = [(0, 2*np.pi) for _ in range((N+1)*n)]

        result = minimize(self.objective, z0, constraints=constraints, bounds=bounds)
        return result
```

### Differential Dynamic Programming (DDP/iLQR)

For real-time applications, iLQR provides efficient local optimization:

```python
class ILQR:
    """
    Iterative Linear Quadratic Regulator for trajectory optimization.
    """

    def __init__(self, dynamics, cost_func, n, m):
        self.dynamics = dynamics
        self.cost = cost_func
        self.n = n  # State dimension
        self.m = m  # Control dimension

    def backward_pass(self, x_trj, u_trj):
        """
        Compute gains and value function derivatives.
        """
        N = len(x_trj) - 1

        # Initialize
        Vx = self.cost.terminal_grad(x_trj[-1])
        Vxx = self.cost.terminal_hess(x_trj[-1])

        K = []  # Feedback gains
        k = []  # Feedforward terms

        for t in range(N - 1, -1, -1):
            x, u = x_trj[t], u_trj[t]

            # Linearize dynamics
            A, B = self.dynamics.jacobians(x, u)

            # Quadraticize cost
            l_x, l_u, l_xx, l_uu, l_ux = self.cost.quadratic(t, x, u)

            # Riccati update
            Qx = l_x + A.T @ Vx
            Qu = l_u + B.T @ Vx
            Qxx = l_xx + A.T @ Vxx @ A
            Quu = l_uu + B.T @ Vxx @ B
            Qux = l_ux + B.T @ Vxx @ A

            # Regularization for numerical stability
            reg = 1e-6
            Quu_reg = Quu + reg * np.eye(self.m)

            # Compute gains
            K_t = np.linalg.solve(Quu_reg, Qux)
            k_t = np.linalg.solve(Quu_reg, Qu)

            # Update value function
            Vx = Qx - K_t.T @ Quu_reg @ k_t
            Vxx = Qxx - K_t.T @ Quu_reg @ K_t

            K.append(K_t)
            k.append(k_t)

        return K[::-1], k[::-1]

    def forward_pass(self, x_trj, u_trj, K, k, alpha=1.0):
        """
        Simulate with updated controls.
        """
        x_new = [x_trj[0]]
        u_new = []

        for t in range(len(u_trj)):
            # Feedback + feedforward
            delta_u = -K[t] @ (x_new[-1] - x_trj[t]) - alpha * k[t]
            u_new.append(u_trj[t] + delta_u)

            # Simulate dynamics
            x_next = self.dynamics.step(x_new[-1], u_new[-1])
            x_new.append(x_next)

        return x_new, u_new

    def optimize(self, x0, u_init, max_iter=100):
        """
        Run iLQR optimization.
        """
        x_trj = [x0]
        for _ in range(len(u_init)):
            x_trj.append(self.dynamics.step(x_trj[-1], u_init[0]))

        u_trj = u_init.copy()

        for iteration in range(max_iter):
            K, k = self.backward_pass(x_trj, u_trj)

            # Line search
            for alpha in [1.0, 0.8, 0.6, 0.4, 0.2]:
                x_new, u_new = self.forward_pass(x_trj, u_trj, K, k, alpha)

                # Check convergence
                if self.cost.evaluate(x_new, u_new) < self.cost.evaluate(x_trj, u_trj):
                    x_trj, u_trj = x_new, u_new
                    break

            # Check termination
            if np.max(np.abs(u_new[0] - u_init[0])) < 1e-6:
                break

        return x_trj, u_trj
```

### Research Directions in Optimal Control for Manipulation

1. **Model Predictive Control (MPC)**: Real-time optimization with receding horizon (Koenemann et al., 2015)

2. **Learning-Based Cost Functions**: Data-driven task specification (Levine et al., 2016)

3. **Contact-Rich Manipulation**: Mixed-integer formulations for switching contacts (Mordatch et al., 2012)

4. **Whole-Body Optimization**: Simultaneous task and balance (Kuindersma et al., 2016)

### Summary and Key Takeaways

- Optimal control provides principled trajectory generation
- Direct methods convert infinite-dimensional problems to NLP
- DDP/iLQR enable real-time local optimization
- Contact-rich tasks require hybrid system formulations

---

## Chapter Summary

This chapter developed advanced mathematical foundations for robot manipulation:

**Kinematics**:
- DH convention and PoE formula for forward kinematics
- Analytical and numerical methods for inverse kinematics
- Singularity analysis via Jacobian SVD

**Dynamics**:
- Lagrangian formulation with full derivations
- Newton-Euler recursion for O(n) computation
- Properties of robot dynamics (symmetry, passivity)

**Advanced Topics**:
- Screw theory and Lie group structure
- Optimal control for trajectory planning
- Physics simulation and validation

**Key Takeaways**:
1. Multiple kinematic formulations serve different applications
2. Dynamics equations enable model-based control design
3. Optimal control bridges planning and execution
4. Simulation validation catches modeling errors

**Check Your Understanding**:
- Derive the PoE formula from screw theory axioms
- Show that $\dot{M} - 2C$ is skew-symmetric
- Compare computational complexity of Lagrangian vs. Newton-Euler
- Formulate minimum-time trajectory planning as optimal control

**Next Steps**: Chapter 3 covers motion planning algorithms (probabilistic roadmaps, RRT*, potential fields) building on configuration space concepts. Chapter 4 introduces feedback control (computed torque, impedance control) using the dynamics models developed here.

---

## Lab Exercises

**Lab 2.1** (45 min): **Implement PoE Forward Kinematics**
Derive screw coordinates for a 3-DOF arm and implement forward kinematics using the Product of Exponentials formula. Compare results against DH-based implementation.

**Lab 2.2** (60 min): **Implement Complete Newton-Euler Algorithm**
Implement the full recursive Newton-Euler algorithm including mass matrix computation. Validate against symbolic Lagrangian dynamics for a 2-link arm.

**Lab 2.3** (90 min): **iLQR Trajectory Optimization**
Implement iLQR for minimum-effort trajectory planning on a 2-link arm. Experiment with different cost functions and analyze convergence behavior.

**Lab 2.4** (120 min): **Contact-Rich Manipulation Planning**
Implement a simple hybrid system model for planar pushing. Formulate as mixed-integer optimal control and compare against sampling-based planning.

**Lab 2.5** (Research): **Reproduce a Paper Result**
Select one paper from the references and implement its key algorithm. Compare performance against baseline methods.

---

## References

### Primary Texts
- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley.
- Murray, R. M., Li, Z., & Sastry, S. S. (1994). *A Mathematical Introduction to Robotic Manipulation*. CRC Press.
- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2010). *Robotics: Modelling, Planning and Control*. Springer.

### Research Papers
- Berkenkamp, F., et al. (2016). "Safe Model-based Reinforcement Learning with Stability Guarantees." NeurIPS.
- Diaz-Calderon, A., et al. (2005). "Efficient Parallel IK for Serial Chains." ICRA.
- Koenemann, J., et al. (2015). "Whole-body Model Predictive Control for Mobile Manipulation." ICRA.
- Kuindersma, S., et al. (2016). "Optimization-based Locomotion Planning." IJRR.
- Kavraki, L. E., et al. (1996). "Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces." TRO.
- Karaman, S., & Frazzoli, E. (2011). "Sampling-based Algorithms for Optimal Motion Planning." IJRR.
- Levine, S., et al. (2016). "End-to-End Training of Deep Visuomotor Policies." JMLR.
- Mordatch, I., et al. (2012). "Discovery of Complex Behaviors through Contact-Aware Optimization." ACM TOG.

### Software and Resources
- MuJoCo Documentation: https://mujoco.readthedocs.io/
- Drake: https://drake.mit.edu/
- Pinocchio: https://stack-of-tasks.github.io/pinocchio/
- Kavraki Lab Motion Planning Library: https://www.kavrakilab.org/motion-planning/

### Advanced Reading
- Murray, R. M. (Ed.). (2017). *A Mathematical Introduction to Robotic Manipulation* (2nd ed.). CRC Press. (Forthcoming revision with recent advances)
- Bicchi, A., & Kumar, V. (2000). "Robotic Grasping and Contact: A Review." ICRA.
- Pratt, G. A., & Williamson, M. M. (1995). "Series Elastic Actuators." IROS.
