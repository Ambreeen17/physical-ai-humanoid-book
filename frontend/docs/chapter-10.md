---
sidebar_position: 11
title: "Chapter 10: Reinforcement Learning"
---

# Chapter 10: Reinforcement Learning

<PersonalizationToggle chapterId="10" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Model robotics problems** as Markov Decision Processes (MDPs)
2. **Implement value-based methods** (DQN) for discrete actions
3. **Use policy-gradient methods** (PPO, SAC) for continuous control
4. **Train robots in simulation** using Isaac Gym / Orbax
5. **Bridge the sim-to-real gap** with domain randomization

---

## Introduction

Classical control (Chapter 6) works great when you have a perfect model. But what if the dynamics are complex (e.g., granular soil)? what if the environment is unstructured?

**Reinforcement Learning (RL)** allows robots to learn optimal behaviors through trial and error, just like humans learn to walk or ride a bike.

---

## Section 10.1: The RL Framework

### Markov Decision Process (MDP)

An MDP is defined by the tuple $(S, A, P, R, \gamma)$:
- **State ($S$)**: The robot's observation (e.g., joint angles, camera image)
- **Action ($A$)**: The command sent to actuators
- **Transition ($P$)**: Probability of moving to state $s'$ given $s, a$
- **Reward ($R$)**: Scalar feedback signal (e.g., +1 for walking, -1 for falling)
- **Discount ($\gamma$)**: How much future rewards matter ($0 \le \gamma \le 1$)

```python
import gym
import numpy as np

class RobotEnv(gym.Env):
    """
    OpenAI Gym interface for a robot environment.
    """
    def __init__(self):
        super().__init__()
        # Continuous action space: [torque_1, torque_2, ...]
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(7,))

        # State space: [q, q_dot, end_effector_pos]
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(20,))

    def step(self, action):
        """Execute one time step."""
        # 1. Apply action to simulation
        self.sim.set_joint_torques(action)
        self.sim.step()

        # 2. Get new state
        obs = self._get_observation()

        # 3. Compute reward
        reward = self._compute_reward(obs, action)

        # 4. Check termination
        done = self._check_termination(obs)

        return obs, reward, done, {}

    def _compute_reward(self, obs, action):
        # Example: Minimize distance to target, minimize energy
        dist = np.linalg.norm(obs[:3] - self.target_pos)
        energy = np.sum(np.square(action))
        return -dist - 0.01 * energy
```

---

## Section 10.2: Deep Q-Learning (DQN)

For discrete action spaces (e.g., "move left", "move right").

```python
import torch
import torch.nn as nn
import torch.optim as optim

class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim)
        )

    def forward(self, x):
        return self.net(x)

class DQNAgent:
    def __init__(self, state_dim, action_dim):
        self.q_net = DQN(state_dim, action_dim)
        self.target_args = DQN(state_dim, action_dim)
        self.optimizer = optim.Adam(self.q_net.parameters(), lr=1e-3)
        self.replay_buffer = []

    def update(self, batch_size=64):
        if len(self.replay_buffer) < batch_size:
            return

        # Sample batch
        batch = np.random.choice(self.replay_buffer, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)

        # Compute loss
        # ... (Bellman equation)
        loss.backward()
        self.optimizer.step()
```

---

## Section 10.3: Proximal Policy Optimization (PPO)

PPO is the industry standard for continuous control robotics. It's a **Policy Gradient** method.

### The Actor-Critic Architecture

- **Actor ($\pi_\theta(a|s)$)**: Outputs action distribution (mean, std)
- **Critic ($V_\phi(s)$)**: Estimates value of state

```python
class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super().__init__()
        # Actor
        self.actor_mean = nn.Sequential(
            nn.Linear(state_dim, 64), nn.Tanh(),
            nn.Linear(64, 64), nn.Tanh(),
            nn.Linear(64, action_dim)
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, action_dim))

        # Critic
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 64), nn.Tanh(),
            nn.Linear(64, 64), nn.Tanh(),
            nn.Linear(64, 1)
        )

    def act(self, state):
        mean = self.actor_mean(state)
        std = self.actor_logstd.exp()
        dist = torch.distributions.Normal(mean, std)
        action = dist.sample()
        return action

    def evaluate(self, state, action):
        # Helper for PPO update
        mean = self.actor_mean(state)
        std = self.actor_logstd.exp()
        dist = torch.distributions.Normal(mean, std)
        log_prob = dist.log_prob(action)
        value = self.critic(state)
        return log_prob, value
```

---

## Section 10.4: Soft Actor-Critic (SAC)

SAC is an off-policy algorithm that maximizes both reward and entropy (randomness), encouraging exploration.

```python
class SACAgent:
    """
    Soft Actor-Critic for sample-efficient continuous control.
    """
    def __init__(self, state_dim, action_dim):
        self.actor = Actor(state_dim, action_dim)
        self.critic_1 = SoftQNetwork(state_dim, action_dim)
        self.critic_2 = SoftQNetwork(state_dim, action_dim)
        self.alpha = 0.2  # Entropy coefficient

    def update(self, batch):
        # 1. Update critics (minimize Bellman error)
        # target = r + gamma * (min(Q1, Q2) - alpha * log_prob)

        # 2. Update actor (maximize Q - alpha * log_prob)

        # 3. Tuning alpha (optional automatically tuning)
        pass
```

---

## Section 10.5: Sim-to-Real Transfer

Robots trained in perfectly clean simulations often fail in the messy real world. This is the **Reality Gap**.

### Domain Randomization

We randomize physical parameters during training so the real world looks like just another variation.

```python
def randomize_environment(sim):
    """
    Apply domain randomization.
    """
    # 1. Physical properties
    sim.set_friction(np.random.uniform(0.5, 1.2))
    sim.set_mass(robot_link, np.random.uniform(0.9, 1.1) * nominal_mass)

    # 2. Visual appearance
    sim.set_light_position(np.random.normal(0, 1, 3))
    sim.set_color(object, np.random.rand(3))

    # 3. Sensor noise
    sim.set_camera_noise(np.random.uniform(0, 0.05))
```

---

## Summary

### Key Takeaways

1. **RL** enables learning complex behaviors without analytical models
2. **PPO** is robust and standard for on-policy continuous control
3. **SAC** is sample-efficient for off-policy learning
4. **Reward shaping** is critical—you get what you reward, not what you want
5. **Domain randomization** is essential for transferring policies to real hardware

### Looking Ahead

In Chapter 11, we'll explore **Imitation Learning**—teaching robots by demonstrating behaviors rather than defining reward functions.

---

**Chapter completed**: 2026-01-20
**Chapter**: 10 - Reinforcement Learning
