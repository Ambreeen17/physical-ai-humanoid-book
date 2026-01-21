---
sidebar_position: 12
title: "Chapter 11: Imitation Learning"
---

# Chapter 11: Imitation Learning

<PersonalizationToggle chapterId="11" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Understand Imitation Learning (IL)** and when to use it over RL
2. **Implement Behavior Cloning (BC)** to mimic expert actions
3. **Address covariate shift** with DAgger (Dataset Aggregation)
4. **Use Inverse Reinforcement Learning (IRL)** to recover reward functions
5. **Collect demonstration data** using teleoperation

---

## Introduction

In Reinforcement Learning (Chapter 10), we spent hours crafting reward functions. But for many tasks—like folding a shirt—it's hard to write a reward function but easy to *show* the robot what to do.

**Imitation Learning** allows robots to learn from expert demonstrations.

---

## Section 11.1: Collecting Demonstrations

Before we can learn, we need data. Demonstrations are sequences of state-action pairs: $\tau = \{(s_0, a_0), (s_1, a_1), \dots, (s_T, a_T)\}$.

### Teleoperation Pipeline

```python
class TeleopRecorder:
    """
    Record human demonstrations using a joystick or VR controller.
    """
    def __init__(self, robot, interface):
        self.robot = robot
        self.interface = interface
        self.buffer = []

    def record_episode(self):
        """Record one demonstration episode."""
        obs = self.robot.reset()
        episode_data = []
        done = False

        print("Recording started...")
        while not done:
            # Get action from human input
            action = self.interface.get_action()

            # Execute on robot
            next_obs, reward, done, _ = self.robot.step(action)

            # Store (s, a) pair
            episode_data.append((obs, action))
            obs = next_obs

        self.buffer.extend(episode_data)
        print(f"Recorded episode with {len(episode_data)} steps.")

    def save_dataset(self, filename="demos.pkl"):
        import pickle
        with open(filename, 'wb') as f:
            pickle.dump(self.buffer, f)
```

---

## Section 11.2: Behavior Cloning (BC)

Behavior Cloning treats imitation as a **Supervised Learning** problem.
Input: State $s$
Target: Expert Action $a^*$
Loss: MSE (for continuous) or Cross-Entropy (for discrete)

```python
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset

class BCNetwork(nn.Module):
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

def train_bc(model, dataset, epochs=100):
    optimizer = optim.Adam(model.parameters(), lr=1e-3)
    criterion = nn.MSELoss()

    loader = DataLoader(dataset, batch_size=32, shuffle=True)

    for epoch in range(epochs):
        total_loss = 0
        for states, actions in loader:
            optimizer.zero_grad()
            pred_actions = model(states)
            loss = criterion(pred_actions, actions)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()

        print(f"Epoch {epoch}: Loss {total_loss/len(loader)}")
```

### The Covariate Shift Problem

BC fails because errors compound. If the robot drifts slightly off the expert's path, it enters a state it has never seen (since the expert never makes mistakes!), panics, acts randomly, and crashes. This is **covariate shift**.

---

## Section 11.3: DAgger (Dataset Aggregation)

DAgger solves covariate shift by iteratively collecting data *from the robot's own policy*, corrected by the expert.

**Algorithm:**
1. Train policy $\pi$ on expert data $D$.
2. Run $\pi$ to collect new observations.
3. Ask expert to label actions for these observations.
4. Add to dataset $D$.
5. Repeat.

```python
class DAggerTrainer:
    def __init__(self, env, expert_policy, learner_model):
        self.env = env
        self.expert = expert_policy
        self.learner = learner_model
        self.dataset = []

    def train_iteration(self):
        # 1. Rollout with LEARNER policy
        obs = self.env.reset()
        done = False
        new_data = []

        while not done:
            # Robot chooses action
            action_learner = self.learner.predict(obs)

            # Expert says what they WOULD have done
            action_expert = self.expert.predict(obs)

            # Execute learner's action to explore
            next_obs, _, done, _ = self.env.step(action_learner)

            # Store Expert's action as ground truth
            new_data.append((obs, action_expert))
            obs = next_obs

        # 2. Aggregate data
        self.dataset.extend(new_data)

        # 3. Retrain learner
        train_bc(self.learner, self.dataset)
```

---

## Section 11.4: Inverse Reinforcement Learning (IRL)

Instead of copying actions directly, IRL tries to figure out the **reward function** the expert is maximizing.

**Advantage**: The learned reward is robust to dynamics changes.
**Disadvantage**: Hard (ill-posed problem).

### Maximum Entropy IRL

We assume expert trajectories are samples from a distribution:
$P(\tau) \propto \exp(R(\tau))$

The goal is to find reward weights $w$ such that the expert's feature counts match the learner's feature counts.

```python
def unnormalized_reward(state, weights):
    features = extract_features(state)
    return np.dot(weights, features)
```

---

## Section 11.5: One-Shot Imitation

Can we learn from a *single* video of a human doing a task?

Modern approaches (like **MAML**, **TAME**) allow robots to adapt to new tasks with very few demonstrations, often leveraging large pre-trained visual models.

---

## Summary

### Key Takeaways

1. **Behavior Cloning** is simplest: supervised learning $(s \to a)$.
2. **Covariate Shift** kills BC: small errors pile up until failure.
3. **DAgger** fixes this by asking the expert to correct the robot's own mistakes.
4. **Teleoperation** is the bottleneck: data collection is expensive.

### Looking Ahead

In Chapter 12, we'll combine Vision and Language with Action in Foundation Models (**VLA**), enabling robots to understand "pick up the red apple" without specific training for apples.

---

**Chapter completed**: 2026-01-20
**Chapter**: 11 - Imitation Learning
