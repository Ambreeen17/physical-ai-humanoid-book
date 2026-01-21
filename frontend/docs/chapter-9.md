---
sidebar_position: 10
title: "Chapter 9: Task & Motion Planning"
---

# Chapter 9: Task & Motion Planning

<PersonalizationToggle chapterId="9" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Understand TAMP concepts** and why classical planning is insufficient
2. **Implement symbolic planning** using PDDL
3. **Combine logic and geometry** for hybrid planning
4. **Solve long-horizon tasks** like setting a table or assembling furniture
5. **Use PDDLStream** for integrated task and motion planning

---

## Introduction

In Chapter 7, we learned how to move a robot from A to B (Motion Planning). In Chapter 8, we learned how to pick things up (Manipulation).

But what if the goal is "clean the kitchen"?

You can't just run RRT to "clean kitchen." You need a sequence of high-level actions:
1. Open the cupboard
2. Pick up the mug
3. Place mug in dishwasher
4. Close cupboard

This is **Task and Motion Planning (TAMP)**: reasoning about *what* to do (discrete logic) and *how* to do it (continuous geometry).

---

## Section 9.1: Symbolic Planning with PDDL

### The Planning Domain Definition Language (PDDL)

PDDL describes the world using **objects**, **predicates**, and **actions**.

**Domain Definition (`domain.pddl`)**:
```lisp
(define (domain household-robot)
  (:requirements :strips :typing)
  (:types robot location object)

  (:predicates
    (at ?r - robot ?l - location)
    (holding ?r - robot ?o - object)
    (at-obj ?o - object ?l - location)
    (empty-hand ?r - robot)
    (reachable ?l - location ?l2 - location)
  )

  (:action move
    :parameters (?r - robot ?from - location ?to - location)
    :precondition (and (at ?r ?from) (reachable ?from ?to))
    :effect (and (not (at ?r ?from)) (at ?r ?to))
  )

  (:action pick
    :parameters (?r - robot ?o - object ?l - location)
    :precondition (and (at ?r ?l) (at-obj ?o ?l) (empty-hand ?r))
    :effect (and (not (at-obj ?o ?l)) (not (empty-hand ?r)) (holding ?r ?o))
  )

  (:action place
    :parameters (?r - robot ?o - object ?l - location)
    :precondition (and (at ?r ?l) (holding ?r ?o))
    :effect (and (not (holding ?r ?o)) (empty-hand ?r) (at-obj ?o ?l))
  )
)
```

**Problem Definition (`problem.pddl`)**:
```lisp
(define (problem clean-kitchen)
  (:domain household-robot)
  (:objects
    r1 - robot
    mug - object
    table cupboard dishwasher - location
  )
  (:init
    (at r1 table)
    (at-obj mug table)
    (empty-hand r1)
    (reachable table cupboard)
    (reachable cupboard dishwasher)
    (reachable table dishwasher)
  )
  (:goal
    (at-obj mug dishwasher)
  )
)
```

### Python PDDL Planner

```python
class PDDLPlanner:
    """
    Simple symbolic planner using PDDL.
    """
    def __init__(self, domain_file, problem_file):
        self.domain_file = domain_file
        self.problem_file = problem_file

    def solve(self):
        """
        Call external planner (e.g., Fast Downward)
        """
        # In a real implementation, we would call an external binary
        # Here we simulate the output plan
        plan = [
            "move(r1, table, dishwasher)",  # Wait, mug is at table!
            # Correct plan:
            "pick(r1, mug, table)",
            "move(r1, table, dishwasher)",
            "place(r1, mug, dishwasher)"
        ]
        return plan
```

---

## Section 9.2: The TAMP Problem

Pure symbolic planning ignores geometry. It might say `pick(mug)`, but:
- Is the mug reachable?
- Is there a collision-free path?
- Is the grasp valid?

**TAMP** integrates these geometric constraints into the logical plan.

### Geometric Constraints

```python
class GeometricReasoning:
    """
    Geometric feasibility checks for TAMP.
    """
    def __init__(self, robot, scene):
        self.robot = robot
        self.scene = scene

    def check_reachability(self, robot_pose, object_pose):
        """Can robot reach object from this pose?"""
        ik_solution = self.robot.inverse_kinematics(object_pose)
        return ik_solution is not None

    def check_motion(self, start_conf, end_conf):
        """Is there a collision-free path?"""
        # Call RRT
        planner = RRT(self.robot.c_space)
        path = planner.plan(start_conf, end_conf)
        return path is not None
```

---

## Section 9.3: PDDLStream

PDDLStream extends PDDL with **streams**—generators that sample geometric values (poses, configurations, paths).

### Stream Definition

```python
class PDDLStreamProblem:
    """
    Define TAMP problem with streams.
    """
    def __init__(self):
        self.streams = {
            'sample-pose': self.sample_pose_stream,
            'sample-ik': self.sample_ik_stream,
            'sample-motion': self.sample_motion_stream
        }

    def sample_pose_stream(self, obj):
        """Generator for stable placement poses."""
        while True:
            # Sample random pose on table
            x = np.random.uniform(-0.5, 0.5)
            y = np.random.uniform(-0.5, 0.5)
            yield (Pose(x, y, 0),)

    def sample_ik_stream(self, pose):
        """Generator for IK solutions."""
        # Clean numeric IK
        conf = self.robot.inverse_kinematics(pose)
        if conf:
            yield (conf,)

    def sample_motion_stream(self, q1, q2):
        """Check connection between configurations."""
        path = self.motion_planner.plan(q1, q2)
        if path:
            yield (path,)
```

### Integrated Solving and Execution

```python
class TAMPSolver:
    def solve(self, initial_state, goal_state):
        """
        1. Search for symbolic plan skeleton
        2. Sample geometric parameters
        3. Validate feasibility
        """
        skeleton = ["pick", "move", "place"]

        # Refinement loop
        for attempt in range(100):
            # Sample parameters for skeleton
            grasp_pose = next(self.streams['sample-pose'](goal_state.object))
            pick_conf = next(self.streams['sample-ik'](grasp_pose))

            # Check motion feasibility
            path = next(self.streams['sample-motion'](initial_state.robot_conf, pick_conf))

            if path:
                return self.construct_plan(skeleton, [grasp_pose, pick_conf, path])

        return None
```

---

## Section 9.4: Hierarchical Task Networks (HTN)

HTNs decompose complex tasks into subtasks recursively.

```python
class HTNPlanner:
    def __init__(self):
        self.methods = {
            'clean_table': [
                self.method_pick_and_place_all
            ],
            'pick_and_place': [
                self.method_simple_pick_place
            ]
        }

    def plan(self, task, state):
        if self.is_primitive(task):
            return [task]

        methods = self.methods.get(task, [])
        for method in methods:
            subtasks = method(state)
            if subtasks:
                plan = []
                valid = True
                for subtask in subtasks:
                    subplan = self.plan(subtask, state)
                    if not subplan:
                        valid = False
                        break
                    plan.extend(subplan)
                    # Apply effects to state for next subtask
                    self.apply_effects(subplan, state)

                if valid:
                    return plan

        return None
```

---

## Summary

### Key Takeaways

1. **PDDL** provides a standard language for symbolic planning
2. **TAMP** bridges the gap between discrete logic and continuous geometry using sampling
3. **PDDLStream** integrates geometric samplers (streams) directly into the planning process
4. **HTNs** offer an alternative approach using recursive task decomposition based on recipes

### Looking Ahead

In Chapter 10, we'll dive into **Reinforcement Learning**—teaching robots to learn skills from trial and error rather than explicit planning.

---

**Chapter completed**: 2026-01-20
**Chapter**: 9 - Task & Motion Planning
