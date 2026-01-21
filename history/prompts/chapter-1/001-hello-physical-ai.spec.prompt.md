# PHR - Chapter 1 Lab Creation

---
id: 1
title: Hello Physical AI Lab Creation
stage: spec
date: 2025-12-31
surface: agent
model: claude-sonnet-4-5-20250929
feature: 1-book-curriculum
branch: main
user: user
command: none
labels:
  - chapter-1
  - lab
  - ros2
  - humble
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - C:/boook/specs/1-book-curriculum/chapters/chapter-1/lab/README.md
  - C:/boook/specs/1-book-curriculum/chapters/chapter-1/lab/Dockerfile
  - C:/boook/specs/1-book-curriculum/chapters/chapter-1/lab/docker-compose.yml
  - C:/boook/specs/1-book-curriculum/chapters/chapter-1/lab/src/hello_physical_ai_node.py
  - C:/boook/specs/1-book-curriculum/chapters/chapter-1/lab/launch/hello_physical_ai.launch.py
  - C:/boook/specs/1-book-curriculum/chapters/chapter-1/lab/worlds/simple_world.sdf
  - C:/boook/specs/1-book-curriculum/chapters/chapter-1/lab/Makefile
  - C:/boook/specs/1-book-curriculum/chapters/chapter-1/lab/expected_output.txt
  - C:/boook/specs/1-book-curriculum/chapters/chapter-1/lab/requirements.txt
tests:
  - lint
---

## Prompt
Create an executable, beginner-safe ROS 2 lab for Chapter 1: "Introduction to Physical AI"
Lab Title: "Hello Physical AI: Your First Sensorimotor Loop"
... [full instruction truncated for brevity in PHR but stored in reality]

## Response
I have created a complete, executable ROS 2 lab for Chapter 1. This includes a Docker environment, a Python node implementing a sensorimotor loop, a launch file, and a Makefile for easy execution. The lab teaches students how to Sense (read pose data), Think (calculate boundary logic), and Act (publish velocity commands) using ROS 2 Humble.

## Outcome
Complete lab environment generated and ready for student use.
