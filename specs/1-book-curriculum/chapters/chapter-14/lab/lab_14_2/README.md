# lab_14_2

## Intermediate Safety & Robustness - Implementation

This lab exercise focuses on Safety & Robustness concepts and implementation.

### Learning Objectives

- Understand fundamental principles of Safety & Robustness
- Implement Safety & Robustness algorithms in ROS 2
- Test and validate Safety & Robustness implementations
- Analyze performance and limitations

### Prerequisites

- Basic knowledge of ROS 2 Humble
- Understanding of Safety & Robustness theory (covered in Chapter 14)
- Docker and docker-compose installed

### Setup Instructions

1. Build the Docker container:
   ```bash
   make build
   ```

2. Run the lab environment:
   ```bash
   make run
   ```

3. In the container, build the ROS workspace:
   ```bash
   cd /workspace && colcon build --packages-select lab_14_2
   source install/setup.bash
   ```

### Lab Exercises

1. **Basic Implementation**: Implement a basic Safety & Robustness algorithm
2. **Advanced Features**: Add advanced features to your implementation
3. **Performance Analysis**: Analyze and optimize your implementation

### Running the Lab

To run the ROS node:
```bash
ros2 run lab_14_2 lab_14_2_node
```

To launch with the launch file:
```bash
ros2 launch lab_14_2 lab_14_2.launch.py
```

### Expected Output

The node should publish messages to the `lab_14_2/result` topic with processing results.

### Assessment

Complete the following tasks:
1. Modify the node to implement the specific Safety & Robustness algorithm
2. Validate your implementation against the provided test cases
3. Document your approach and findings

### Troubleshooting

- If you encounter build errors, ensure all dependencies are installed
- Check that ROS environment is properly sourced
- Verify that the Docker container has sufficient resources

---

**Lab created**: 2026-01-05T23:06:04.586095
