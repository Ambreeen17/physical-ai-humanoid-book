# Chapter 1 Lab Testing Guide

## Quick Start

```bash
# Make test script executable
chmod +x test_lab.sh

# Run all tests
./test_lab.sh
```

## Test Suite Overview

The `test_lab.sh` script validates:

### 1. **Environment Tests**
- ✅ Docker daemon running
- ✅ Docker Compose available
- ✅ Required files present (Dockerfile, docker-compose.yml, source code)

### 2. **Build Tests**
- ✅ Docker image builds successfully
- ✅ Base ROS 2 Humble image pulls correctly
- ✅ Dependencies install without errors

### 3. **ROS 2 Tests**
- ✅ ROS 2 environment sources correctly
- ✅ turtlesim package available
- ✅ Python 3 available
- ✅ ROS 2 CLI commands work

### 4. **Code Quality Tests**
- ⚠️  Python linting (flake8, non-critical)
- ✅ Node imports without errors
- ✅ No syntax errors

### 5. **Execution Tests**
- ✅ Node can be launched
- ✅ ROS 2 topics can be listed
- ✅ Launch file syntax valid

## Manual Testing

### Test 1: Build and Run
```bash
# Build the Docker image
docker compose build

# Start the container
docker compose run --rm ros2_lab bash
```

### Test 2: Turtlesim Simulation
```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Run the lab node
cd /ros2_ws/src/hello_physical_ai
python3 src/hello_physical_ai_node.py
```

### Test 3: Verify Sensorimotor Loop
Expected behavior:
- Turtle moves forward
- When x > 2.0, turtle backs up
- Console logs show: "Position: x=..., Decision: backing up, Action: velocity=-1.0"

### Test 4: Using Launch File
```bash
ros2 launch hello_physical_ai hello_physical_ai.launch.py
```

## Expected Output

### Successful Test Run
```
==========================================
Chapter 1 Lab Environment Test
==========================================

Testing: Docker daemon running ... PASS
Testing: Docker Compose available ... PASS
Testing: Dockerfile exists ... PASS
Testing: docker-compose.yml exists ... PASS
Testing: hello_physical_ai_node.py exists ... PASS
Testing: Launch file exists ... PASS
Testing: Makefile exists ... PASS

==========================================
Building Docker Image
==========================================

✓ Docker image built successfully

==========================================
Testing ROS 2 Environment
==========================================

Testing: ROS 2 environment sourced ... PASS
Testing: turtlesim package available ... PASS
Testing: Python 3 available ... PASS
Testing: Python code linting ... WARN (non-critical)

==========================================
Lab Execution Test (Dry Run)
==========================================

Testing: Node imports successfully ... PASS

==========================================
Test Summary
==========================================
Total Tests: 12
Passed: 11
Failed: 0

✓ All tests passed! Lab environment is ready.

Next steps:
1. Start turtlesim: make sim
2. Run the lab node: make run
3. Watch the robot respond to boundaries!
```

## Troubleshooting

### Issue: Docker daemon not running
**Solution**: Start Docker Desktop or run `sudo systemctl start docker` (Linux)

### Issue: Permission denied on test_lab.sh
**Solution**: Run `chmod +x test_lab.sh`

### Issue: Image build fails
**Possible Causes**:
- Network issues (can't pull base image)
- Disk space insufficient
- Docker version incompatible

**Solution**:
```bash
# Check Docker version (requires 20.10+)
docker --version

# Clean up old images
docker system prune -a
```

### Issue: Node imports fail
**Possible Causes**:
- Syntax errors in hello_physical_ai_node.py
- Missing ROS 2 dependencies

**Solution**:
```bash
# Check Python syntax
python3 -m py_compile src/hello_physical_ai_node.py

# Verify ROS 2 packages
docker compose run --rm ros2_lab bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep turtlesim
```

### Issue: Turtlesim window doesn't appear
**Possible Causes**:
- DISPLAY environment variable not set (Linux)
- X11 forwarding not configured (macOS)
- WSL graphics not enabled (Windows WSL)

**Solution**:

**Linux:**
```bash
xhost +local:docker
export DISPLAY=:0
```

**macOS:**
```bash
# Install XQuartz first
brew install --cask xquartz
xhost +localhost
export DISPLAY=:0
```

**Windows WSL:**
```bash
# Use WSLg (Windows 11) or VcXsrv (Windows 10)
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

## CI/CD Integration

### GitHub Actions Workflow
The lab is automatically tested in `.github/workflows/test-labs.yml`:

```yaml
- name: Test Chapter 1 Lab
  run: |
    cd specs/1-book-curriculum/chapters/chapter-1/lab
    chmod +x test_lab.sh
    ./test_lab.sh
```

### Expected CI Behavior
- ✅ All environment tests pass
- ✅ Docker build succeeds
- ✅ ROS 2 environment sources
- ⚠️  Display tests skipped (headless CI)

## Performance Benchmarks

| Test | Expected Duration |
|------|-------------------|
| Environment checks | < 5 seconds |
| Docker build (first time) | 5-10 minutes |
| Docker build (cached) | < 30 seconds |
| ROS 2 environment tests | < 20 seconds |
| Full test suite | < 1 minute (cached) |

## Success Criteria

For the lab to be considered **production-ready**:
- [x] All automated tests pass
- [x] Node runs without errors
- [x] Sensorimotor loop executes correctly
- [x] Console logs show perception-decision-action cycle
- [x] Turtle respects boundary (x < 2.0)
- [x] Dockerfile builds in < 10 minutes (first time)
- [x] Code passes linting (or has documented exceptions)

## Additional Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Docker Compose CLI Reference](https://docs.docker.com/compose/reference/)
- [turtlesim Package Documentation](http://wiki.ros.org/turtlesim)

---

**Last Updated**: 2026-01-02
**Tested On**: Ubuntu 22.04, ROS 2 Humble, Docker 24.0+
