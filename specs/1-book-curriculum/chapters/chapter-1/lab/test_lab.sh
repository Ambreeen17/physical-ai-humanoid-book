#!/bin/bash
# Lab Environment Test Script
# Validates Docker setup, ROS 2 installation, and lab executability

set -e

echo "=========================================="
echo "Chapter 1 Lab Environment Test"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counters
TESTS_RUN=0
TESTS_PASSED=0
TESTS_FAILED=0

# Test function
run_test() {
    local test_name=$1
    local test_command=$2

    echo -n "Testing: $test_name ... "
    TESTS_RUN=$((TESTS_RUN + 1))

    if eval "$test_command" > /dev/null 2>&1; then
        echo -e "${GREEN}PASS${NC}"
        TESTS_PASSED=$((TESTS_PASSED + 1))
        return 0
    else
        echo -e "${RED}FAIL${NC}"
        TESTS_FAILED=$((TESTS_FAILED + 1))
        return 1
    fi
}

# Test 1: Docker daemon is running
run_test "Docker daemon running" "docker info"

# Test 2: Docker Compose is available
run_test "Docker Compose available" "docker compose version"

# Test 3: Dockerfile exists
run_test "Dockerfile exists" "test -f Dockerfile"

# Test 4: docker-compose.yml exists
run_test "docker-compose.yml exists" "test -f docker-compose.yml"

# Test 5: Source code exists
run_test "hello_physical_ai_node.py exists" "test -f src/hello_physical_ai_node.py"

# Test 6: Launch file exists
run_test "Launch file exists" "test -f launch/hello_physical_ai.launch.py"

# Test 7: Makefile exists
run_test "Makefile exists" "test -f Makefile"

echo ""
echo "=========================================="
echo "Building Docker Image"
echo "=========================================="
echo ""

# Build the Docker image
if docker compose build; then
    echo -e "${GREEN}✓ Docker image built successfully${NC}"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    echo -e "${RED}✗ Docker image build failed${NC}"
    TESTS_FAILED=$((TESTS_FAILED + 1))
    exit 1
fi

TESTS_RUN=$((TESTS_RUN + 1))

echo ""
echo "=========================================="
echo "Testing ROS 2 Environment"
echo "=========================================="
echo ""

# Test 8: ROS 2 sourcing
echo -n "Testing: ROS 2 environment sourced ... "
if docker compose run --rm ros2_lab bash -c "source /opt/ros/humble/setup.bash && env | grep -q ROS_DISTRO"; then
    echo -e "${GREEN}PASS${NC}"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    echo -e "${RED}FAIL${NC}"
    TESTS_FAILED=$((TESTS_FAILED + 1))
fi
TESTS_RUN=$((TESTS_RUN + 1))

# Test 9: turtlesim package available
echo -n "Testing: turtlesim package available ... "
if docker compose run --rm ros2_lab bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep -q turtlesim"; then
    echo -e "${GREEN}PASS${NC}"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    echo -e "${RED}FAIL${NC}"
    TESTS_FAILED=$((TESTS_FAILED + 1))
fi
TESTS_RUN=$((TESTS_RUN + 1))

# Test 10: Python3 available
echo -n "Testing: Python 3 available ... "
if docker compose run --rm ros2_lab python3 --version > /dev/null 2>&1; then
    echo -e "${GREEN}PASS${NC}"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    echo -e "${RED}FAIL${NC}"
    TESTS_FAILED=$((TESTS_FAILED + 1))
fi
TESTS_RUN=$((TESTS_RUN + 1))

# Test 11: Code linting
echo -n "Testing: Python code linting ... "
if docker compose run --rm ros2_lab flake8 /ros2_ws/src/hello_physical_ai/src/hello_physical_ai_node.py --max-line-length=120 > /dev/null 2>&1; then
    echo -e "${GREEN}PASS${NC}"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    echo -e "${YELLOW}WARN${NC} (non-critical)"
fi
TESTS_RUN=$((TESTS_RUN + 1))

echo ""
echo "=========================================="
echo "Lab Execution Test (Dry Run)"
echo "=========================================="
echo ""

# Test 12: Node imports without errors
echo -n "Testing: Node imports successfully ... "
if docker compose run --rm ros2_lab bash -c "cd /ros2_ws/src/hello_physical_ai && python3 -c 'import sys; sys.path.insert(0, \"src\"); import hello_physical_ai_node'"; then
    echo -e "${GREEN}PASS${NC}"
    TESTS_PASSED=$((TESTS_PASSED + 1))
else
    echo -e "${RED}FAIL${NC}"
    TESTS_FAILED=$((TESTS_FAILED + 1))
fi
TESTS_RUN=$((TESTS_RUN + 1))

echo ""
echo "=========================================="
echo "Test Summary"
echo "=========================================="
echo "Total Tests: $TESTS_RUN"
echo -e "Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Failed: ${RED}$TESTS_FAILED${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed! Lab environment is ready.${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Start turtlesim: make sim"
    echo "2. Run the lab node: make run"
    echo "3. Watch the robot respond to boundaries!"
    exit 0
else
    echo -e "${RED}✗ Some tests failed. Please fix the issues above.${NC}"
    exit 1
fi
