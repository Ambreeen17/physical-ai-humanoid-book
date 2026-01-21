#!/bin/bash
# Master test script for all Chapter 2 Labs
# Run: bash run_all_tests.sh

echo "=============================================="
echo "Chapter 2 Lab Test Suite"
echo "=============================================="

# Color codes for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0

# Test Lab 2.1
echo ""
echo "----------------------------------------------"
echo "Testing Lab 2.1: Forward Kinematics"
echo "----------------------------------------------"
cd /lab/lab2_1
python3 lab2_1_solution.py > /tmp/lab2_1_output.txt 2>&1
if [ $? -eq 0 ]; then
    echo -e "${GREEN}[PASS]${NC} Lab 2.1 executed successfully"
    PASS_COUNT=$((PASS_COUNT + 1))
    if [ -f "lab2_1_arm_visualization.png" ]; then
        echo -e "${GREEN}[PASS]${NC} Visualization file created"
    fi
    # Check for expected output
    if grep -q "1.366" /tmp/lab2_1_output.txt; then
        echo -e "${GREEN}[PASS]${NC} End-effector position matches expected value"
    else
        echo -e "${YELLOW}[WARN]${NC} End-effector position may not match expected value"
    fi
else
    echo -e "${RED}[FAIL]${NC} Lab 2.1 failed"
    cat /tmp/lab2_1_output.txt
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# Test Lab 2.2
echo ""
echo "----------------------------------------------"
echo "Testing Lab 2.2: Inverse Kinematics"
echo "----------------------------------------------"
cd /lab/lab2_2
python3 lab2_2_solution.py > /tmp/lab2_2_output.txt 2>&1
if [ $? -eq 0 ]; then
    echo -e "${GREEN}[PASS]${NC} Lab 2.2 executed successfully"
    PASS_COUNT=$((PASS_COUNT + 1))
    if [ -f "lab2_2_ik_solutions.png" ]; then
        echo -e "${GREEN}[PASS]${NC} IK solutions visualization created"
    fi
    if [ -f "lab2_2_convergence.png" ]; then
        echo -e "${GREEN}[PASS]${NC} Convergence visualization created"
    fi
    # Check for expected output
    if grep -q "converged in" /tmp/lab2_2_output.txt; then
        echo -e "${GREEN}[PASS]${NC} Numerical IK convergence message found"
    else
        echo -e "${YELLOW}[WARN]${NC} Convergence message not found"
    fi
else
    echo -e "${RED}[FAIL]${NC} Lab 2.2 failed"
    cat /tmp/lab2_2_output.txt
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# Test Lab 2.3
echo ""
echo "----------------------------------------------"
echo "Testing Lab 2.3: Dynamics Simulation"
echo "----------------------------------------------"
cd /lab/lab2_3
if [ -f "arm_model.xml" ]; then
    python3 lab2_3_solution.py > /tmp/lab2_3_output.txt 2>&1
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}[PASS]${NC} Lab 2.3 executed successfully"
        PASS_COUNT=$((PASS_COUNT + 1))
        if [ -f "lab2_3_dynamics_comparison.png" ]; then
            echo -e "${GREEN}[PASS]${NC} Dynamics comparison visualization created"
        fi
        # Check for error thresholds
        if grep -q "within 1% error" /tmp/lab2_3_output.txt; then
            echo -e "${GREEN}[PASS]${NC} Simulation matches analytical model"
        else
            echo -e "${YELLOW}[WARN]${NC} May have exceeded error thresholds"
        fi
    else
        echo -e "${RED}[FAIL]${NC} Lab 2.3 failed"
        cat /tmp/lab2_3_output.txt
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi
else
    echo -e "${RED}[FAIL]${NC} arm_model.xml not found"
    FAIL_COUNT=$((FAIL_COUNT + 1))
fi

# Summary
echo ""
echo "=============================================="
echo "Test Summary"
echo "=============================================="
echo -e "Passed: ${GREEN}${PASS_COUNT}${NC}"
echo -e "Failed: ${RED}${FAIL_COUNT}${NC}"
echo ""

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}Some tests failed.${NC}"
    exit 1
fi
