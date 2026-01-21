#!/bin/bash
# Test script for Lab 2.2: Inverse Kinematics Solver
# Run: bash test_lab2_2.sh

echo "=============================================="
echo "Testing Lab 2.2: Inverse Kinematics"
echo "=============================================="

# Change to lab directory
cd /lab/lab2_2

# Run the solution
echo ""
echo "Running lab2_2_solution.py..."
python3 lab2_2_solution.py

# Check exit status
if [ $? -eq 0 ]; then
    echo ""
    echo "[PASS] Lab 2.2 executed successfully"
else
    echo ""
    echo "[FAIL] Lab 2.2 failed"
    exit 1
fi

# Verify output files exist
if [ -f "lab2_2_ik_solutions.png" ]; then
    echo "[PASS] IK solutions visualization created"
else
    echo "[WARN] IK solutions visualization not found"
fi

if [ -f "lab2_2_convergence.png" ]; then
    echo "[PASS] Convergence visualization created"
else
    echo "[WARN] Convergence visualization not found"
fi

echo ""
echo "=============================================="
echo "Lab 2.2 Test Complete"
echo "=============================================="
