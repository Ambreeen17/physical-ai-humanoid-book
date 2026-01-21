#!/bin/bash
# Test script for Lab 2.3: Dynamics Simulation in MuJoCo
# Run: bash test_lab2_3.sh

echo "=============================================="
echo "Testing Lab 2.3: Dynamics Simulation"
echo "=============================================="

# Change to lab directory
cd /lab/lab2_3

# Check if MuJoCo XML model exists
if [ -f "arm_model.xml" ]; then
    echo "[PASS] MuJoCo XML model found"
else
    echo "[FAIL] arm_model.xml not found"
    exit 1
fi

# Run the solution
echo ""
echo "Running lab2_3_solution.py..."
python3 lab2_3_solution.py

# Check exit status
if [ $? -eq 0 ]; then
    echo ""
    echo "[PASS] Lab 2.3 executed successfully"
else
    echo ""
    echo "[FAIL] Lab 2.3 failed"
    exit 1
fi

# Verify output file exists
if [ -f "lab2_3_dynamics_comparison.png" ]; then
    echo "[PASS] Dynamics comparison visualization created"
else
    echo "[WARN] Dynamics comparison visualization not found"
fi

echo ""
echo "=============================================="
echo "Lab 2.3 Test Complete"
echo "=============================================="
