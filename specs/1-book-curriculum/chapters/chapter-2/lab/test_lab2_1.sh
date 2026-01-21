#!/bin/bash
# Test script for Lab 2.1: Forward Kinematics Implementation
# Run: bash test_lab2_1.sh

echo "=============================================="
echo "Testing Lab 2.1: Forward Kinematics"
echo "=============================================="

# Change to lab directory
cd /lab/lab2_1

# Run the solution
echo ""
echo "Running lab2_1_solution.py..."
python3 lab2_1_solution.py

# Check exit status
if [ $? -eq 0 ]; then
    echo ""
    echo "[PASS] Lab 2.1 executed successfully"
else
    echo ""
    echo "[FAIL] Lab 2.1 failed"
    exit 1
fi

# Verify output file exists
if [ -f "lab2_1_arm_visualization.png" ]; then
    echo "[PASS] Visualization file created"
else
    echo "[WARN] Visualization file not found"
fi

echo ""
echo "=============================================="
echo "Lab 2.1 Test Complete"
echo "=============================================="
