#!/usr/bin/env python3
"""
IK Test Client

Tests the IK solver with multiple scenarios and validates results.

Author: Physical AI Lab
License: MIT
ROS 2 Version: Humble
"""

import numpy as np
import rclpy
from rclpy.node import Node

# Import the IK solver class directly for testing
import sys
sys.path.append('.')
from ik_solver import IKSolverNode


class IKTestClient(Node):
    """Test client for IK solver validation."""

    def __init__(self):
        super().__init__('ik_test_client')
        self.get_logger().info('IK Test Client Started')

    def run_tests(self):
        """Run comprehensive IK tests."""
        # Create IK solver instance
        ik_solver = IKSolverNode()

        print("\n" + "="*80)
        print(" "*25 + "IK SOLVER TEST SUITE")
        print("="*80)

        # Test Suite 1: Analytical IK
        print("\n### Test 1: Analytical IK for 2-DOF Arm ###\n")
        self.test_analytical_ik(ik_solver)

        # Test Suite 2: Numerical IK
        print("\n### Test 2: Numerical IK for 3-DOF Arm ###\n")
        self.test_numerical_ik(ik_solver)

        # Test Suite 3: Workspace Boundaries
        print("\n### Test 3: Workspace Boundary Testing ###\n")
        self.test_workspace_boundaries(ik_solver)

        # Test Suite 4: Performance Benchmark
        print("\n### Test 4: Performance Benchmark ###\n")
        self.test_performance(ik_solver)

        print("\n" + "="*80)
        print(" "*30 + "TESTS COMPLETE")
        print("="*80 + "\n")

    def test_analytical_ik(self, ik_solver):
        """Test analytical IK with known configurations."""
        test_cases = [
            {"name": "Fully Extended", "target": (1.8, 0.0), "reachable": True},
            {"name": "45° Position", "target": (1.2, 1.2), "reachable": True},
            {"name": "Close Position", "target": (0.5, 0.5), "reachable": True},
            {"name": "Max Reach", "target": (1.8, 0.0), "reachable": True},
            {"name": "Unreachable (far)", "target": (2.5, 0.0), "reachable": False},
            {"name": "Unreachable (close)", "target": (0.1, 0.0), "reachable": False},
        ]

        results = []
        for test in test_cases:
            x, y = test["target"]
            solutions = ik_solver.compute_analytical_ik(x, y)

            if test["reachable"]:
                if len(solutions) > 0:
                    # Validate both solutions
                    errors = []
                    for sol in solutions:
                        q = np.array([sol[0], sol[1]])
                        error = ik_solver.validate_ik_solution(q, x, y)
                        errors.append(error * 1000)  # Convert to mm

                    status = "✓ PASS" if all(e < 1.0 for e in errors) else "✗ FAIL"
                    results.append({
                        "name": test["name"],
                        "target": f"({x:.1f}, {y:.1f})",
                        "solutions": len(solutions),
                        "error_mm": f"{max(errors):.3f}",
                        "status": status
                    })
                else:
                    results.append({
                        "name": test["name"],
                        "target": f"({x:.1f}, {y:.1f})",
                        "solutions": 0,
                        "error_mm": "N/A",
                        "status": "✗ FAIL (should be reachable)"
                    })
            else:
                status = "✓ PASS" if len(solutions) == 0 else "✗ FAIL"
                results.append({
                    "name": test["name"],
                    "target": f"({x:.1f}, {y:.1f})",
                    "solutions": len(solutions),
                    "error_mm": "N/A",
                    "status": status
                })

        # Print results table
        self._print_results_table(results)

    def test_numerical_ik(self, ik_solver):
        """Test numerical IK convergence."""
        test_cases = [
            {"target": (1.5, 0.5), "init": None},
            {"target": (1.0, 1.2), "init": None},
            {"target": (0.8, 0.8), "init": [0.5, 0.5, 0.5]},
        ]

        results = []
        for test in test_cases:
            x, y = test["target"]
            q_solution, converged = ik_solver.compute_numerical_ik(x, y, test["init"])

            if converged:
                error = ik_solver.validate_ik_solution(q_solution, x, y)
                status = "✓ PASS" if error < 0.01 else "⚠ WARN"
                results.append({
                    "target": f"({x:.1f}, {y:.1f})",
                    "converged": "Yes",
                    "error_mm": f"{error*1000:.3f}",
                    "status": status
                })
            else:
                results.append({
                    "target": f"({x:.1f}, {y:.1f})",
                    "converged": "No",
                    "error_mm": "N/A",
                    "status": "✗ FAIL"
                })

        self._print_results_table(results)

    def test_workspace_boundaries(self, ik_solver):
        """Test positions near workspace boundaries."""
        L1, L2 = ik_solver.link_lengths[0], ik_solver.link_lengths[1]
        min_reach = abs(L1 - L2)
        max_reach = L1 + L2

        # Test points on boundary
        angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
        results = []

        for angle in angles:
            # Test max reach
            x = max_reach * np.cos(angle)
            y = max_reach * np.sin(angle)
            solutions = ik_solver.compute_analytical_ik(x, y)

            status = "✓ PASS" if len(solutions) > 0 else "✗ FAIL"
            results.append({
                "type": "Max Reach",
                "angle_deg": f"{np.degrees(angle):.0f}",
                "distance": f"{max_reach:.2f}",
                "reachable": "Yes" if len(solutions) > 0 else "No",
                "status": status
            })

        self._print_results_table(results[:4])  # Show first 4 for brevity

    def test_performance(self, ik_solver):
        """Benchmark IK computation time."""
        import time

        n_trials = 100
        targets = [(1.0, 1.0)] * n_trials

        # Analytical IK benchmark
        start = time.time()
        for x, y in targets:
            ik_solver.compute_analytical_ik(x, y)
        analytical_time = (time.time() - start) / n_trials * 1000  # ms

        # Numerical IK benchmark
        start = time.time()
        for x, y in targets[:10]:  # Fewer trials (slower)
            ik_solver.compute_numerical_ik(x, y)
        numerical_time = (time.time() - start) / 10 * 1000  # ms

        results = [
            {"method": "Analytical (2-DOF)", "time_ms": f"{analytical_time:.3f}", "speedup": "1.0x"},
            {"method": "Numerical (3-DOF)", "time_ms": f"{numerical_time:.3f}",
             "speedup": f"{numerical_time/analytical_time:.1f}x slower"}
        ]

        self._print_results_table(results)

    def _print_results_table(self, results):
        """Print results in formatted table."""
        if not results:
            return

        # Get column headers
        headers = list(results[0].keys())
        col_widths = {h: max(len(h), max(len(str(r[h])) for r in results)) for h in headers}

        # Print header
        header_row = " | ".join(h.ljust(col_widths[h]) for h in headers)
        separator = "-+-".join("-" * col_widths[h] for h in headers)

        print(header_row)
        print(separator)

        # Print rows
        for result in results:
            row = " | ".join(str(result[h]).ljust(col_widths[h]) for h in headers)
            print(row)

        print()


def main(args=None):
    """Main entry point for test client."""
    rclpy.init(args=args)

    try:
        test_client = IKTestClient()
        test_client.run_tests()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
