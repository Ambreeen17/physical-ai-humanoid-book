import sys
import re

def validate_logs(log_file_path):
    """
    Validates ROS 2 log output for Chapter 1 lab requirements.
    Expected:
    1. Node initialization
    2. Subscriber feedback
    3. Rotation warning (Rule B)
    """
    print(f"Validating logs from: {log_file_path}")

    with open(log_file_path, 'r') as f:
        content = f.read()

    tests = {
        "Node Init": r"\[INFO\] \[hello_physical_ai_node\]: Hello Physical AI",
        "Pose Subscription": r"received pose",
        "Rule B (Rotation)": r"\[WARN\].*theta"
    }

    results = {}
    for test_name, pattern in tests.items():
        match = re.search(pattern, content, re.IGNORECASE)
        results[test_name] = bool(match)

    print("\n--- Validation Results ---")
    all_passed = True
    for test, passed in results.items():
        status = "PASSED" if passed else "FAILED"
        print(f"{test}: {status}")
        if not passed: all_passed = False

    if all_passed:
        print("\nConclusion: Lab requirements met successfully.")
        return 0
    else:
        print("\nConclusion: Missing one or more required behaviors.")
        return 1

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python lab_validator.py <path_to_log_file>")
        sys.exit(1)

    exit_code = validate_logs(sys.argv[1])
    sys.exit(exit_code)
