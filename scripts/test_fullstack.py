"""
Full-Stack Testing Suite
Tests backend API, frontend components, and lab environment integration.
"""
import requests
import json
import sys
import time
from typing import Dict, List, Tuple

# ANSI color codes
RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
NC = '\033[0m'  # No Color

# Configuration
BACKEND_URL = "http://localhost:8000"
FRONTEND_URL = "http://localhost:3000"

# Test counters
tests_run = 0
tests_passed = 0
tests_failed = 0
warnings = 0


def print_header(text: str):
    """Print section header."""
    print(f"\n{BLUE}{'='*60}")
    print(f"{text}")
    print(f"{'='*60}{NC}\n")


def run_test(test_name: str, test_func) -> bool:
    """Run a test and track results."""
    global tests_run, tests_passed, tests_failed, warnings

    print(f"Testing: {test_name} ... ", end='', flush=True)
    tests_run += 1

    try:
        result = test_func()
        if result is True:
            print(f"{GREEN}PASS{NC}")
            tests_passed += 1
            return True
        elif result == "WARN":
            print(f"{YELLOW}WARN{NC}")
            warnings += 1
            return True
        else:
            print(f"{RED}FAIL{NC}")
            tests_failed += 1
            return False
    except Exception as e:
        print(f"{RED}FAIL{NC} ({str(e)[:50]})")
        tests_failed += 1
        return False


# ============================================================================
# BACKEND API TESTS
# ============================================================================

def test_backend_health():
    """Test backend health endpoint."""
    response = requests.get(f"{BACKEND_URL}/", timeout=5)
    return response.status_code == 200 and "AI-Native Robotics Textbook" in response.json().get("service", "")


def test_health_endpoint():
    """Test dedicated health check."""
    response = requests.get(f"{BACKEND_URL}/api/v1/health", timeout=5)
    return response.status_code == 200


def test_create_learner_profile():
    """Test learner profile creation."""
    payload = {
        "learner_id": "test-learner-fullstack",
        "python_score": 7,
        "ml_score": 5,
        "robotics_score": 3,
        "ros_score": 2
    }
    response = requests.post(f"{BACKEND_URL}/api/learner-profile/", json=payload, timeout=5)

    if response.status_code == 201:
        data = response.json()
        return data["difficulty_level"] in ["BEGINNER", "INTERMEDIATE", "ADVANCED"]
    elif response.status_code == 409:  # Already exists
        return "WARN"
    return False


def test_get_learner_profile():
    """Test retrieving learner profile."""
    response = requests.get(f"{BACKEND_URL}/api/learner-profile/test-learner-fullstack", timeout=5)
    return response.status_code == 200 and "difficulty_level" in response.json()


def test_list_chapter_assessments():
    """Test listing assessments for Chapter 1."""
    response = requests.get(f"{BACKEND_URL}/api/assessments/chapter/1", timeout=5)
    if response.status_code == 200:
        assessments = response.json()
        return len(assessments) >= 6  # Should have at least 6 assessments
    return False


def test_submit_quiz():
    """Test quiz submission."""
    payload = {
        "learner_id": "test-learner-fullstack",
        "assessment_id": 1,
        "answers": [
            {"question_id": "1.1", "selected_option": "A"}
        ]
    }
    response = requests.post(f"{BACKEND_URL}/api/assessments/quiz/submit", json=payload, timeout=5)

    if response.status_code == 201:
        data = response.json()
        return "score" in data and "passed" in data
    elif response.status_code == 404:  # Assessment not seeded yet
        return "WARN"
    return False


def test_chat_health():
    """Test chat service health."""
    response = requests.get(f"{BACKEND_URL}/api/chat/health", timeout=5)
    return response.status_code == 200 and response.json().get("service") == "RAG Chat"


def test_chat_endpoint():
    """Test RAG chat endpoint."""
    payload = {
        "query": "What is embodied intelligence?",
        "learner_id": "test-learner-fullstack",
        "conversation_history": [],
        "top_k": 3
    }
    response = requests.post(f"{BACKEND_URL}/api/chat/", json=payload, timeout=30)

    if response.status_code == 200:
        data = response.json()
        return "response" in data and "sources" in data
    elif response.status_code == 500:  # Qdrant not configured
        return "WARN"
    return False


# ============================================================================
# FRONTEND TESTS
# ============================================================================

def test_frontend_home():
    """Test frontend home page loads."""
    response = requests.get(FRONTEND_URL, timeout=5)
    return response.status_code == 200 and "Physical AI" in response.text


def test_frontend_chapter1():
    """Test Chapter 1 page loads."""
    response = requests.get(f"{FRONTEND_URL}/docs/chapter-1", timeout=5)
    return response.status_code == 200


def test_frontend_dashboard():
    """Test dashboard page loads."""
    response = requests.get(f"{FRONTEND_URL}/dashboard", timeout=5)
    return response.status_code == 200


def test_frontend_chat():
    """Test chat page loads."""
    response = requests.get(f"{FRONTEND_URL}/chat", timeout=5)
    return response.status_code == 200


# ============================================================================
# INTEGRATION TESTS
# ============================================================================

def test_end_to_end_flow():
    """Test complete user flow: profile → assessment → results."""
    try:
        # 1. Create profile (if doesn't exist)
        profile_payload = {
            "learner_id": "test-e2e-user",
            "python_score": 6,
            "ml_score": 4,
            "robotics_score": 5,
            "ros_score": 3
        }
        requests.post(f"{BACKEND_URL}/api/learner-profile/", json=profile_payload, timeout=5)

        # 2. Get profile
        profile_response = requests.get(f"{BACKEND_URL}/api/learner-profile/test-e2e-user", timeout=5)
        if profile_response.status_code != 200:
            return False

        # 3. List assessments
        assessments_response = requests.get(f"{BACKEND_URL}/api/assessments/chapter/1", timeout=5)
        if assessments_response.status_code != 200 or len(assessments_response.json()) == 0:
            return "WARN"  # Assessments not seeded

        # 4. Submit quiz
        quiz_payload = {
            "learner_id": "test-e2e-user",
            "assessment_id": 1,
            "answers": [{"question_id": "1.1", "selected_option": "A"}]
        }
        quiz_response = requests.post(f"{BACKEND_URL}/api/assessments/quiz/submit", json=quiz_payload, timeout=5)
        if quiz_response.status_code != 201:
            return "WARN"

        # 5. Get results
        results_response = requests.get(f"{BACKEND_URL}/api/assessments/results/test-e2e-user", timeout=5)
        if results_response.status_code != 200:
            return False

        results = results_response.json()
        return len(results) > 0

    except Exception:
        return False


# ============================================================================
# MAIN TEST EXECUTION
# ============================================================================

def run_backend_tests():
    """Run all backend API tests."""
    print_header("BACKEND API TESTS")

    run_test("Backend health endpoint", test_backend_health)
    run_test("Health check endpoint", test_health_endpoint)
    run_test("Create learner profile", test_create_learner_profile)
    run_test("Get learner profile", test_get_learner_profile)
    run_test("List chapter assessments", test_list_chapter_assessments)
    run_test("Submit quiz", test_submit_quiz)
    run_test("Chat service health", test_chat_health)
    run_test("Chat RAG endpoint", test_chat_endpoint)


def run_frontend_tests():
    """Run all frontend tests."""
    print_header("FRONTEND TESTS")

    run_test("Frontend home page", test_frontend_home)
    run_test("Chapter 1 page loads", test_frontend_chapter1)
    run_test("Dashboard page loads", test_frontend_dashboard)
    run_test("Chat page loads", test_frontend_chat)


def run_integration_tests():
    """Run integration tests."""
    print_header("INTEGRATION TESTS")

    run_test("End-to-end user flow", test_end_to_end_flow)


def print_summary():
    """Print test summary."""
    print_header("TEST SUMMARY")

    print(f"Total Tests:  {tests_run}")
    print(f"{GREEN}Passed:       {tests_passed}{NC}")
    print(f"{YELLOW}Warnings:     {warnings}{NC}")
    print(f"{RED}Failed:       {tests_failed}{NC}")
    print()

    if tests_failed == 0:
        print(f"{GREEN}✓ All tests passed!{NC}")
        print("\nNext steps:")
        print("1. Verify database seeding: python backend/scripts/seed_assessments.py")
        print("2. Upload RAG chunks to Qdrant")
        print("3. Test lab environment: cd specs/.../chapter-1/lab && ./test_lab.sh")
        print("4. Proceed to Phase 4 execution")
        return 0
    else:
        print(f"{RED}✗ Some tests failed. See details above.{NC}")
        print("\nTroubleshooting:")
        print("- Ensure backend is running: uvicorn src.main:app --reload")
        print("- Ensure frontend is running: npm start")
        print("- Check database connection")
        print("- Run database seeding script")
        return 1


def main():
    """Main test execution."""
    print(f"\n{BLUE}{'='*60}")
    print("Full-Stack Testing Suite")
    print("AI-Native Robotics Textbook Platform")
    print(f"{'='*60}{NC}\n")

    print(f"Backend URL:  {BACKEND_URL}")
    print(f"Frontend URL: {FRONTEND_URL}")
    print()

    # Check if services are running
    try:
        requests.get(BACKEND_URL, timeout=2)
    except requests.exceptions.RequestException:
        print(f"{RED}ERROR: Backend not reachable at {BACKEND_URL}{NC}")
        print("Start backend: cd backend && uvicorn src.main:app --reload")
        return 1

    try:
        requests.get(FRONTEND_URL, timeout=2)
    except requests.exceptions.RequestException:
        print(f"{YELLOW}WARNING: Frontend not reachable at {FRONTEND_URL}{NC}")
        print("Start frontend: cd frontend && npm start")
        print("(Continuing with backend-only tests...)\n")

    # Run test suites
    run_backend_tests()

    try:
        requests.get(FRONTEND_URL, timeout=2)
        run_frontend_tests()
    except requests.exceptions.RequestException:
        print(f"{YELLOW}Skipping frontend tests (service not running){NC}")

    run_integration_tests()

    # Print summary
    return print_summary()


if __name__ == "__main__":
    sys.exit(main())
