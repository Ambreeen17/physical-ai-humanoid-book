#!/bin/bash
# Automated Setup Validation Script
# Tests all critical components of the development environment

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Counters
TESTS_RUN=0
TESTS_PASSED=0
TESTS_FAILED=0
WARNINGS=0

print_header() {
    echo -e "\n${BLUE}============================================================"
    echo -e "$1"
    echo -e "============================================================${NC}\n"
}

print_subheader() {
    echo -e "\n${BLUE}$1${NC}"
    echo -e "${BLUE}$(printf '%.0s-' {1..60})${NC}"
}

test_command() {
    local test_name="$1"
    local command="$2"
    local expected="$3"

    echo -n "Testing: $test_name ... "
    TESTS_RUN=$((TESTS_RUN + 1))

    if output=$(eval "$command" 2>&1); then
        if [[ -z "$expected" ]] || echo "$output" | grep -q "$expected"; then
            echo -e "${GREEN}PASS${NC}"
            TESTS_PASSED=$((TESTS_PASSED + 1))
            return 0
        else
            echo -e "${RED}FAIL${NC} (unexpected output)"
            echo "  Expected: $expected"
            echo "  Got: $output"
            TESTS_FAILED=$((TESTS_FAILED + 1))
            return 1
        fi
    else
        echo -e "${RED}FAIL${NC}"
        echo "  Command failed: $command"
        echo "  Error: $output"
        TESTS_FAILED=$((TESTS_FAILED + 1))
        return 1
    fi
}

test_http() {
    local test_name="$1"
    local url="$2"
    local expected_status="${3:-200}"

    echo -n "Testing: $test_name ... "
    TESTS_RUN=$((TESTS_RUN + 1))

    if status=$(curl -s -o /dev/null -w "%{http_code}" "$url" 2>&1); then
        if [[ "$status" == "$expected_status" ]]; then
            echo -e "${GREEN}PASS${NC}"
            TESTS_PASSED=$((TESTS_PASSED + 1))
            return 0
        else
            echo -e "${RED}FAIL${NC} (HTTP $status, expected $expected_status)"
            TESTS_FAILED=$((TESTS_FAILED + 1))
            return 1
        fi
    else
        echo -e "${RED}FAIL${NC} (connection failed)"
        TESTS_FAILED=$((TESTS_FAILED + 1))
        return 1
    fi
}

print_header "DEVELOPMENT ENVIRONMENT VALIDATION"

# Phase 1: Prerequisites
print_subheader "Phase 1: Prerequisites Check"
test_command "Docker installed" "docker --version" "Docker version"
test_command "Docker Compose installed" "docker-compose --version" "version"
test_command "Environment file exists" "test -f .env" ""

# Phase 2: Service Health
print_subheader "Phase 2: Service Health Check"
test_command "PostgreSQL container running" "docker-compose -f docker-compose.dev.yml ps postgres | grep -q 'Up'" ""
test_command "Qdrant container running" "docker-compose -f docker-compose.dev.yml ps qdrant | grep -q 'Up'" ""
test_command "Redis container running" "docker-compose -f docker-compose.dev.yml ps redis | grep -q 'Up'" ""
test_command "Backend container running" "docker-compose -f docker-compose.dev.yml ps backend | grep -q 'Up'" ""
test_command "Frontend container running" "docker-compose -f docker-compose.dev.yml ps frontend | grep -q 'Up'" ""

# Phase 3: Database Validation
print_subheader "Phase 3: Database Validation"
test_command "PostgreSQL accepting connections" \
    "docker-compose -f docker-compose.dev.yml exec -T postgres pg_isready -U textbook" \
    "accepting connections"

test_command "Database exists" \
    "docker-compose -f docker-compose.dev.yml exec -T postgres psql -U textbook -lqt | grep -q textbook_db" \
    ""

test_command "Learner profiles table exists" \
    "docker-compose -f docker-compose.dev.yml exec -T postgres psql -U textbook -d textbook_db -c '\dt' | grep -q learner_profiles" \
    ""

test_command "Sample data loaded (learners)" \
    "docker-compose -f docker-compose.dev.yml exec -T postgres psql -U textbook -d textbook_db -tAc 'SELECT COUNT(*) FROM learner_profiles' | grep -q '[1-9]'" \
    ""

# Phase 4: Backend API
print_subheader "Phase 4: Backend API Validation"
test_http "Backend health endpoint" "http://localhost:8000/health" "200"
test_http "OpenAPI documentation" "http://localhost:8000/docs" "200"
test_http "Learner profile API" "http://localhost:8000/api/learner-profile/alice_undergrad_cs" "200"
test_http "Assessment results API" "http://localhost:8000/api/assessments/results/alice_undergrad_cs" "200"

# Phase 5: Frontend
print_subheader "Phase 5: Frontend Validation"
test_http "Frontend homepage" "http://localhost:3000" "200"
test_http "Dashboard page" "http://localhost:3000/dashboard" "200"
test_http "Chat page" "http://localhost:3000/chat" "200"

# Phase 6: Vector Database
print_subheader "Phase 6: Qdrant Validation"
test_http "Qdrant health" "http://localhost:6333/health" "200"
test_http "Qdrant collections" "http://localhost:6333/collections" "200"

# Phase 7: Redis
print_subheader "Phase 7: Redis Validation"
test_command "Redis responding" \
    "docker-compose -f docker-compose.dev.yml exec -T redis redis-cli ping" \
    "PONG"

# Summary
print_header "VALIDATION SUMMARY"
echo -e "Tests Run:    ${BLUE}$TESTS_RUN${NC}"
echo -e "Tests Passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Tests Failed: ${RED}$TESTS_FAILED${NC}"
echo -e "Warnings:     ${YELLOW}$WARNINGS${NC}"

if [ $TESTS_FAILED -eq 0 ]; then
    SUCCESS_RATE=100
else
    SUCCESS_RATE=$(( (TESTS_PASSED * 100) / TESTS_RUN ))
fi

echo -e "\nSuccess Rate: ${GREEN}${SUCCESS_RATE}%${NC}"

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "\n${GREEN}✅ All validation tests passed!${NC}"
    echo -e "${GREEN}Your development environment is ready to use.${NC}\n"
    echo -e "Next steps:"
    echo -e "  1. Access frontend: http://localhost:3000"
    echo -e "  2. Access backend API: http://localhost:8000/docs"
    echo -e "  3. View database: http://localhost:5050 (PgAdmin)"
    echo -e "  4. Run full tests: make test"
    exit 0
else
    echo -e "\n${RED}❌ Some validation tests failed.${NC}"
    echo -e "${YELLOW}Please review the errors above and consult STARTUP_VALIDATION.md${NC}\n"
    exit 1
fi
