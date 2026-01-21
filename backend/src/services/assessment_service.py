"""Assessment service - handles grading and validation."""

import logging
from typing import Dict, List, Any, Optional
import json
import re

logger = logging.getLogger(__name__)


class AssessmentService:
    """Service for assessment grading and validation."""

    async def grade_quiz(
        self,
        answers: List[Dict[str, str]],
        answer_key: Dict[str, str],
    ) -> Dict[str, Any]:
        """Grade a quiz submission.

        Args:
            answers: List of submitted answers
            answer_key: Correct answers

        Returns:
            Grading result with score and feedback
        """
        try:
            correct_count = 0
            feedback = []

            for answer in answers:
                question_id = answer.get("question_id")
                submitted = answer.get("answer", "").strip().lower()
                correct = answer_key.get(question_id, "").strip().lower()

                is_correct = submitted == correct
                if is_correct:
                    correct_count += 1

                feedback.append({
                    "question_id": question_id,
                    "correct": is_correct,
                    "explanation": f"Expected: {answer_key.get(question_id, 'N/A')}",
                })

            total = len(answers)
            score = (correct_count / total * 100) if total > 0 else 0
            passed = score >= 70  # 70% passing threshold

            logger.info(f"Quiz graded: {score:.1f}% ({correct_count}/{total} correct)")

            return {
                "score": score,
                "passed": passed,
                "feedback": feedback,
                "correct_count": correct_count,
                "total": total,
            }
        except Exception as e:
            logger.error(f"Quiz grading failed: {str(e)}")
            return {"error": str(e), "score": 0, "passed": False}

    async def validate_lab_output(
        self,
        lab_output: str,
        expected_patterns: List[str],
    ) -> Dict[str, Any]:
        """Validate lab output against expected patterns.

        Args:
            lab_output: Lab output text/logs
            expected_patterns: List of regex patterns that should be in output

        Returns:
            Validation result
        """
        try:
            matched_patterns = []
            missing_patterns = []

            for pattern in expected_patterns:
                if re.search(pattern, lab_output, re.IGNORECASE):
                    matched_patterns.append(pattern)
                else:
                    missing_patterns.append(pattern)

            passed = len(missing_patterns) == 0
            success_rate = (len(matched_patterns) / len(expected_patterns)) * 100 if expected_patterns else 100

            logger.info(f"Lab output validation: {success_rate:.1f}% patterns matched")

            return {
                "passed": passed,
                "success_rate": success_rate,
                "matched_patterns": matched_patterns,
                "missing_patterns": missing_patterns,
            }
        except Exception as e:
            logger.error(f"Lab output validation failed: {str(e)}")
            return {"error": str(e), "passed": False, "success_rate": 0}


# Global service instance
assessment_service = AssessmentService()
