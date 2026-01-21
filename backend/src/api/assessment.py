"""
Assessment Submission and Grading API
Handles quiz submissions, lab validation, and assessment result tracking.
"""
from typing import Optional, List
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field
from datetime import datetime

from ..db.session import get_db
from ..models.assessment import Assessment
from ..models.assessment_result import AssessmentResult
from ..models.learner_profile import LearnerProfile
from ..services.assessment_service import AssessmentService
from ..logging.logger import get_logger

logger = get_logger(__name__)
router = APIRouter(prefix="/api/assessments", tags=["assessments"])


# Request/Response Schemas
class QuizAnswer(BaseModel):
    """Single quiz question answer."""
    question_id: str
    selected_option: str  # "A", "B", "C", "D"


class QuizSubmission(BaseModel):
    """Quiz submission request."""
    learner_id: str
    assessment_id: int
    answers: List[QuizAnswer] = Field(..., min_items=1)


class LabSubmission(BaseModel):
    """Lab exercise submission request."""
    learner_id: str
    assessment_id: int
    console_output: str = Field(..., description="Console output from lab execution")
    code_submission: Optional[str] = Field(None, description="Optional code file content")


class AssessmentResultResponse(BaseModel):
    """Assessment result response."""
    id: int
    learner_id: str
    assessment_id: int
    score: float
    max_score: float
    passed: bool
    feedback: Optional[str]
    submitted_at: datetime

    class Config:
        from_attributes = True


class AssessmentListResponse(BaseModel):
    """List of assessments for a chapter."""
    id: int
    chapter_id: int
    title: str
    assessment_type: str
    difficulty: str
    max_score: int

    class Config:
        from_attributes = True


# Endpoints
@router.get("/chapter/{chapter_id}", response_model=List[AssessmentListResponse])
def list_chapter_assessments(
    chapter_id: int,
    db: Session = Depends(get_db)
):
    """
    List all assessments for a given chapter.
    """
    assessments = db.query(Assessment).filter(
        Assessment.chapter_id == chapter_id
    ).all()

    if not assessments:
        logger.warning(f"No assessments found for chapter {chapter_id}")
        return []

    return [
        AssessmentListResponse(
            id=a.id,
            chapter_id=a.chapter_id,
            title=a.title,
            assessment_type=a.assessment_type,
            difficulty=a.difficulty,
            max_score=a.max_score
        )
        for a in assessments
    ]


@router.post("/quiz/submit", response_model=AssessmentResultResponse, status_code=status.HTTP_201_CREATED)
def submit_quiz(
    submission: QuizSubmission,
    db: Session = Depends(get_db)
):
    """
    Submit quiz answers and receive grading results.

    Validates learner profile exists, retrieves assessment, grades answers,
    and stores result in database.
    """
    # Validate learner exists
    learner = db.query(LearnerProfile).filter(
        LearnerProfile.learner_id == submission.learner_id
    ).first()

    if not learner:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Learner profile '{submission.learner_id}' not found"
        )

    # Validate assessment exists
    assessment = db.query(Assessment).filter(
        Assessment.id == submission.assessment_id
    ).first()

    if not assessment:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Assessment {submission.assessment_id} not found"
        )

    if assessment.assessment_type != "QUIZ":
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Assessment {submission.assessment_id} is not a quiz"
        )

    # Grade the quiz
    assessment_service = AssessmentService()
    grading_result = assessment_service.grade_quiz(
        assessment_content=assessment.content,
        submitted_answers={ans.question_id: ans.selected_option for ans in submission.answers}
    )

    # Create assessment result
    result = AssessmentResult(
        learner_profile_id=learner.id,
        assessment_id=assessment.id,
        score=grading_result["score"],
        max_score=grading_result["max_score"],
        passed=grading_result["passed"],
        feedback=grading_result.get("feedback"),
        raw_submission={"answers": [ans.dict() for ans in submission.answers]}
    )

    db.add(result)
    db.commit()
    db.refresh(result)

    logger.info(
        f"Quiz submitted: learner={submission.learner_id}, "
        f"assessment={submission.assessment_id}, score={result.score}/{result.max_score}"
    )

    return AssessmentResultResponse(
        id=result.id,
        learner_id=submission.learner_id,
        assessment_id=result.assessment_id,
        score=result.score,
        max_score=result.max_score,
        passed=result.passed,
        feedback=result.feedback,
        submitted_at=result.created_at
    )


@router.post("/lab/submit", response_model=AssessmentResultResponse, status_code=status.HTTP_201_CREATED)
def submit_lab(
    submission: LabSubmission,
    db: Session = Depends(get_db)
):
    """
    Submit lab exercise output for validation.

    Uses regex pattern matching to validate console output against expected patterns.
    """
    # Validate learner exists
    learner = db.query(LearnerProfile).filter(
        LearnerProfile.learner_id == submission.learner_id
    ).first()

    if not learner:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Learner profile '{submission.learner_id}' not found"
        )

    # Validate assessment exists
    assessment = db.query(Assessment).filter(
        Assessment.id == submission.assessment_id
    ).first()

    if not assessment:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Assessment {submission.assessment_id} not found"
        )

    if assessment.assessment_type != "LAB_EXERCISE":
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=f"Assessment {submission.assessment_id} is not a lab exercise"
        )

    # Validate lab output
    assessment_service = AssessmentService()
    validation_result = assessment_service.validate_lab_output(
        expected_patterns=assessment.content.get("expected_patterns", []),
        console_output=submission.console_output
    )

    # Create assessment result
    result = AssessmentResult(
        learner_profile_id=learner.id,
        assessment_id=assessment.id,
        score=validation_result["score"],
        max_score=validation_result["max_score"],
        passed=validation_result["passed"],
        feedback=validation_result.get("feedback"),
        raw_submission={
            "console_output": submission.console_output,
            "code_submission": submission.code_submission
        }
    )

    db.add(result)
    db.commit()
    db.refresh(result)

    logger.info(
        f"Lab submitted: learner={submission.learner_id}, "
        f"assessment={submission.assessment_id}, score={result.score}/{result.max_score}"
    )

    return AssessmentResultResponse(
        id=result.id,
        learner_id=submission.learner_id,
        assessment_id=result.assessment_id,
        score=result.score,
        max_score=result.max_score,
        passed=result.passed,
        feedback=result.feedback,
        submitted_at=result.created_at
    )


@router.get("/results/{learner_id}", response_model=List[AssessmentResultResponse])
def get_learner_results(
    learner_id: str,
    db: Session = Depends(get_db)
):
    """
    Retrieve all assessment results for a learner.
    """
    learner = db.query(LearnerProfile).filter(
        LearnerProfile.learner_id == learner_id
    ).first()

    if not learner:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Learner profile '{learner_id}' not found"
        )

    results = db.query(AssessmentResult).filter(
        AssessmentResult.learner_profile_id == learner.id
    ).order_by(AssessmentResult.created_at.desc()).all()

    return [
        AssessmentResultResponse(
            id=r.id,
            learner_id=learner_id,
            assessment_id=r.assessment_id,
            score=r.score,
            max_score=r.max_score,
            passed=r.passed,
            feedback=r.feedback,
            submitted_at=r.created_at
        )
        for r in results
    ]


@router.get("/result/{result_id}", response_model=AssessmentResultResponse)
def get_assessment_result(
    result_id: int,
    db: Session = Depends(get_db)
):
    """
    Retrieve a specific assessment result by ID.
    """
    result = db.query(AssessmentResult).filter(
        AssessmentResult.id == result_id
    ).first()

    if not result:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Assessment result {result_id} not found"
        )

    learner = db.query(LearnerProfile).filter(
        LearnerProfile.id == result.learner_profile_id
    ).first()

    return AssessmentResultResponse(
        id=result.id,
        learner_id=learner.learner_id,
        assessment_id=result.assessment_id,
        score=result.score,
        max_score=result.max_score,
        passed=result.passed,
        feedback=result.feedback,
        submitted_at=result.created_at
    )
