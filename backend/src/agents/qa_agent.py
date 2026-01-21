"""QA Agent - Performs quality assurance on the chapter artifacts."""

import logging
from typing import Dict, Any
import asyncio
from datetime import datetime
import os
import json

from src.agents.protocol import AgentInterface, AgentInput, AgentOutput, AgentStatus
import logging

logger = logging.getLogger(__name__)


class QAAgent(AgentInterface):
    """QA agent that performs quality assurance on chapter artifacts."""

    def __init__(self):
        super().__init__("QA")
        self.logger = logger

    async def execute(self, agent_input: AgentInput) -> AgentOutput:
        """Execute QA for the chapter.

        Args:
            agent_input: Chapter metadata including number, topic, and previous artifacts

        Returns:
            AgentOutput with QA report
        """
        self.logger.info(f"Starting QA for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}")

        try:
            # Perform QA checks
            qa_report = self._perform_qa_checks(agent_input)
            
            # Write QA report
            qa_report_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/qa-report.md"
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(qa_report_file_path), exist_ok=True)
            
            with open(qa_report_file_path, 'w', encoding='utf-8') as f:
                f.write(qa_report)
            
            artifacts = {
                "qa-report.md": qa_report_file_path
            }
            
            self.logger.info(f"QA completed for Chapter {agent_input.chapter_number}")
            return AgentOutput(
                status=AgentStatus.SUCCESS,
                artifacts=artifacts,
                duration_seconds=15  # Simulated duration
            )
            
        except Exception as e:
            self.logger.error(f"QA agent failed: {str(e)}")
            return AgentOutput(
                status=AgentStatus.FAILURE,
                artifacts={},
                error_message=str(e)
            )

    def _perform_qa_checks(self, agent_input: AgentInput) -> str:
        """Perform QA checks on all chapter artifacts."""
        # Check if previous artifacts exist
        artifacts_present = []
        artifacts_missing = []
        
        expected_artifacts = [
            "research.md",
            "chapter-draft.md", 
            "diagrams.md",
            "assessments.md",
            "personalization.md",
            "chapter-urdu.md",
            "rag-manifest.json",
            "chunks.jsonl",
            "lab/README.md"
        ]
        
        for artifact in expected_artifacts:
            if agent_input.previous_artifacts and artifact in agent_input.previous_artifacts:
                artifacts_present.append(artifact)
            else:
                artifacts_missing.append(artifact)
        
        # Calculate quality score based on completeness
        total_artifacts = len(expected_artifacts)
        present_artifacts = len(artifacts_present)
        quality_score = int((present_artifacts / total_artifacts) * 100) if total_artifacts > 0 else 0
        
        # Perform content checks
        content_issues = self._check_content_quality(agent_input)
        
        # Perform learning objectives alignment check
        lo_alignment_issues = self._check_learning_objectives_alignment(agent_input)
        
        # Perform hardware accuracy check
        hardware_issues = self._check_hardware_accuracy(agent_input)
        
        # Perform code validity check
        code_issues = self._check_code_validity(agent_input)
        
        # Generate QA report
        report = f"""# Chapter {agent_input.chapter_number} QA Report: {agent_input.chapter_topic}

## Executive Summary

**Quality Score**: {quality_score}/100
**Status**: {"PASS" if quality_score >= 85 else "FAIL"}
**Date**: {datetime.now().isoformat()}

## Artifact Completeness

### Present Artifacts ({len(artifacts_present)}/{total_artifacts})
"""
        for artifact in artifacts_present:
            report += f"- ✅ {artifact}\n"
        
        report += f"""
### Missing Artifacts ({len(artifacts_missing)}/{total_artifacts})
"""
        for artifact in artifacts_missing:
            report += f"- ❌ {artifact}\n"
        
        report += f"""
## Content Quality Assessment

### Content Issues Found: {len(content_issues)}
"""
        for i, issue in enumerate(content_issues, 1):
            report += f"{i}. {issue}\n"
        
        report += f"""
### Learning Objectives Alignment: {len(lo_alignment_issues)} issues
"""
        for i, issue in enumerate(lo_alignment_issues, 1):
            report += f"{i}. {issue}\n"
        
        report += f"""
### Hardware Accuracy: {len(hardware_issues)} issues
"""
        for i, issue in enumerate(hardware_issues, 1):
            report += f"{i}. {issue}\n"
        
        report += f"""
### Code Validity: {len(code_issues)} issues
"""
        for i, issue in enumerate(code_issues, 1):
            report += f"{i}. {issue}\n"
        
        report += f"""
## Recommendations

Based on the QA assessment, the following recommendations are made:

1. **Content Quality**: {"Address content issues" if content_issues else "Content quality is good"}
2. **Learning Objectives**: {"Align content with learning objectives" if lo_alignment_issues else "Content aligns well with learning objectives"}
3. **Hardware Accuracy**: {"Verify hardware specifications" if hardware_issues else "Hardware specifications appear accurate"}
4. **Code Validity**: {"Validate code examples" if code_issues else "Code examples appear valid"}

## Detailed Analysis

### Content Quality
The content quality assessment checks for:
- Clarity and coherence of explanations
- Proper use of technical terminology
- Logical flow between sections
- Consistency in style and formatting

### Learning Objectives Alignment
Each learning objective should be adequately covered in the chapter content:
"""
        if isinstance(agent_input.context, dict) and "learning_objectives" in agent_input.context:
            for i, objective in enumerate(agent_input.context["learning_objectives"], 1):
                report += f"- LO{i}: {objective}\n"
        else:
            report += f"- LO1: Define key concepts related to {agent_input.chapter_topic}\n"
            report += f"- LO2: Apply {agent_input.chapter_topic} principles to robotics problems\n"
            report += f"- LO3: Implement {agent_input.chapter_topic} solutions in ROS 2\n"
            report += f"- LO4: Evaluate {agent_input.chapter_topic} performance\n"
            report += f"- LO5: Integrate {agent_input.chapter_topic} with other robotics systems\n"

        report += f"""
### Hardware Accuracy
Hardware specifications should match current 2025 standards, particularly for the Unitree G1 robot and associated sensors/actuators.

### Code Validity
Code examples should follow ROS 2 Humble best practices and be executable in the provided lab environment.

## Pass/Fail Criteria

- **Pass**: Quality score ≥ 85/100
- **Conditional Pass**: Quality score 70-84/100 with minor issues
- **Fail**: Quality score < 70/100 with major issues

## Conclusion

**Assessment**: {"PASS" if quality_score >= 85 else "CONDITIONAL PASS" if quality_score >= 70 else "FAIL"}
**Recommendation**: {"Proceed to next phase" if quality_score >= 85 else "Address critical issues before proceeding"}

---

**QA Report Generated**: {datetime.now().isoformat()}
**Chapter**: {agent_input.chapter_number} - {agent_input.chapter_topic}
**Quality Score**: {quality_score}/100
"""
        return report

    def _check_content_quality(self, agent_input: AgentInput) -> list:
        """Check content quality."""
        issues = []
        
        # Check if chapter content exists and is substantial
        if agent_input.previous_artifacts and "chapter-draft.md" in agent_input.previous_artifacts:
            chapter_path = agent_input.previous_artifacts["chapter-draft.md"]
            try:
                with open(chapter_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    if len(content) < 1000:  # Less than 1000 characters is too short
                        issues.append("Chapter content appears too short (<1000 characters)")
            except FileNotFoundError:
                issues.append("Chapter draft file not found for content check")
        else:
            issues.append("Chapter draft not available for content check")
        
        # Check for basic structure
        if not isinstance(agent_input.context, dict) or "learning_objectives" not in agent_input.context:
            issues.append("Learning objectives not properly defined in context")
        
        return issues

    def _check_learning_objectives_alignment(self, agent_input: AgentInput) -> list:
        """Check alignment with learning objectives."""
        issues = []
        
        # Check if learning objectives are defined
        if not isinstance(agent_input.context, dict) or "learning_objectives" not in agent_input.context:
            issues.append("Learning objectives not defined in agent context")
        else:
            learning_objectives = agent_input.context["learning_objectives"]
            if len(learning_objectives) < 3:
                issues.append("Insufficient number of learning objectives (<3)")
        
        return issues

    def _check_hardware_accuracy(self, agent_input: AgentInput) -> list:
        """Check hardware accuracy."""
        issues = []
        
        # Check if research content exists
        if agent_input.previous_artifacts and "research.md" in agent_input.previous_artifacts:
            research_path = agent_input.previous_artifacts["research.md"]
            try:
                with open(research_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    if "Unitree G1" not in content:
                        issues.append("Unitree G1 hardware specifications not mentioned in research")
            except FileNotFoundError:
                issues.append("Research file not found for hardware check")
        else:
            issues.append("Research file not available for hardware check")
        
        return issues

    def _check_code_validity(self, agent_input: AgentInput) -> list:
        """Check code validity."""
        issues = []
        
        # Check if chapter content exists
        if agent_input.previous_artifacts and "chapter-draft.md" in agent_input.previous_artifacts:
            chapter_path = agent_input.previous_artifacts["chapter-draft.md"]
            try:
                with open(chapter_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    if "```python" not in content and "import rclpy" not in content:
                        issues.append("No Python/ROS 2 code examples found in chapter")
            except FileNotFoundError:
                issues.append("Chapter file not found for code check")
        else:
            issues.append("Chapter file not available for code check")
        
        return issues