"""
Script to validate the generated chapters and integration.
"""
import os
import json

def validate_generated_chapters():
    """Validate that chapters were generated and integrated properly."""
    print("Validating generated chapters...")
    
    # Check that the source chapters exist and have content
    for chapter_num in range(2, 5):
        source_path = f"specs/1-book-curriculum/chapters/chapter-{chapter_num}/chapter-draft.md"
        if os.path.exists(source_path):
            with open(source_path, 'r', encoding='utf-8') as f:
                content = f.read()
                if len(content) > 100:  # Check that it has substantial content
                    print(f"OK Chapter {chapter_num} source exists and has content ({len(content)} chars)")
                else:
                    print(f"MISS Chapter {chapter_num} source exists but has minimal content")
        else:
            print(f"MISS Chapter {chapter_num} source does not exist")

    # Check that the frontend chapters exist and have content
    for chapter_num in range(2, 5):
        frontend_path = f"frontend/docs/chapter-{chapter_num}.md"
        if os.path.exists(frontend_path):
            with open(frontend_path, 'r', encoding='utf-8') as f:
                content = f.read()
                if len(content) > 100:  # Check that it has substantial content
                    if '<PersonalizationToggle' in content:
                        print(f"OK Chapter {chapter_num} frontend integrated with personalization toggle ({len(content)} chars)")
                    else:
                        print(f"MISS Chapter {chapter_num} frontend integrated but missing personalization toggle")
                else:
                    print(f"MISS Chapter {chapter_num} frontend exists but has minimal content")
        else:
            print(f"MISS Chapter {chapter_num} frontend does not exist")
    
    # Check that other artifacts were generated
    expected_artifacts = [
        "specs/1-book-curriculum/chapters/chapter-2/research.md",
        "specs/1-book-curriculum/chapters/chapter-2/diagrams.md",
        "specs/1-book-curriculum/chapters/chapter-2/assessments.md",
        "specs/1-book-curriculum/chapters/chapter-2/personalization.md",
        "specs/1-book-curriculum/chapters/chapter-2/chapter-urdu.md",
        "specs/1-book-curriculum/chapters/chapter-2/qa-report.md",
        "specs/1-book-curriculum/chapters/chapter-2/lab/README.md",
        "specs/1-book-curriculum/chapters/chapter-3/research.md",
        "specs/1-book-curriculum/chapters/chapter-3/diagrams.md",
        "specs/1-book-curriculum/chapters/chapter-3/assessments.md",
        "specs/1-book-curriculum/chapters/chapter-3/personalization.md",
        "specs/1-book-curriculum/chapters/chapter-3/chapter-urdu.md",
        "specs/1-book-curriculum/chapters/chapter-3/qa-report.md",
        "specs/1-book-curriculum/chapters/chapter-3/lab/README.md",
        "specs/1-book-curriculum/chapters/chapter-4/research.md",
        "specs/1-book-curriculum/chapters/chapter-4/diagrams.md",
        "specs/1-book-curriculum/chapters/chapter-4/assessments.md",
        "specs/1-book-curriculum/chapters/chapter-4/personalization.md",
        "specs/1-book-curriculum/chapters/chapter-4/chapter-urdu.md",
        "specs/1-book-curriculum/chapters/chapter-4/qa-report.md",
        "specs/1-book-curriculum/chapters/chapter-4/lab/README.md",
    ]
    
    print("\nValidating additional artifacts...")
    artifacts_found = 0
    for artifact in expected_artifacts:
        if os.path.exists(artifact):
            artifacts_found += 1
            print(f"OK {artifact}")
        else:
            print(f"MISS {artifact}")

    print(f"\nSummary: {artifacts_found}/{len(expected_artifacts)} artifacts found")

    # Check QA reports for quality
    print("\nChecking QA reports...")
    for chapter_num in range(2, 5):
        qa_report_path = f"specs/1-book-curriculum/chapters/chapter-{chapter_num}/qa-report.md"
        if os.path.exists(qa_report_path):
            with open(qa_report_path, 'r', encoding='utf-8') as f:
                content = f.read()
                if "Quality Score" in content:
                    # Extract quality score
                    for line in content.split('\n'):
                        if "Quality Score" in line:
                            print(f"OK Chapter {chapter_num} QA report: {line.strip()}")
                            break
                else:
                    print(f"MISS Chapter {chapter_num} QA report exists but format unclear")
        else:
            print(f"MISS Chapter {chapter_num} QA report does not exist")

    print("\nValidation complete!")

if __name__ == "__main__":
    validate_generated_chapters()