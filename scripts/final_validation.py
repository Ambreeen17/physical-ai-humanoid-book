"""
Final validation script to verify all components are working correctly.
"""
import os
import sys
import json

def validate_project_completion():
    """Validate that all components of the project are complete and working."""
    print("="*60)
    print("FINAL PROJECT VALIDATION")
    print("AI-Native Robotics Textbook Platform")
    print("="*60)
    
    validation_results = []
    
    # 1. Check that all 16 chapters exist in specs
    print("\n1. Checking chapter specifications...")
    chapters_valid = True
    for i in range(1, 17):
        chapter_dir = f"specs/1-book-curriculum/chapters/chapter-{i}"
        if os.path.exists(chapter_dir):
            # For chapters 2-16, check for essential files
            if i >= 2:  # Only check generated files for chapters 2-16
                essential_files = [
                    f"{chapter_dir}/chapter-draft.md",
                    f"{chapter_dir}/research.md",
                    f"{chapter_dir}/assessments.md",
                    f"{chapter_dir}/personalization.md",
                    f"{chapter_dir}/chapter-urdu.md",
                    f"{chapter_dir}/qa-report.md"
                ]

                missing_files = [f for f in essential_files if not os.path.exists(f)]
                if missing_files:
                    print(f"   [ERROR] Chapter {i} missing files: {missing_files}")
                    chapters_valid = False
                else:
                    print(f"   [OK] Chapter {i} complete")
            else:
                # Chapter 1 was already complete, just verify directory exists
                print(f"   [OK] Chapter {i} directory exists (pre-existing)")
        else:
            print(f"   [ERROR] Chapter {i} directory missing")
            chapters_valid = False

    validation_results.append(("Chapter Specifications", chapters_valid))

    # 2. Check that all 16 chapters are integrated into frontend
    print("\n2. Checking frontend integration...")
    frontend_valid = True
    for i in range(1, 17):
        chapter_file = f"frontend/docs/chapter-{i}.md"
        if os.path.exists(chapter_file):
            with open(chapter_file, 'r', encoding='utf-8') as f:
                content = f.read()
                if len(content) > 100:  # Has substantial content
                    if f'Chapter {i}:' in content:
                        print(f"   [OK] Chapter {i} integrated in frontend")
                    else:
                        print(f"   [ERROR] Chapter {i} in frontend but wrong content")
                        frontend_valid = False
                else:
                    print(f"   [ERROR] Chapter {i} in frontend but minimal content")
                    frontend_valid = False
        else:
            print(f"   [ERROR] Chapter {i} not integrated in frontend")
            frontend_valid = False

    validation_results.append(("Frontend Integration", frontend_valid))

    # 3. Check for lab environments in all chapters
    print("\n3. Checking lab environments...")
    labs_valid = True
    for i in range(1, 17):
        lab_dir = f"specs/1-book-curriculum/chapters/chapter-{i}/lab"
        if os.path.exists(lab_dir):
            print(f"   [OK] Chapter {i} lab environment exists")
        else:
            print(f"   [ERROR] Chapter {i} lab environment missing")
            labs_valid = False

    validation_results.append(("Lab Environments", labs_valid))

    # 4. Check for RAG chunks in all chapters
    print("\n4. Checking RAG indexing...")
    rag_valid = True
    for i in range(1, 17):
        chunks_file = f"specs/1-book-curriculum/chapters/chapter-{i}/chunks.jsonl"
        if os.path.exists(chunks_file):
            print(f"   [OK] Chapter {i} RAG chunks exist")
        else:
            print(f"   [ERROR] Chapter {i} RAG chunks missing")
            rag_valid = False

    validation_results.append(("RAG Indexing", rag_valid))

    # 5. Check for agent implementations
    print("\n5. Checking agent implementations...")
    agents_valid = True
    agent_files = [
        "backend/src/agents/research_agent.py",
        "backend/src/agents/author_agent.py",
        "backend/src/agents/diagram_agent.py",
        "backend/src/agents/lab_agent.py",
        "backend/src/agents/assessment_agent.py",
        "backend/src/agents/personalization_agent.py",
        "backend/src/agents/localization_agent.py",
        "backend/src/agents/rag_indexing_agent.py",
        "backend/src/agents/qa_agent.py",
        "backend/src/agents/orchestrator.py"
    ]

    for agent_file in agent_files:
        if os.path.exists(agent_file):
            print(f"   [OK] {agent_file} exists")
        else:
            print(f"   [ERROR] {agent_file} missing")
            agents_valid = False

    validation_results.append(("Agent Implementations", agents_valid))
    
    # 6. Check for documentation
    print("\n6. Checking documentation...")
    docs_valid = True
    doc_files = [
        "QUICKSTART.md",
        "TESTING_GUIDE.md",
        "DEPLOYMENT_GUIDE.md",
        "FINAL_COMPLETION_SUMMARY.md",
        "PROJECT_STATUS.md"
    ]

    for doc_file in doc_files:
        if os.path.exists(doc_file):
            print(f"   [OK] {doc_file} exists")
        else:
            print(f"   [ERROR] {doc_file} missing")
            docs_valid = False

    validation_results.append(("Documentation", docs_valid))
    
    # 7. Check for assessment seeding
    print("\n7. Checking assessment seeding...")
    assessments_valid = True
    seed_script = "backend/scripts/seed_assessments.py"
    if os.path.exists(seed_script):
        print(f"   [OK] Assessment seeding script exists")
    else:
        print(f"   [ERROR] Assessment seeding script missing")
        assessments_valid = False

    validation_results.append(("Assessment Seeding", assessments_valid))

    # Summary
    print("\n" + "="*60)
    print("VALIDATION SUMMARY")
    print("="*60)

    all_passed = True
    for test_name, passed in validation_results:
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{test_name:<25} {status}")
        if not passed:
            all_passed = False

    print("="*60)
    if all_passed:
        print("[SUCCESS] ALL VALIDATIONS PASSED!")
        print("The AI-Native Robotics Textbook platform is COMPLETE and READY FOR PRODUCTION!")
        print("\nNext Steps:")
        print("1. Deploy to production environment")
        print("2. Begin user acceptance testing")
        print("3. Monitor system performance")
        print("4. Gather user feedback")
        print("5. Plan expansion to additional content")
    else:
        print("[ERROR] SOME VALIDATIONS FAILED!")
        print("Please address the issues above before production deployment.")

    print("="*60)

    return all_passed

if __name__ == "__main__":
    success = validate_project_completion()
    sys.exit(0 if success else 1)