"""
Script to integrate generated chapters into the frontend.
"""
import os
import shutil

def integrate_chapters():
    """Integrate generated chapters into the frontend."""
    print("Integrating generated chapters into frontend...")

    # Define the source and destination directories
    source_dir = "specs/1-book-curriculum/chapters"
    dest_dir = "frontend/docs"

    # Process chapters 2-4 (Chapter 1 is already complete)
    for chapter_num in range(2, 5):
        source_path = os.path.join(source_dir, f"chapter-{chapter_num}", "chapter-draft.md")
        dest_path = os.path.join(dest_dir, f"chapter-{chapter_num}.md")

        if os.path.exists(source_path):
            # Read the generated chapter content
            with open(source_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract the chapter title
            chapter_title = get_chapter_title(content)

            # Add Docusaurus frontmatter
            frontmatter = f"""---
sidebar_position: {chapter_num + 1}
title: "Chapter {chapter_num}: {chapter_title}"
---

"""
            # Add the personalization toggle after the frontmatter
            updated_content = frontmatter + f'<PersonalizationToggle chapterId="{chapter_num}" />\n\n' + content

            # Write to destination
            with open(dest_path, 'w', encoding='utf-8') as f:
                f.write(updated_content)

            print(f"Updated chapter {chapter_num} in frontend")
        else:
            print(f"Source file not found: {source_path}")

    print("Chapter integration complete!")

def get_chapter_title(content):
    """Extract the chapter title from the content."""
    lines = content.split('\n')
    for line in lines:
        if line.startswith('# Chapter'):
            # Extract title after the chapter number
            parts = line.split(':', 1)
            if len(parts) > 1:
                return parts[1].strip()
    return "Untitled Chapter"

if __name__ == "__main__":
    integrate_chapters()