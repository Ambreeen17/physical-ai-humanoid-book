"""
Script to upload RAG chunks for all chapters to Qdrant.
"""
import os
import subprocess
import sys

def upload_rag_chunks():
    """Upload RAG chunks for all chapters to Qdrant."""
    print("Uploading RAG chunks to Qdrant...")
    
    # Chapters to process
    chapters = [1, 2, 3, 4]
    
    for chapter_num in chapters:
        print(f"\nUploading Chapter {chapter_num} RAG chunks...")
        
        # Change to the chapter directory
        chapter_dir = f"specs/1-book-curriculum/chapters/chapter-{chapter_num}"
        upload_script = os.path.join(chapter_dir, "upload_script.py")
        chunks_file = os.path.join(chapter_dir, "chunks.jsonl")
        
        # Check if the chunks file exists
        if not os.path.exists(chunks_file):
            print(f"Chunks file does not exist for Chapter {chapter_num}: {chunks_file}")
            continue
            
        # Check if the upload script exists
        if not os.path.exists(upload_script):
            print(f"Upload script does not exist for Chapter {chapter_num}: {upload_script}")
            continue
        
        # Run the upload script
        try:
            print(f"Running upload script for Chapter {chapter_num}...")
            result = subprocess.run([sys.executable, "upload_script.py"],
                                  cwd=chapter_dir,
                                  capture_output=True, text=True)
            
            if result.returncode == 0:
                print(f"OK Chapter {chapter_num} RAG chunks uploaded successfully")
                print(result.stdout)
            else:
                print(f"FAIL Chapter {chapter_num} upload failed")
                print("STDOUT:", result.stdout)
                print("STDERR:", result.stderr)

        except Exception as e:
            print(f"FAIL Error uploading Chapter {chapter_num}: {str(e)}")

    print("\nRAG chunk upload process completed!")

if __name__ == "__main__":
    upload_rag_chunks()