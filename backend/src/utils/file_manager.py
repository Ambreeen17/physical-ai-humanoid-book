"""File management utilities for artifact handling."""

import logging
import os
from pathlib import Path
from typing import Dict, Optional

logger = logging.getLogger(__name__)


class FileManager:
    """Manages artifact files for chapters."""

    def __init__(self, base_dir: str = "./specs/1-book-curriculum/chapters"):
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)

    def get_chapter_dir(self, chapter_number: int) -> Path:
        """Get chapter directory path.

        Args:
            chapter_number: Chapter number

        Returns:
            Path to chapter directory
        """
        chapter_dir = self.base_dir / f"chapter-{chapter_number}"
        chapter_dir.mkdir(parents=True, exist_ok=True)
        return chapter_dir

    def save_artifact(self, chapter_number: int, filename: str, content: str) -> Path:
        """Save artifact file.

        Args:
            chapter_number: Chapter number
            filename: Filename (e.g., "research.md", "chapter-draft.md")
            content: File content

        Returns:
            Path to saved file
        """
        try:
            chapter_dir = self.get_chapter_dir(chapter_number)
            file_path = chapter_dir / filename

            with open(file_path, "w", encoding="utf-8") as f:
                f.write(content)

            logger.info(f"Saved artifact: {file_path}")
            return file_path
        except Exception as e:
            logger.error(f"Failed to save artifact: {str(e)}")
            raise

    def read_artifact(self, chapter_number: int, filename: str) -> Optional[str]:
        """Read artifact file.

        Args:
            chapter_number: Chapter number
            filename: Filename

        Returns:
            File content or None
        """
        try:
            file_path = self.get_chapter_dir(chapter_number) / filename
            if file_path.exists():
                with open(file_path, "r", encoding="utf-8") as f:
                    return f.read()
            else:
                logger.warning(f"Artifact not found: {file_path}")
                return None
        except Exception as e:
            logger.error(f"Failed to read artifact: {str(e)}")
            return None

    def list_artifacts(self, chapter_number: int) -> Dict[str, Path]:
        """List all artifacts in chapter directory.

        Args:
            chapter_number: Chapter number

        Returns:
            Dictionary of filename to path
        """
        chapter_dir = self.get_chapter_dir(chapter_number)
        return {f.name: f for f in chapter_dir.iterdir() if f.is_file()}


# Global instance
file_manager = FileManager()
