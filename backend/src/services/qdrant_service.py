"""Qdrant service - handles vector database operations."""

import logging
from typing import List, Dict, Any, Optional
import os

logger = logging.getLogger(__name__)

# Try to import qdrant_client, but make it optional
try:
    from qdrant_client import QdrantClient
    from qdrant_client.models import Distance, VectorParams, PointStruct
    QDRANT_AVAILABLE = True
except ImportError:
    QDRANT_AVAILABLE = False
    QdrantClient = None


class QdrantService:
    """Service for RAG vector database operations."""

    def __init__(self, qdrant_url: str = None, api_key: str = None):
        self.qdrant_url = qdrant_url or os.getenv("QDRANT_URL", "http://localhost:6333")
        self.api_key = api_key or os.getenv("QDRANT_API_KEY")
        self.collection_name = "textbook_chapters"
        self.client = None
        self._available = False

        if not QDRANT_AVAILABLE:
            logger.warning("Qdrant client not installed. Vector search disabled.")
            return

        try:
            self.client = QdrantClient(url=self.qdrant_url, api_key=self.api_key)
            self._ensure_collection()
            self._available = True
            logger.info("Connected to Qdrant successfully")
        except Exception as e:
            logger.warning(f"Could not connect to Qdrant: {e}. Vector search disabled.")

    def _ensure_collection(self):
        """Ensure collection exists, create if not."""
        try:
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection '{self.collection_name}' exists")
        except Exception:
            logger.info(f"Creating collection '{self.collection_name}'")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
            )

    async def upsert_chunks(
        self,
        chapter_id: str,
        chunks: List[Dict[str, Any]],
        embeddings: List[List[float]],
    ) -> bool:
        """Upsert chapter chunks with embeddings.

        Args:
            chapter_id: Chapter ID
            chunks: List of chunk data (content, section, etc.)
            embeddings: List of embedding vectors (same length as chunks)

        Returns:
            True if successful
        """
        if not self._available:
            logger.warning("Qdrant not available, skipping upsert")
            return False
        try:
            points = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                point_id = hash(f"{chapter_id}_{i}") % (2**31)
                points.append(
                    PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={
                            "chapter_id": chapter_id,
                            "section": chunk.get("section"),
                            "content": chunk.get("content"),
                            "position": i,
                        },
                    )
                )

            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )
            logger.info(f"Upserted {len(points)} chunks for chapter {chapter_id}")
            return True
        except Exception as e:
            logger.error(f"Failed to upsert chunks: {str(e)}")
            return False

    async def search(
        self,
        query_embedding: List[float],
        limit: int = 5,
        chapter_filter: Optional[str] = None,
    ) -> List[Dict[str, Any]]:
        """Search for similar chunks.

        Args:
            query_embedding: Query embedding vector
            limit: Number of results to return
            chapter_filter: Optional chapter ID to filter by

        Returns:
            List of similar chunks with scores
        """
        if not self._available:
            logger.warning("Qdrant not available, returning empty results")
            return []
        try:
            query_filter = None
            if chapter_filter:
                query_filter = {"must": [{"key": "chapter_id", "match": {"value": chapter_filter}}]}

            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=query_filter,
                limit=limit,
            )

            return [
                {
                    "score": result.score,
                    "chapter_id": result.payload["chapter_id"],
                    "section": result.payload.get("section"),
                    "content": result.payload.get("content"),
                    "position": result.payload.get("position"),
                }
                for result in results
            ]
        except Exception as e:
            logger.error(f"Search failed: {str(e)}")
            return []


# Global service instance
qdrant_service = QdrantService()
