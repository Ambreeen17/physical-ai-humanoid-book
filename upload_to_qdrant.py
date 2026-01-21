#!/usr/bin/env python3
"""
RAG Upload Script for Qdrant Cloud
Uploads Chapter 1 semantic chunks with embeddings to Qdrant vector database.
"""

import json
import os
import sys
from typing import List, Dict, Any
import logging
from datetime import datetime
import random

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

try:
    from qdrant_client import QdrantClient
    from qdrant_client.models import Distance, VectorParams, PointStruct
except ImportError:
    logger.error("qdrant-client not installed. Install with: pip install qdrant-client")
    sys.exit(1)

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", None)
COLLECTION_NAME = "textbook_chapters"
VECTOR_SIZE = 1536  # OpenAI text-embedding-3-small dimension
BATCH_SIZE = 10

# Paths
BASE_PATH = "/root/ros_ws/specs/1-book-curriculum/chapters/chapter-1" if sys.platform != "win32" else "C:/boook/specs/1-book-curriculum/chapters/chapter-1"
CHUNKS_FILE = os.path.join(BASE_PATH, "chunks.jsonl")
MANIFEST_FILE = os.path.join(BASE_PATH, "rag-manifest.json")
EMBEDDINGS_FILE = os.path.join(BASE_PATH, "embeddings.npy")


def load_chunks(chunks_file: str) -> List[Dict[str, Any]]:
    """Load chunks from JSONL file."""
    chunks = []
    if not os.path.exists(chunks_file):
        logger.warning(f"Chunks file not found: {chunks_file}")
        return chunks

    try:
        with open(chunks_file, 'r', encoding='utf-8') as f:
            for line in f:
                if line.strip():
                    chunks.append(json.loads(line))
        logger.info(f"Loaded {len(chunks)} chunks from {chunks_file}")
        return chunks
    except Exception as e:
        logger.error(f"Failed to load chunks: {e}")
        return chunks


def load_or_generate_embeddings(embeddings_file: str, num_chunks: int) -> List[List[float]]:
    """Load embeddings from file or generate mock embeddings."""
    try:
        import numpy as np
        if os.path.exists(embeddings_file):
            embeddings = np.load(embeddings_file)
            logger.info(f"Loaded embeddings from {embeddings_file}: shape {embeddings.shape}")
            return embeddings.tolist()
    except ImportError:
        logger.warning("NumPy not installed; will generate mock embeddings")
    except Exception as e:
        logger.warning(f"Failed to load embeddings file: {e}")

    # Generate mock embeddings (for testing without real embedding model)
    logger.info(f"Generating mock embeddings for {num_chunks} chunks...")
    embeddings = []
    for i in range(num_chunks):
        # Random normalized vector
        vec = [random.gauss(0, 0.1) for _ in range(VECTOR_SIZE)]
        # Normalize
        magnitude = sum(x**2 for x in vec) ** 0.5
        if magnitude > 0:
            vec = [x / magnitude for x in vec]
        embeddings.append(vec)
    return embeddings


def initialize_qdrant() -> QdrantClient:
    """Initialize Qdrant client and ensure collection exists."""
    try:
        if QDRANT_API_KEY:
            client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
            logger.info(f"Connected to Qdrant Cloud: {QDRANT_URL}")
        else:
            client = QdrantClient(url=QDRANT_URL)
            logger.info(f"Connected to Qdrant: {QDRANT_URL}")

        # Check if collection exists
        try:
            client.get_collection(COLLECTION_NAME)
            logger.info(f"Collection '{COLLECTION_NAME}' already exists")
        except Exception:
            logger.info(f"Creating collection '{COLLECTION_NAME}'...")
            client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(size=VECTOR_SIZE, distance=Distance.COSINE),
            )
            logger.info(f"Collection '{COLLECTION_NAME}' created successfully")

        return client
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant: {e}")
        raise


def upload_chunks_to_qdrant(
    client: QdrantClient,
    chunks: List[Dict[str, Any]],
    embeddings: List[List[float]]
) -> bool:
    """Upload chunks with embeddings to Qdrant."""
    if len(chunks) != len(embeddings):
        logger.error(f"Chunk count ({len(chunks)}) != embedding count ({len(embeddings)})")
        return False

    try:
        # Prepare points for upsert
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            # Generate deterministic ID from chunk_id
            chunk_id = chunk.get("chunk_id", f"ch1_{i}")
            point_id = hash(chunk_id) % (2**31)  # Convert to positive int

            # Create point with metadata payload
            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "chunk_id": chunk_id,
                    "chapter": chunk.get("chapter", 1),
                    "chapter_title": chunk.get("chapter_title", ""),
                    "section": chunk.get("section", ""),
                    "content": chunk.get("content", "")[:500],  # Limit content size in payload
                    "content_type": chunk.get("content_type", "theory"),
                    "difficulty": chunk.get("difficulty", "beginner"),
                    "keywords": chunk.get("keywords", []),
                    "source": chunk.get("source", ""),
                    "timestamp": chunk.get("timestamp", datetime.utcnow().isoformat()),
                }
            )
            points.append(point)

        # Upload in batches
        logger.info(f"Uploading {len(points)} points to Qdrant in batches of {BATCH_SIZE}...")
        for i in range(0, len(points), BATCH_SIZE):
            batch = points[i:i+BATCH_SIZE]
            client.upsert(
                collection_name=COLLECTION_NAME,
                points=batch
            )
            logger.info(f"Uploaded batch {i//BATCH_SIZE + 1}/{(len(points)-1)//BATCH_SIZE + 1}")

        logger.info(f"Successfully uploaded {len(points)} points to Qdrant")
        return True
    except Exception as e:
        logger.error(f"Failed to upload chunks: {e}")
        return False


def validate_upload(client: QdrantClient, num_chunks: int) -> bool:
    """Validate that chunks were uploaded correctly."""
    try:
        collection_info = client.get_collection(COLLECTION_NAME)
        point_count = collection_info.points_count
        logger.info(f"Collection '{COLLECTION_NAME}' now contains {point_count} points")

        if point_count >= num_chunks:
            logger.info("✓ Upload validation successful")
            return True
        else:
            logger.warning(f"Expected ≥{num_chunks} points, got {point_count}")
            return False
    except Exception as e:
        logger.error(f"Validation failed: {e}")
        return False


def test_search(client: QdrantClient) -> bool:
    """Test vector search functionality."""
    try:
        # Create a test query vector
        test_query = [0.1] * VECTOR_SIZE
        magnitude = sum(x**2 for x in test_query) ** 0.5
        test_query = [x / magnitude for x in test_query]

        # Search
        results = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=test_query,
            limit=3,
        )

        logger.info(f"Test search returned {len(results)} results")
        for i, result in enumerate(results):
            logger.info(f"  Result {i+1}: ID={result.id}, Score={result.score:.4f}")
            if hasattr(result, 'payload') and result.payload:
                logger.info(f"    Section: {result.payload.get('section', 'N/A')}")

        return len(results) > 0
    except Exception as e:
        logger.error(f"Test search failed: {e}")
        return False


def main():
    """Main upload workflow."""
    logger.info("=" * 60)
    logger.info("RAG Upload to Qdrant - Chapter 1")
    logger.info("=" * 60)

    # Step 1: Load chunks
    logger.info("\n[Step 1] Loading chunks...")
    chunks = load_chunks(CHUNKS_FILE)
    if not chunks:
        logger.error("No chunks loaded. Exiting.")
        return False

    # Step 2: Load or generate embeddings
    logger.info(f"\n[Step 2] Loading embeddings...")
    embeddings = load_or_generate_embeddings(EMBEDDINGS_FILE, len(chunks))

    if len(embeddings) != len(chunks):
        logger.error(f"Embedding count mismatch: {len(embeddings)} vs {len(chunks)}")
        return False

    # Step 3: Initialize Qdrant
    logger.info(f"\n[Step 3] Initializing Qdrant...")
    try:
        client = initialize_qdrant()
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant: {e}")
        return False

    # Step 4: Upload chunks
    logger.info(f"\n[Step 4] Uploading chunks to Qdrant...")
    success = upload_chunks_to_qdrant(client, chunks, embeddings)

    if not success:
        logger.error("Upload failed")
        return False

    # Step 5: Validate upload
    logger.info(f"\n[Step 5] Validating upload...")
    validation_ok = validate_upload(client, len(chunks))

    # Step 6: Test search
    logger.info(f"\n[Step 6] Testing vector search...")
    search_ok = test_search(client)

    # Final summary
    logger.info("\n" + "=" * 60)
    if validation_ok and search_ok:
        logger.info("✓ RAG Upload Complete - All checks passed!")
        logger.info("=" * 60)
        logger.info(f"\nSummary:")
        logger.info(f"  - Chunks uploaded: {len(chunks)}")
        logger.info(f"  - Collection: {COLLECTION_NAME}")
        logger.info(f"  - Qdrant URL: {QDRANT_URL}")
        logger.info(f"  - Vector dimension: {VECTOR_SIZE}")
        return True
    else:
        logger.error("✗ Upload validation or search test failed")
        logger.info("=" * 60)
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
