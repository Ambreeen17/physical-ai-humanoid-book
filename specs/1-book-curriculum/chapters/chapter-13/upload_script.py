"""
RAG Upload Script for Chapter 13: System Integration

This script uploads the chapter content to Qdrant vector database for RAG chatbot.
"""
import json
import numpy as np
from qdrant_client import QdrantClient
from qdrant_client.http import models
import os
from typing import List, Dict, Any

# Configuration
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
COLLECTION_NAME = "textbook_chapter_13"

# Initialize Qdrant client
client = QdrantClient(url=QDRANT_URL)

def create_collection():
    """Create Qdrant collection for this chapter."""
    try:
        client.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=1536,  # OpenAI embedding dimension
                distance=models.Distance.COSINE
            )
        )
        print(f"Collection {{COLLECTION_NAME}} created successfully")
    except Exception as e:
        print(f"Collection might already exist: {{e}}")

def load_chunks() -> List[Dict[str, Any]]:
    """Load chunks from JSONL file."""
    chunks_file = "chunks.jsonl"
    chunks = []

    with open(chunks_file, "r", encoding="utf-8") as f:
        for line in f:
            if line.strip():
                chunks.append(json.loads(line))

    return chunks

def generate_embeddings(chunks: List[Dict[str, Any]]) -> List[List[float]]:
    """Generate embeddings for chunks (simulated)."""
    # In a real implementation, this would call OpenAI API
    # For simulation, we'll create random embeddings
    embeddings = []
    for i, chunk in enumerate(chunks):
        # Simulate embedding generation
        # In real implementation: response = openai.Embedding.create(input=chunk["content"], model="text-embedding-3-small")
        embedding = [0.1 * (i + 1) for _ in range(1536)]  # Placeholder values
        embeddings.append(embedding)

    return embeddings

def upload_to_qdrant(chunks: List[Dict[str, Any]], embeddings: List[List[float]]):
    """Upload chunks and embeddings to Qdrant."""
    points = []

    for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
        point = models.PointStruct(
            id=i,
            vector=embedding,
            payload={
                "id": chunk["id"],
                "content": chunk["content"],
                "type": chunk["type"],
                "chapter": 13,
                "topic": "System Integration",
                "metadata": chunk["metadata"]
            }
        )
        points.append(point)

    # Upload in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        client.upsert(
            collection_name=COLLECTION_NAME,
            points=batch
        )
        print(f"Uploaded batch {{i//batch_size + 1}} of {{(len(points) - 1)//batch_size + 1}}")

    print(f"Successfully uploaded {{len(points)}} chunks to {{COLLECTION_NAME}}")

def main():
    """Main execution function."""
    print("Starting RAG upload for Chapter 13: System Integration")

    # Create collection
    create_collection()

    # Load chunks
    print("Loading chunks...")
    chunks = load_chunks()
    print(f"Loaded {{len(chunks)}} chunks")

    # Generate embeddings
    print("Generating embeddings...")
    embeddings = generate_embeddings(chunks)
    print(f"Generated {{len(embeddings)}} embeddings")

    # Upload to Qdrant
    print("Uploading to Qdrant...")
    upload_to_qdrant(chunks, embeddings)

    print("RAG upload completed successfully!")

if __name__ == "__main__":
    main()
