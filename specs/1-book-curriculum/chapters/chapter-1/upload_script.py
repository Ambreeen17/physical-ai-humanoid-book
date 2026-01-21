import os
import json
import numpy as np
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Configurations
COLLECTION_NAME = "chapter_1_indexing"
JSONL_PATH = "C:/boook/specs/1-book-curriculum/chapters/chapter-1/chunks.jsonl"
EMBEDDINGS_PATH = "C:/boook/specs/1-book-curriculum/chapters/chapter-1/embeddings.npy"

def upload_to_qdrant():
    # Initialize client (requires QDRANT_URL and QDRANT_API_KEY env vars)
    url = os.environ.get("QDRANT_URL", "http://localhost:6333")
    api_key = os.environ.get("QDRANT_API_KEY")

    client = QdrantClient(url=url, api_key=api_key)

    # Check if collection exists, if not create it
    if not client.collection_exists(COLLECTION_NAME):
        print(f"Creating collection: {COLLECTION_NAME}")
        client.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=1536,  # text-embedding-3-small
                distance=models.Distance.COSINE
            )
        )

    # Load data
    print("Loading chunks and embeddings...")
    chunks = []
    with open(JSONL_PATH, "r", encoding="utf-8") as f:
        for line in f:
            chunks.append(json.loads(line))

    embeddings = np.load(EMBEDDINGS_PATH)

    # Prepare points
    points = []
    for idx, (chunk, vector) in enumerate(zip(chunks, embeddings)):
        points.append(
            models.PointStruct(
                id=idx,
                payload=chunk,
                vector=vector.tolist()
            )
        )

    # Upsert
    print(f"Uploading {len(points)} points to Qdrant...")
    client.upsert(
        collection_name=COLLECTION_NAME,
        points=points
    )
    print("Upload complete!")

if __name__ == "__main__":
    upload_to_qdrant()
