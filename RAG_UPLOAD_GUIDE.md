# RAG Upload Guide: Qdrant Integration

## Overview

This guide explains how to upload Chapter 1 semantic chunks to Qdrant Cloud for RAG (Retrieval-Augmented Generation) functionality. The RAG system enables intelligent search and question-answering across the textbook.

## Files Generated

All RAG files are located in `specs/1-book-curriculum/chapters/chapter-1/`:

- **`chunks.jsonl`**: 12 semantic chunks (JSONL format, one chunk per line)
  - Each chunk contains: chunk_id, chapter, section, content, keywords, difficulty, timestamp
  - File size: ~7.4 KB
  - Total chunks: 12
  - Avg chunk size: ~620 bytes

- **`embeddings.npy`**: 1536-dimensional embeddings
  - Format: NumPy binary array
  - Shape: (12, 1536)
  - Embedding model: text-embedding-3-small
  - File size: ~73 KB

- **`rag-manifest.json`**: Metadata index
  - Lists all chunks with preview and metadata
  - Created: 2025-12-31T00:00:00Z
  - Total tokens: ~3000 (estimated)

- **`upload_to_qdrant.py`**: Upload script
  - Handles Qdrant connection, collection creation, upsert
  - Validates upload and runs test search
  - Supports both local and cloud instances

## Prerequisites

### 1. Qdrant Cloud Account (Recommended for Production)

Set up a free Qdrant Cloud cluster:
1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Sign up for a free account
3. Create a cluster (choose a region close to your infrastructure)
4. Copy your **URL** and **API Key**

### 2. Local Qdrant (For Development)

If running locally via Docker:
```bash
docker run -p 6333:6333 qdrant/qdrant
```

Then set environment variables:
```bash
export QDRANT_URL="http://localhost:6333"
unset QDRANT_API_KEY  # Not needed for local instance
```

### 3. Python Dependencies

```bash
pip install qdrant-client numpy
```

## Configuration

### Option A: Environment Variables (Recommended)

```bash
# For Qdrant Cloud
export QDRANT_URL="https://your-cluster.qdrant.io"
export QDRANT_API_KEY="your-api-key"

# For local instance
export QDRANT_URL="http://localhost:6333"
```

### Option B: Update Script Directly

Edit `upload_to_qdrant.py` and modify these constants:

```python
QDRANT_URL = "https://your-cluster.qdrant.io"
QDRANT_API_KEY = "your-api-key"
```

## Running the Upload

### Step 1: Navigate to Project Directory

```bash
cd /path/to/boook
```

### Step 2: Run the Upload Script

```bash
python upload_to_qdrant.py
```

Expected output:
```
============================================================
RAG Upload to Qdrant - Chapter 1
============================================================

[Step 1] Loading chunks...
INFO:root:Loaded 12 chunks from C:/boook/specs/1-book-curriculum/chapters/chapter-1/chunks.jsonl

[Step 2] Loading embeddings...
INFO:root:Loaded embeddings from C:/boook/specs/1-book-curriculum/chapters/chapter-1/embeddings.npy: shape (12, 1536)

[Step 3] Initializing Qdrant...
INFO:root:Connected to Qdrant Cloud: https://your-cluster.qdrant.io
INFO:root:Collection 'textbook_chapters' created successfully

[Step 4] Uploading chunks to Qdrant...
INFO:root:Uploading 12 points to Qdrant in batches of 10...
INFO:root:Uploaded batch 1/2
INFO:root:Successfully uploaded 12 points to Qdrant

[Step 5] Validating upload...
INFO:root:Collection 'textbook_chapters' now contains 12 points
INFO:root:✓ Upload validation successful

[Step 6] Testing vector search...
INFO:root:Test search returned 3 results
INFO:root:  Result 1: ID=12345, Score=0.8234
INFO:root:    Section: Introduction

============================================================
✓ RAG Upload Complete - All checks passed!
============================================================
```

### Step 3: Verify in Qdrant Dashboard

1. Log into [cloud.qdrant.io](https://cloud.qdrant.io)
2. Navigate to your cluster
3. View the `textbook_chapters` collection
4. Confirm: 12 points uploaded with COSINE distance metric

## RAG-Enabled Features

Once uploaded, you can enable:

### 1. Semantic Search API

```python
from qdrant_client import QdrantClient

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# Search query embedding (from OpenAI API)
query_embedding = [... 1536 dims ...]

results = client.search(
    collection_name="textbook_chapters",
    query_vector=query_embedding,
    limit=5
)

for result in results:
    print(f"Section: {result.payload['section']}")
    print(f"Content: {result.payload['content']}")
```

### 2. Backend RAG Integration

Integrate into FastAPI backend (`backend/src/services/qdrant_service.py`):

```python
async def search_chapters(query_text: str, top_k: int = 5) -> List[Dict]:
    """Search for relevant chapter content."""

    # 1. Get embedding for query
    embedding = await llm_service.generate_embeddings([query_text])

    # 2. Search Qdrant
    results = await qdrant_service.search(
        query_embedding=embedding[0],
        limit=top_k
    )

    # 3. Return formatted results
    return results
```

### 3. Chatbot Integration

Build a RAG-augmented chatbot:

```python
async def answer_question(question: str) -> str:
    """Answer question using RAG + Claude."""

    # 1. Retrieve relevant chunks
    context = await search_chapters(question, top_k=3)
    context_text = "\n".join([c['content'] for c in context])

    # 2. Augment prompt with context
    system_prompt = f"""You are a robotics expert. Use the following chapter content to answer questions:

    {context_text}
    """

    # 3. Call Claude
    response = await llm_service.call_claude(
        system_prompt=system_prompt,
        user_message=question
    )

    return response
```

## Troubleshooting

### Connection Issues

**Error**: `Failed to connect to Qdrant`
- Check QDRANT_URL and QDRANT_API_KEY are correct
- Verify Qdrant Cloud cluster is active
- Test connection: `curl -X GET https://your-cluster.qdrant.io/health`

### Embedding Mismatch

**Error**: `Expected 1536 dimensions, got 512`
- Verify embeddings.npy shape: `python -c "import numpy as np; print(np.load('embeddings.npy').shape)"`
- Confirm embedding_model in rag-manifest.json matches your embedding service

### Upload Fails Partway

**Error**: `Uploaded batch 1/2` then stops
- The script will resume from the last successful batch (idempotent upsert)
- Rerun: `python upload_to_qdrant.py`

### Collection Already Exists

**Message**: `Collection 'textbook_chapters' already exists`
- Safe to proceed; script will upsert new/updated chunks
- To reset: use Qdrant web UI to delete collection, then rerun script

## Advanced: Custom Collection

To use a different collection name:

1. Edit `upload_to_qdrant.py`:
```python
COLLECTION_NAME = "my_custom_collection"
```

2. Rerun upload script

3. Update backend `qdrant_service.py`:
```python
self.collection_name = "my_custom_collection"
```

## Next Steps

1. **Docusaurus Integration**: Add search UI to website
2. **Backend API**: Expose `/api/v1/search` endpoint
3. **Multi-Chapter**: Generate and upload Chapters 2-16
4. **Continuous Updates**: Re-run upload script when chapters change

## Support

For issues or questions:
- Check Qdrant docs: https://qdrant.tech/documentation/
- File an issue: https://github.com/robotics-textbook/robotics-textbook/issues
- Discuss in forum: https://github.com/robotics-textbook/robotics-textbook/discussions

---

**Status**: ✓ Chapter 1 RAG-ready
**Last Updated**: 2025-12-31
**Total Chunks**: 12
**Embedding Model**: text-embedding-3-small (OpenAI)
