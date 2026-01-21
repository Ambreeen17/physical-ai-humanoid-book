import numpy as np
import json
import os

# Simulate creating embeddings (as I cannot actually call OpenAI API in this environment)
# In a real scenario, this would use openai.Embedding.create()
# We'll generate a dummy NumPy array with the correct dimensions (1536) for 12 chunks

OUTPUT_DIR = "C:/boook/specs/1-book-curriculum/chapters/chapter-1"
NUM_CHUNKS = 12
DIMENSIONS = 1536

# Generate reproducible "random" embeddings for testing/manifest
np.random.seed(42)
embeddings = np.random.rand(NUM_CHUNKS, DIMENSIONS).astype(np.float32)

# Save as .npy
np.save(os.path.join(OUTPUT_DIR, "embeddings.npy"), embeddings)

print(f"Mock embeddings generated: {embeddings.shape}")
print(f"Stored at: {os.path.join(OUTPUT_DIR, 'embeddings.npy')}")
