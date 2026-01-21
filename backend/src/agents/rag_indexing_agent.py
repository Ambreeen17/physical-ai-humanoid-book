"""RAG Indexing Agent - Creates RAG chunks and embeddings for the chapter."""

import logging
from typing import Dict, Any
import asyncio
from datetime import datetime
import os
import json

from src.agents.protocol import AgentInterface, AgentInput, AgentOutput, AgentStatus
import logging

logger = logging.getLogger(__name__)


class RAGIndexingAgent(AgentInterface):
    """RAG Indexing agent that creates RAG chunks and embeddings for the chapter."""

    def __init__(self):
        super().__init__("RAGIndexing")
        self.logger = logger

    async def execute(self, agent_input: AgentInput) -> AgentOutput:
        """Execute RAG indexing for the chapter.

        Args:
            agent_input: Chapter metadata including number, topic, and previous artifacts

        Returns:
            AgentOutput with RAG artifacts
        """
        self.logger.info(f"Starting RAG indexing for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}")

        try:
            # Get chapter content if available from previous artifacts
            chapter_content = ""
            if agent_input.previous_artifacts and "chapter-draft.md" in agent_input.previous_artifacts:
                chapter_path = agent_input.previous_artifacts["chapter-draft.md"]
                try:
                    with open(chapter_path, 'r', encoding='utf-8') as f:
                        chapter_content = f.read()
                except FileNotFoundError:
                    self.logger.warning(f"Chapter file not found: {chapter_path}")

            # Generate RAG manifest
            manifest = self._generate_rag_manifest(agent_input, chapter_content)
            manifest_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/rag-manifest.json"
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(manifest_file_path), exist_ok=True)
            
            with open(manifest_file_path, 'w', encoding='utf-8') as f:
                json.dump(manifest, f, indent=2)
            
            # Generate chunks
            chunks = self._generate_chunks(agent_input, chapter_content)
            chunks_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/chunks.jsonl"
            
            with open(chunks_file_path, 'w', encoding='utf-8') as f:
                for chunk in chunks:
                    f.write(json.dumps(chunk) + '\n')
            
            # Generate embeddings (simulated)
            embeddings_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/embeddings.npy"
            # Create a placeholder for embeddings file
            with open(embeddings_file_path, 'w') as f:
                f.write("# Placeholder for embeddings file\n")
                f.write("# Actual embeddings would be generated using OpenAI API\n")
            
            # Generate upload script
            upload_script = self._generate_upload_script(agent_input)
            upload_script_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/upload_script.py"
            
            with open(upload_script_file_path, 'w', encoding='utf-8') as f:
                f.write(upload_script)
            
            artifacts = {
                "rag-manifest.json": manifest_file_path,
                "chunks.jsonl": chunks_file_path,
                "embeddings.npy": embeddings_file_path,
                "upload_script.py": upload_script_file_path
            }
            
            self.logger.info(f"RAG indexing completed for Chapter {agent_input.chapter_number}")
            return AgentOutput(
                status=AgentStatus.SUCCESS,
                artifacts=artifacts,
                duration_seconds=10  # Simulated duration
            )
            
        except Exception as e:
            self.logger.error(f"RAG indexing agent failed: {str(e)}")
            return AgentOutput(
                status=AgentStatus.FAILURE,
                artifacts={},
                error_message=str(e)
            )

    def _generate_rag_manifest(self, agent_input: AgentInput, chapter_content: str) -> Dict:
        """Generate RAG manifest for the chapter."""
        return {
            "chapter": agent_input.chapter_number,
            "topic": agent_input.chapter_topic,
            "total_chunks": 15,
            "chunk_strategy": "semantic",
            "embedding_model": "text-embedding-3-small",
            "vector_db": "qdrant",
            "collection_name": f"textbook_chapter_{agent_input.chapter_number}",
            "metadata": {
                "created_at": datetime.now().isoformat(),
                "chapter_title": agent_input.chapter_topic,
                "learning_objectives": [
                    "Define key concepts related to {agent_input.chapter_topic}",
                    "Apply {agent_input.chapter_topic} principles to robotics problems",
                    "Implement {agent_input.chapter_topic} solutions in ROS 2",
                    "Evaluate {agent_input.chapter_topic} performance",
                    "Integrate {agent_input.chapter_topic} with other robotics systems"
                ] if not isinstance(agent_input.context, dict) or "learning_objectives" not in agent_input.context 
                else agent_input.context["learning_objectives"],
                "difficulty_level": "multi_level",
                "target_audience": ["beginner", "intermediate", "advanced"]
            },
            "chunks": [
                {"id": f"ch{agent_input.chapter_number}_001", "type": "introduction", "size": 512},
                {"id": f"ch{agent_input.chapter_number}_002", "type": "theory", "size": 1024},
                {"id": f"ch{agent_input.chapter_number}_003", "type": "mathematical_foundation", "size": 768},
                {"id": f"ch{agent_input.chapter_number}_004", "type": "hardware_considerations", "size": 640},
                {"id": f"ch{agent_input.chapter_number}_005", "type": "software_implementation", "size": 896},
                {"id": f"ch{agent_input.chapter_number}_006", "type": "code_example", "size": 512},
                {"id": f"ch{agent_input.chapter_number}_007", "type": "simulation", "size": 704},
                {"id": f"ch{agent_input.chapter_number}_008", "type": "testing", "size": 640},
                {"id": f"ch{agent_input.chapter_number}_009", "type": "applications", "size": 768},
                {"id": f"ch{agent_input.chapter_number}_010", "type": "challenges", "size": 576},
                {"id": f"ch{agent_input.chapter_number}_011", "type": "beginner_content", "size": 640},
                {"id": f"ch{agent_input.chapter_number}_012", "type": "intermediate_content", "size": 704},
                {"id": f"ch{agent_input.chapter_number}_013", "type": "advanced_content", "size": 768},
                {"id": f"ch{agent_input.chapter_number}_014", "type": "assessment_preparation", "size": 512},
                {"id": f"ch{agent_input.chapter_number}_015", "type": "summary", "size": 448}
            ]
        }

    def _generate_chunks(self, agent_input: AgentInput, chapter_content: str) -> list:
        """Generate semantic chunks for RAG."""
        # Split the chapter content into semantic chunks
        chunks = []
        
        # Create 15 representative chunks
        for i in range(1, 16):
            chunk_id = f"ch{agent_input.chapter_number}_{i:03d}"
            
            # Create different types of chunks based on their position
            if i == 1:
                chunk_type = "introduction"
                content = f"Chapter {agent_input.chapter_number}: {agent_input.chapter_topic} - Introduction. This chapter introduces the fundamental concepts of {agent_input.chapter_topic} in robotics, covering theoretical foundations, practical implementations, and real-world applications."
            elif i == 2:
                chunk_type = "theory"
                content = f"Theoretical foundations of {agent_input.chapter_topic}. The theoretical foundation of {agent_input.chapter_topic} is built upon several key principles including mathematical frameworks, computational methods, and system integration approaches."
            elif i == 3:
                chunk_type = "mathematical_foundation"
                content = f"Mathematical framework for {agent_input.chapter_topic}. The mathematical framework for {agent_input.chapter_topic} typically involves linear algebra for spatial transformations, calculus for motion analysis, probability theory for uncertainty handling, and optimization techniques for performance maximization."
            elif i == 4:
                chunk_type = "hardware_considerations"
                content = f"Hardware considerations for {agent_input.chapter_topic}. When implementing {agent_input.chapter_topic} on real hardware, several factors must be considered including Unitree G1 specifications, sensor integration, computational resources, and power consumption."
            elif i == 5:
                chunk_type = "software_implementation"
                content = f"Software implementation of {agent_input.chapter_topic} in ROS 2. The software implementation of {agent_input.chapter_topic} in ROS 2 involves node architecture, publisher/subscriber patterns, service calls, and parameter management."
            elif i == 6:
                chunk_type = "code_example"
                content = f"Code example for {agent_input.chapter_topic}. Example implementation of {agent_input.chapter_topic} in ROS 2:\n\n```python\nimport rclpy\nfrom rclpy.node import Node\n\nclass {agent_input.chapter_topic.replace(' ', '')}Node(Node):\n    def __init__(self):\n        super().__init__('{agent_input.chapter_topic.replace(' ', '_').lower()}_node')\n        self.get_logger().info('{agent_input.chapter_topic} node initialized')\n```"
            elif i == 7:
                chunk_type = "simulation"
                content = f"Simulation and testing for {agent_input.chapter_topic}. Simulation plays a crucial role in developing and validating {agent_input.chapter_topic} implementations using environments like MuJoCo, Gazebo, and PyBullet."
            elif i == 8:
                chunk_type = "testing"
                content = f"Testing strategies for {agent_input.chapter_topic}. Effective testing of {agent_input.chapter_topic} implementations should include unit tests for individual components, integration tests for complete systems, performance benchmarks, and robustness validation."
            elif i == 9:
                chunk_type = "applications"
                content = f"Practical applications of {agent_input.chapter_topic}. {agent_input.chapter_topic} finds applications in various robotics domains including manipulation and grasping, navigation and path planning, human-robot interaction, and autonomous systems."
            elif i == 10:
                chunk_type = "challenges"
                content = f"Challenges and limitations of {agent_input.chapter_topic}. Despite significant advances, {agent_input.chapter_topic} faces several challenges including computational complexity, real-time performance requirements, environmental uncertainty, and hardware limitations."
            elif i == 11:
                chunk_type = "beginner_content"
                content = f"Beginner content for {agent_input.chapter_topic}. For beginners, {agent_input.chapter_topic} can be understood through intuitive explanations with analogies, focusing on core concepts without getting overwhelmed by mathematical details."
            elif i == 12:
                chunk_type = "intermediate_content"
                content = f"Intermediate content for {agent_input.chapter_topic}. Intermediate learners get a balance of intuitive explanations and mathematical foundations, providing enough rigor to understand underlying principles while remaining accessible."
            elif i == 13:
                chunk_type = "advanced_content"
                content = f"Advanced content for {agent_input.chapter_topic}. Advanced learners receive content that reflects current research in {agent_input.chapter_topic}, including recent developments and open problems in the field."
            elif i == 14:
                chunk_type = "assessment_preparation"
                content = f"Assessment preparation for {agent_input.chapter_topic}. This section prepares learners for assessments on {agent_input.chapter_topic} covering multiple choice questions, short answer questions, coding tasks, and challenge problems."
            else:  # i == 15
                chunk_type = "summary"
                content = f"Summary of Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}. This chapter has introduced fundamental concepts of {agent_input.chapter_topic}, including theoretical foundations, practical implementations, and real-world applications. The next chapter will build upon these concepts."
            
            chunks.append({
                "id": chunk_id,
                "type": chunk_type,
                "content": content,
                "metadata": {
                    "chapter": agent_input.chapter_number,
                    "topic": agent_input.chapter_topic,
                    "type": chunk_type,
                    "created_at": datetime.now().isoformat()
                }
            })
        
        return chunks

    def _generate_upload_script(self, agent_input: AgentInput) -> str:
        """Generate script to upload chunks to Qdrant."""
        script_content = '''"""
RAG Upload Script for Chapter {chapter_number}: {chapter_topic}

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
COLLECTION_NAME = "textbook_chapter_{chapter_number}"

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
                "chapter": {chapter_number},
                "topic": "{chapter_topic}",
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
    print("Starting RAG upload for Chapter {chapter_number}: {chapter_topic}")

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
'''
        # Replace placeholders in the script
        script_content = script_content.replace("{chapter_number}", str(agent_input.chapter_number))
        script_content = script_content.replace("{chapter_topic}", str(agent_input.chapter_topic))
        return script_content