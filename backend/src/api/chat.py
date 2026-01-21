"""
RAG-Powered Chatbot API
Handles conversational queries using Qdrant vector search and Claude.
"""
from typing import List, Optional
from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, Field

from ..services.llm_service import LLMService
from ..services.qdrant_service import QdrantService
from ..logging.logger import get_logger

logger = get_logger(__name__)
router = APIRouter(prefix="/api/chat", tags=["chat"])


# Request/Response Schemas
class ConversationMessage(BaseModel):
    """Single message in conversation history."""
    role: str = Field(..., description="Message role: 'user' or 'assistant'")
    content: str = Field(..., description="Message content")


class ChatRequest(BaseModel):
    """Chat query request."""
    query: str = Field(..., min_length=1, description="User query")
    learner_id: Optional[str] = Field(None, description="Learner ID for personalization")
    conversation_history: List[ConversationMessage] = Field(
        default=[],
        description="Previous conversation messages for context"
    )
    top_k: int = Field(default=5, ge=1, le=10, description="Number of relevant chunks to retrieve")


class SourceReference(BaseModel):
    """Source document reference."""
    title: str
    section: Optional[str]
    url: Optional[str]
    relevance_score: float


class ChatResponse(BaseModel):
    """Chat response with sources."""
    response: str = Field(..., description="Assistant response text")
    sources: List[SourceReference] = Field(..., description="Source documents used")
    query: str = Field(..., description="Original query (for reference)")


# Endpoints
@router.post("/", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    RAG-powered chat endpoint.

    Process:
    1. Generate query embedding using OpenAI
    2. Search Qdrant for top-k relevant chunks
    3. Construct prompt with retrieved context + conversation history
    4. Call Claude for response generation
    5. Return response with source citations
    """
    try:
        # Initialize services
        llm_service = LLMService()
        qdrant_service = QdrantService()

        # Step 1: Retrieve relevant context from Qdrant
        logger.info(f"Processing chat query: {request.query[:100]}...")

        search_results = await qdrant_service.search(
            query=request.query,
            top_k=request.top_k
        )

        if not search_results:
            logger.warning(f"No relevant documents found for query: {request.query}")
            # Return fallback response
            return ChatResponse(
                response="I couldn't find specific information in the textbook to answer your question. Could you rephrase or ask about a different topic? I'm trained on Physical AI, ROS 2, humanoid robotics, and sim-to-real transfer.",
                sources=[],
                query=request.query
            )

        # Step 2: Format retrieved chunks as context
        context_chunks = []
        sources = []

        for i, result in enumerate(search_results):
            chunk_content = result.get("content", "")
            chunk_metadata = result.get("metadata", {})

            context_chunks.append(f"[Source {i+1}] {chunk_content}")

            sources.append(SourceReference(
                title=chunk_metadata.get("chapter_title", "Unknown Chapter"),
                section=chunk_metadata.get("section", None),
                url=chunk_metadata.get("url", None),
                relevance_score=result.get("score", 0.0)
            ))

        context_text = "\n\n".join(context_chunks)

        # Step 3: Build conversation context
        conversation_context = ""
        if request.conversation_history:
            conversation_context = "\n".join([
                f"{msg.role.capitalize()}: {msg.content}"
                for msg in request.conversation_history[-5:]  # Last 5 messages
            ])

        # Step 4: Construct RAG prompt
        rag_prompt = f"""You are a helpful AI teaching assistant for a Physical AI & Humanoid Robotics textbook. Answer the user's question using the provided context from the textbook.

**Conversation History:**
{conversation_context if conversation_context else "(No previous conversation)"}

**Retrieved Context from Textbook:**
{context_text}

**User Question:**
{request.query}

**Instructions:**
- Answer directly and concisely based on the retrieved context
- If the context doesn't contain enough information, say so explicitly
- Cite specific sections when possible (e.g., "According to Section 1.2...")
- Use a friendly, educational tone
- If the user asks about labs or code, provide practical guidance
- For math/equations, use clear notation

**Answer:**"""

        # Step 5: Call Claude for response generation
        response_text = await llm_service.synthesize_answer(
            query=request.query,
            context_chunks=context_chunks,
            system_prompt=rag_prompt
        )

        logger.info(f"Generated response for query: {request.query[:50]}...")

        return ChatResponse(
            response=response_text,
            sources=sources,
            query=request.query
        )

    except Exception as e:
        logger.error(f"Chat endpoint error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process chat request: {str(e)}"
        )


@router.get("/health")
async def chat_health():
    """Health check for chat service."""
    return {
        "service": "RAG Chat",
        "status": "operational",
        "endpoints": {
            "chat": "POST /api/chat/",
            "health": "GET /api/chat/health"
        }
    }
