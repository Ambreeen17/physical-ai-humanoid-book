"""LLM service - handles Claude and OpenAI API calls."""

import logging
import os
from typing import Optional, Dict, Any
import anthropic
import openai

logger = logging.getLogger(__name__)


class LLMService:
    """Service for interacting with Claude and OpenAI APIs."""

    def __init__(self):
        self.claude_client = anthropic.Anthropic(api_key=os.getenv("CLAUDE_API_KEY"))
        openai.api_key = os.getenv("OPENAI_API_KEY")

    async def call_claude(
        self,
        system_prompt: str,
        user_message: str,
        model: str = "claude-3-haiku-20240307",
        max_tokens: int = 2000,
    ) -> str:
        """Call Claude API.

        Args:
            system_prompt: System prompt for Claude
            user_message: User message
            model: Claude model to use
            max_tokens: Max tokens in response

        Returns:
            Claude's response text
        """
        try:
            logger.info(f"Calling Claude API with model {model}")
            message = self.claude_client.messages.create(
                model=model,
                max_tokens=max_tokens,
                system=system_prompt,
                messages=[{"role": "user", "content": user_message}],
            )
            return message.content[0].text
        except Exception as e:
            logger.error(f"Claude API call failed: {str(e)}")
            raise

    async def generate_embeddings(self, texts: list[str], model: str = "text-embedding-3-small") -> list[list[float]]:
        """Generate embeddings using OpenAI.

        Args:
            texts: List of texts to embed
            model: Embedding model

        Returns:
            List of embedding vectors
        """
        try:
            logger.info(f"Generating {len(texts)} embeddings")
            response = openai.Embedding.create(input=texts, model=model)
            return [item["embedding"] for item in response["data"]]
        except Exception as e:
            logger.error(f"Embedding generation failed: {str(e)}")
            raise

    async def synthesize_answer(self, retrieved_text: str, query: str) -> str:
        """Synthesize an answer from retrieved text.

        Args:
            retrieved_text: Text retrieved from RAG
            query: Original user query

        Returns:
            Synthesized answer
        """
        system_prompt = """You are an AI tutor for a robotics textbook.
Your role is to synthesize clear, accurate answers from provided textbook content.
Include citations to specific sections when relevant.
Keep answers concise but complete."""

        user_message = f"""Based on this textbook content:

{retrieved_text}

Please answer the following question:
{query}

Provide a clear, educational answer with citations to specific sections when helpful."""

        return await self.call_claude(system_prompt, user_message)


# Global service instance
llm_service = LLMService()
