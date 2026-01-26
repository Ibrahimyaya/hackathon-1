"""RAG query engine for processing user queries and generating responses."""

import logging
import time
from typing import Any, Dict, List

import anthropic

from ingestion.embeddings.cohere_embedder import CohereEmbedder
from ingestion.storage.qdrant_store import QdrantStore
from rag.models import RAGResponse, RetrievedChunk
from rag.prompt_builder import build_rag_prompt, calculate_confidence
from utils.config import Config
from utils.errors import CohereAPIError, CohereQuotaError

logger = logging.getLogger(__name__)


class RAGQueryEngine:
    """Handles RAG query processing and response generation."""

    def __init__(
        self,
        config: Config,
        embedder: CohereEmbedder,
        store: QdrantStore,
        claude_client: anthropic.Anthropic,
    ):
        """Initialize the RAG query engine.

        Args:
            config: Application configuration
            embedder: Cohere embedder for query embedding
            store: Qdrant vector store
            claude_client: Anthropic Claude API client
        """
        self.config = config
        self.embedder = embedder
        self.store = store
        self.claude_client = claude_client
        self.conversation_history: List[Dict[str, str]] = []

        logger.info(
            f"Initialized RAGQueryEngine (model={config.claude_model}, "
            f"top_k={config.rag_top_k}, threshold={config.rag_score_threshold})"
        )

    def query(self, question: str, top_k: int | None = None) -> RAGResponse:
        """Process a user query and return a response.

        Args:
            question: User's question
            top_k: Number of chunks to retrieve (uses config default if None)

        Returns:
            RAGResponse with answer, sources, and confidence score

        Raises:
            CohereAPIError: If embedding fails
            CohereQuotaError: If Cohere quota exceeded
            anthropic.APIError: If Claude API call fails
        """
        if top_k is None:
            top_k = self.config.rag_top_k

        logger.info(f"Processing query: {question[:60]}...")

        try:
            # Step 1: Embed the query
            query_vector = self._embed_query(question)
            logger.debug(f"Query embedded (dimension: {len(query_vector)})")

            # Step 2: Search for relevant chunks
            chunks = self._retrieve_chunks(query_vector, top_k)
            logger.info(f"Retrieved {len(chunks)} relevant chunks")

            # Step 3: Build prompt with context and history
            prompt = build_rag_prompt(question, chunks, self.conversation_history)
            logger.debug(f"Prompt built ({len(prompt)} chars)")

            # Step 4: Call Claude API
            answer = self._generate_response(prompt)
            logger.info(f"Generated response ({len(answer)} chars)")

            # Step 5: Calculate confidence and build response
            retrieval_scores = [chunk.score for chunk in chunks]
            confidence = calculate_confidence(retrieval_scores)

            response = RAGResponse(
                answer=answer,
                sources=chunks,
                confidence=confidence,
                query=question,
                model=self.config.claude_model,
            )

            # Step 6: Add to conversation history
            self.add_to_history(question, answer)

            return response

        except CohereQuotaError:
            logger.error("Cohere quota exceeded")
            raise
        except CohereAPIError as e:
            logger.error(f"Embedding error: {str(e)}")
            raise
        except anthropic.APIError as e:
            logger.error(f"Claude API error: {str(e)}")
            raise

    def _embed_query(self, query: str) -> List[float]:
        """Embed a user query using Cohere.

        Args:
            query: Query string

        Returns:
            Embedding vector

        Raises:
            CohereAPIError: If embedding fails
            CohereQuotaError: If quota exceeded
        """
        return self.embedder.embed_query(query)

    def _retrieve_chunks(
        self, query_vector: List[float], top_k: int
    ) -> List[RetrievedChunk]:
        """Retrieve relevant chunks from vector store.

        Args:
            query_vector: Query embedding
            top_k: Number of chunks to retrieve

        Returns:
            List of RetrievedChunk objects sorted by relevance

        Raises:
            Exception: If search fails
        """
        results = self.store.search(
            query_vector=query_vector,
            limit=top_k,
            score_threshold=self.config.rag_score_threshold,
        )

        chunks = []
        for chunk_id, score, payload in results:
            chunk = RetrievedChunk(
                chunk_id=chunk_id,
                text=payload.get("text", ""),
                source_url=payload.get("source_url", ""),
                section=payload.get("section", ""),
                score=score,
            )
            chunks.append(chunk)

        return chunks

    def _generate_response(self, prompt: str) -> str:
        """Generate response using Claude API.

        Args:
            prompt: Full prompt with context and history

        Returns:
            Generated response text

        Raises:
            anthropic.APIError: If API call fails
        """
        max_retries = 3
        for attempt in range(max_retries):
            try:
                message = self.claude_client.messages.create(
                    model=self.config.claude_model,
                    max_tokens=self.config.claude_max_tokens,
                    temperature=self.config.claude_temperature,
                    messages=[{"role": "user", "content": prompt}],
                )

                # Extract text from response
                if message.content and len(message.content) > 0:
                    return message.content[0].text
                else:
                    return "No response generated"

            except anthropic.RateLimitError as e:
                if attempt == max_retries - 1:
                    raise
                wait_time = (2 ** attempt) * 60  # Exponential backoff
                logger.warning(f"Rate limited, waiting {wait_time}s before retry")
                time.sleep(wait_time)
            except anthropic.APIError as e:
                logger.error(f"Claude API error (attempt {attempt + 1}/{max_retries}): {str(e)}")
                if attempt == max_retries - 1:
                    raise
                time.sleep(2 ** attempt)

        raise anthropic.APIError("Failed to generate response after retries")

    def add_to_history(self, question: str, answer: str) -> None:
        """Add Q&A pair to conversation history.

        Maintains max history turns configured in settings.

        Args:
            question: User question
            answer: Assistant answer
        """
        self.conversation_history.append({"question": question, "answer": answer})

        # Trim history to max turns
        if len(self.conversation_history) > self.config.rag_max_history_turns:
            self.conversation_history = self.conversation_history[
                -self.config.rag_max_history_turns :
            ]

        logger.debug(f"Added to history (total turns: {len(self.conversation_history)})")

    def clear_history(self) -> None:
        """Clear conversation history."""
        self.conversation_history = []
        logger.info("Cleared conversation history")

    def get_history(self) -> List[Dict[str, str]]:
        """Get current conversation history.

        Returns:
            List of Q&A pairs
        """
        return self.conversation_history.copy()
