"""Cohere API wrapper for generating embeddings."""

import logging
import time
from typing import List

import cohere

from ingestion.models import DocumentChunk, Embedding
from utils.errors import CohereAPIError, CohereQuotaError

logger = logging.getLogger(__name__)


class CohereEmbedder:
    """Generates embeddings using Cohere API."""

    def __init__(
        self,
        api_key: str,
        model: str = "embed-english-v3.0",
        batch_size: int = 100,
        max_retries: int = 3,
    ):
        """Initialize the Cohere embedder.

        Args:
            api_key: Cohere API key
            model: Model identifier
            batch_size: Number of texts to embed per batch
            max_retries: Maximum retry attempts for API calls

        Raises:
            CohereAPIError: If API initialization fails
        """
        try:
            self.client = cohere.ClientV2(api_key=api_key)
            self.model = model
            self.batch_size = batch_size
            self.max_retries = max_retries
        except Exception as e:
            raise CohereAPIError(f"Failed to initialize Cohere client: {str(e)}") from e

    def embed(self, chunks: List[DocumentChunk]) -> List[Embedding]:
        """Generate embeddings for document chunks.

        Args:
            chunks: List of DocumentChunk objects

        Returns:
            List of Embedding objects

        Raises:
            CohereAPIError: If embedding fails
            CohereQuotaError: If quota is exceeded
        """
        if not chunks:
            return []

        logger.info(f"Generating embeddings for {len(chunks)} chunks")
        embeddings = []
        failed_chunks = []

        # Process chunks in batches
        for i in range(0, len(chunks), self.batch_size):
            batch = chunks[i : i + self.batch_size]
            texts = [chunk.text for chunk in batch]

            try:
                batch_embeddings = self._embed_batch(texts)

                # Create Embedding objects
                for chunk, vector in zip(batch, batch_embeddings):
                    embedding = Embedding(
                        chunk_id=chunk.id,
                        vector=vector,
                        model=self.model,
                    )
                    embeddings.append(embedding)

                logger.debug(f"Embedded {len(batch)} chunks ({i + len(batch)}/{len(chunks)})")

            except CohereQuotaError:
                logger.error("Cohere quota exceeded")
                failed_chunks.extend(batch)
                raise
            except CohereAPIError as e:
                logger.error(f"Failed to embed batch: {str(e)}")
                failed_chunks.extend(batch)

        if failed_chunks:
            logger.warning(f"Failed to embed {len(failed_chunks)} chunks")

        logger.info(f"Successfully generated {len(embeddings)}/{len(chunks)} embeddings")

        return embeddings

    def _embed_batch(self, texts: List[str]) -> List[List[float]]:
        """Embed a batch of texts.

        Args:
            texts: List of text strings

        Returns:
            List of embedding vectors

        Raises:
            CohereAPIError: If embedding fails
            CohereQuotaError: If quota is exceeded
        """
        for attempt in range(self.max_retries):
            try:
                response = self.client.embed(
                    model=self.model,
                    texts=texts,
                    input_type="search_document",
                )

                return response.embeddings

            except Exception as e:
                error_str = str(e).lower()

                # Check for quota/rate limit errors (retry these)
                if "quota" in error_str or "rate limit" in error_str:
                    if attempt == self.max_retries - 1:
                        raise CohereQuotaError(f"Cohere quota exceeded: {str(e)}") from e

                    # Wait before retrying
                    wait_time = (2 ** attempt) * 60  # Exponential backoff in seconds
                    logger.warning(f"Rate limited, waiting {wait_time}s before retry")
                    time.sleep(wait_time)
                    continue

                # Other errors - raise immediately (don't retry)
                logger.error(f"Cohere API error: {str(e)}")
                raise CohereAPIError(f"Failed to embed batch: {str(e)}") from e

        raise CohereAPIError("Failed to embed batch after retries")
