"""Text chunking with semantic awareness and token counting."""

import logging
import re
from typing import List

import tiktoken

from ingestion.models import DocumentChunk
from utils.errors import ChunkingStrategyError, TokenizationError

logger = logging.getLogger(__name__)


class TextChunker:
    """Chunks text into semantically meaningful pieces with token counting."""

    def __init__(
        self,
        min_tokens: int = 256,
        max_tokens: int = 512,
        overlap_tokens: int = 50,
        encoding_name: str = "cl100k_base",
    ):
        """Initialize the text chunker.

        Args:
            min_tokens: Minimum tokens per chunk
            max_tokens: Maximum tokens per chunk
            overlap_tokens: Overlap between chunks for context
            encoding_name: tiktoken encoding name
        """
        self.min_tokens = min_tokens
        self.max_tokens = max_tokens
        self.overlap_tokens = overlap_tokens

        try:
            self.encoding = tiktoken.get_encoding(encoding_name)
        except Exception as e:
            raise TokenizationError(f"Failed to load tokenizer {encoding_name}: {str(e)}") from e

    def chunk(
        self, text: str, source_url: str = "", section: str = "", metadata: dict = None
    ) -> List[DocumentChunk]:
        """Chunk text into semantically meaningful pieces.

        Args:
            text: Text to chunk
            source_url: Source URL for metadata
            section: Section hierarchy for metadata
            metadata: Additional metadata to include

        Returns:
            List of DocumentChunk objects

        Raises:
            ChunkingStrategyError: If chunking fails
        """
        if metadata is None:
            metadata = {}

        try:
            # Split at logical boundaries (sections, paragraphs)
            segments = self._split_at_boundaries(text)

            chunks = []
            chunk_index = 0
            accumulated_text = ""
            accumulated_tokens = 0

            for segment in segments:
                segment_tokens = self._count_tokens(segment)

                # Start new chunk if adding this segment would exceed max
                if accumulated_tokens + segment_tokens > self.max_tokens and accumulated_text:
                    # Save current chunk
                    chunks.append(
                        self._create_chunk(
                            accumulated_text,
                            source_url,
                            section,
                            chunk_index,
                            accumulated_tokens,
                            metadata,
                        )
                    )
                    chunk_index += 1

                    # Add overlap from end of chunk
                    accumulated_text = self._get_overlap(accumulated_text)
                    accumulated_tokens = self._count_tokens(accumulated_text)

                # Add segment to current chunk
                accumulated_text += segment
                accumulated_tokens = self._count_tokens(accumulated_text)

                # Save chunk if it's at max capacity
                if accumulated_tokens >= self.max_tokens:
                    chunks.append(
                        self._create_chunk(
                            accumulated_text,
                            source_url,
                            section,
                            chunk_index,
                            accumulated_tokens,
                            metadata,
                        )
                    )
                    chunk_index += 1
                    accumulated_text = ""
                    accumulated_tokens = 0

            # Save final chunk if it has content
            if accumulated_text.strip():
                chunks.append(
                    self._create_chunk(
                        accumulated_text,
                        source_url,
                        section,
                        chunk_index,
                        accumulated_tokens,
                        metadata,
                    )
                )

            logger.debug(f"Chunked text into {len(chunks)} chunks (avg {accumulated_tokens} tokens)")

            return chunks

        except Exception as e:
            raise ChunkingStrategyError(f"Chunking failed: {str(e)}") from e

    def _split_at_boundaries(self, text: str) -> List[str]:
        """Split text at semantic boundaries (sections, paragraphs).

        Args:
            text: Text to split

        Returns:
            List of text segments
        """
        segments = []

        # First split by double newlines (paragraphs)
        paragraphs = re.split(r"\n\n+", text)

        for para in paragraphs:
            if not para.strip():
                continue

            # Split long paragraphs by sentences
            sentences = re.split(r"(?<=[.!?])\s+", para)

            for sentence in sentences:
                if sentence.strip():
                    segments.append(sentence.strip() + " ")

        return segments

    def _count_tokens(self, text: str) -> int:
        """Count tokens in text.

        Args:
            text: Text to count

        Returns:
            Number of tokens

        Raises:
            TokenizationError: If counting fails
        """
        try:
            tokens = self.encoding.encode(text)
            return len(tokens)
        except Exception as e:
            raise TokenizationError(f"Failed to count tokens: {str(e)}") from e

    def _get_overlap(self, text: str) -> str:
        """Get overlap portion from end of text.

        Args:
            text: Text to extract overlap from

        Returns:
            Overlap text (last ~overlap_tokens worth)
        """
        if not text or self.overlap_tokens <= 0:
            return ""

        # Simple approach: take last portion
        # Estimate character count for target overlap tokens
        total_tokens = self._count_tokens(text)
        if total_tokens == 0:
            return ""

        # Calculate approximate characters per token
        char_per_token = max(1, len(text) / max(total_tokens, 1))
        char_estimate = int(self.overlap_tokens * char_per_token)

        # Get text from end
        overlap_start = max(0, len(text) - char_estimate)

        # Try to find sentence boundary
        nearest_period = text.rfind(".", overlap_start)
        if nearest_period > overlap_start:
            overlap_start = nearest_period + 1

        result = text[overlap_start:].lstrip()
        return result if result else text[-int(char_estimate):] if char_estimate < len(text) else ""

    def _create_chunk(
        self,
        text: str,
        source_url: str,
        section: str,
        chunk_index: int,
        token_count: int,
        metadata: dict,
    ) -> DocumentChunk:
        """Create a DocumentChunk object.

        Args:
            text: Chunk text
            source_url: Source URL
            section: Section hierarchy
            chunk_index: Index of this chunk
            token_count: Number of tokens
            metadata: Additional metadata

        Returns:
            DocumentChunk object
        """
        chunk_id = f"{source_url}:chunk_{chunk_index}"

        return DocumentChunk(
            id=chunk_id,
            text=text.strip(),
            source_url=source_url,
            section=section,
            chunk_index=chunk_index,
            token_count=token_count,
            metadata=metadata,
        )
