"""Data models for the RAG ingestion pipeline."""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class DocumentChunk:
    """Represents a single chunk of extracted and processed document text."""

    id: str
    """Unique identifier for the chunk (e.g., 'url:chunk_index')"""

    text: str
    """The cleaned text content of the chunk"""

    source_url: str
    """URL of the source document"""

    section: str = ""
    """Section or heading hierarchy from the source"""

    chunk_index: int = 0
    """Index of this chunk within the document (0-based)"""

    token_count: int = 0
    """Number of tokens in this chunk (for reference and debugging)"""

    metadata: dict = field(default_factory=dict)
    """Additional metadata (author, date, tags, etc.)"""

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            "id": self.id,
            "text": self.text,
            "source_url": self.source_url,
            "section": self.section,
            "chunk_index": self.chunk_index,
            "token_count": self.token_count,
            "metadata": self.metadata,
        }


@dataclass
class Embedding:
    """Represents a vector embedding for a document chunk."""

    chunk_id: str
    """ID of the chunk this embedding represents"""

    vector: List[float]
    """The embedding vector (e.g., 1024-dimensional for Cohere)"""

    model: str = "embed-english-v3.0"
    """Model used to generate this embedding"""

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            "chunk_id": self.chunk_id,
            "vector": self.vector,
            "model": self.model,
        }


@dataclass
class PipelineStats:
    """Statistics collected during pipeline execution."""

    pages_crawled: int = 0
    pages_failed: int = 0
    chunks_created: int = 0
    chunks_failed: int = 0
    embeddings_generated: int = 0
    embeddings_failed: int = 0
    chunks_stored: int = 0
    chunks_failed_storage: int = 0
    total_tokens: int = 0
    total_duration_seconds: float = 0.0

    def to_dict(self) -> dict:
        """Convert to dictionary representation."""
        return {
            "pages_crawled": self.pages_crawled,
            "pages_failed": self.pages_failed,
            "chunks_created": self.chunks_created,
            "chunks_failed": self.chunks_failed,
            "embeddings_generated": self.embeddings_generated,
            "embeddings_failed": self.embeddings_failed,
            "chunks_stored": self.chunks_stored,
            "chunks_failed_storage": self.chunks_failed_storage,
            "total_tokens": self.total_tokens,
            "total_duration_seconds": self.total_duration_seconds,
        }
