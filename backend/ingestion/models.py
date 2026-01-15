"""
Data models for RAG ingestion pipeline.

Defines DocumentChunk and Embedding dataclasses for type-safe data passing between pipeline stages.
"""

from dataclasses import dataclass, field
from typing import List, Optional
import uuid
from datetime import datetime


@dataclass
class DocumentChunk:
    """
    Represents a single text chunk extracted from documentation.

    Attributes:
        chunk_id: Unique identifier for this chunk
        text: The actual text content of the chunk
        source_url: URL of the document this chunk came from
        section: Document section/heading this chunk belongs to
        chunk_index: Sequential index of this chunk within its document
        token_count: Number of tokens in this chunk (for embedding models)
        metadata: Additional metadata key-value pairs
    """

    text: str
    source_url: str
    section: Optional[str] = None
    chunk_index: int = 0
    token_count: int = 0
    chunk_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    metadata: dict = field(default_factory=dict)
    created_at: datetime = field(default_factory=datetime.utcnow)

    def __post_init__(self):
        """Validate chunk after initialization."""
        if not self.text or not self.text.strip():
            raise ValueError("Chunk text cannot be empty")
        if not self.source_url:
            raise ValueError("Source URL is required")
        if self.token_count < 0:
            raise ValueError("Token count cannot be negative")
        if self.chunk_index < 0:
            raise ValueError("Chunk index cannot be negative")

    def __repr__(self) -> str:
        """String representation of chunk."""
        text_preview = self.text[:50].replace("\n", " ")
        return (
            f"DocumentChunk(id={self.chunk_id[:8]}..., "
            f"section={self.section}, "
            f"tokens={self.token_count}, "
            f"text='{text_preview}...')"
        )

    def to_dict(self) -> dict:
        """Convert chunk to dictionary for serialization."""
        return {
            "chunk_id": self.chunk_id,
            "text": self.text,
            "source_url": self.source_url,
            "section": self.section,
            "chunk_index": self.chunk_index,
            "token_count": self.token_count,
            "metadata": self.metadata,
            "created_at": self.created_at.isoformat(),
        }


@dataclass
class Embedding:
    """
    Represents a vector embedding for a DocumentChunk.

    Attributes:
        embedding_id: Unique identifier for this embedding
        chunk_id: Reference to the chunk this embedding is for
        vector: The embedding vector (list of floats)
        model: Embedding model used to generate this vector
        dimension: Dimensionality of the embedding vector
        metadata: Additional metadata (chunk text, URL, section, etc.)
    """

    vector: List[float]
    chunk_id: str
    model: str = "embed-english-v3.0"
    embedding_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    dimension: Optional[int] = None
    metadata: dict = field(default_factory=dict)
    created_at: datetime = field(default_factory=datetime.utcnow)

    def __post_init__(self):
        """Validate embedding after initialization."""
        if not self.vector or len(self.vector) == 0:
            raise ValueError("Embedding vector cannot be empty")
        if not all(isinstance(x, (int, float)) for x in self.vector):
            raise ValueError("All vector elements must be numeric")
        if not self.chunk_id:
            raise ValueError("Chunk ID is required")

        # Set dimension from vector length if not provided
        if self.dimension is None:
            self.dimension = len(self.vector)
        elif self.dimension != len(self.vector):
            raise ValueError(
                f"Vector dimension ({len(self.vector)}) does not match "
                f"specified dimension ({self.dimension})"
            )

    def __repr__(self) -> str:
        """String representation of embedding."""
        return (
            f"Embedding(id={self.embedding_id[:8]}..., "
            f"chunk_id={self.chunk_id[:8]}..., "
            f"dim={self.dimension}, "
            f"model={self.model})"
        )

    def to_dict(self) -> dict:
        """Convert embedding to dictionary for serialization."""
        return {
            "embedding_id": self.embedding_id,
            "chunk_id": self.chunk_id,
            "vector": self.vector,
            "model": self.model,
            "dimension": self.dimension,
            "metadata": self.metadata,
            "created_at": self.created_at.isoformat(),
        }


@dataclass
class VectorSearchResult:
    """
    Represents a single result from vector similarity search.

    Attributes:
        chunk_id: ID of the matching chunk
        chunk_text: The text content of the chunk
        source_url: URL where this chunk originated
        section: Document section of this chunk
        similarity_score: Cosine similarity score (0.0 to 1.0)
        rank: Rank in the result set (0 is most similar)
    """

    chunk_id: str
    chunk_text: str
    source_url: str
    similarity_score: float
    rank: int = 0
    section: Optional[str] = None

    def __post_init__(self):
        """Validate search result after initialization."""
        if not self.chunk_id:
            raise ValueError("Chunk ID is required")
        if not self.source_url:
            raise ValueError("Source URL is required")
        if not 0.0 <= self.similarity_score <= 1.0:
            raise ValueError("Similarity score must be between 0.0 and 1.0")
        if self.rank < 0:
            raise ValueError("Rank cannot be negative")

    def __repr__(self) -> str:
        """String representation of search result."""
        text_preview = self.chunk_text[:50].replace("\n", " ")
        return (
            f"VectorSearchResult(rank={self.rank}, "
            f"score={self.similarity_score:.3f}, "
            f"section={self.section}, "
            f"text='{text_preview}...')"
        )

    def to_dict(self) -> dict:
        """Convert result to dictionary for serialization."""
        return {
            "chunk_id": self.chunk_id,
            "chunk_text": self.chunk_text,
            "source_url": self.source_url,
            "section": self.section,
            "similarity_score": self.similarity_score,
            "rank": self.rank,
        }


@dataclass
class IngestionStats:
    """
    Statistics collected during pipeline execution.

    Attributes:
        pages_crawled: Number of pages successfully crawled
        pages_failed: Number of pages that failed to crawl
        chunks_created: Total number of chunks created
        tokens_generated: Total tokens across all chunks
        embeddings_created: Number of embeddings successfully generated
        embeddings_failed: Number of embeddings that failed
        docs_stored: Number of documents stored in Qdrant
        duration_seconds: Total pipeline execution time in seconds
    """

    pages_crawled: int = 0
    pages_failed: int = 0
    chunks_created: int = 0
    tokens_generated: int = 0
    embeddings_created: int = 0
    embeddings_failed: int = 0
    docs_stored: int = 0
    duration_seconds: float = 0.0

    def __repr__(self) -> str:
        """String representation of stats."""
        return (
            f"IngestionStats("
            f"pages={self.pages_crawled}/{self.pages_failed}, "
            f"chunks={self.chunks_created}, "
            f"embeddings={self.embeddings_created}/{self.embeddings_failed}, "
            f"stored={self.docs_stored}, "
            f"time={self.duration_seconds:.1f}s)"
        )

    def to_dict(self) -> dict:
        """Convert stats to dictionary for reporting."""
        return {
            "pages_crawled": self.pages_crawled,
            "pages_failed": self.pages_failed,
            "chunks_created": self.chunks_created,
            "tokens_generated": self.tokens_generated,
            "embeddings_created": self.embeddings_created,
            "embeddings_failed": self.embeddings_failed,
            "docs_stored": self.docs_stored,
            "duration_seconds": self.duration_seconds,
        }

    def pages_total(self) -> int:
        """Get total pages (crawled + failed)."""
        return self.pages_crawled + self.pages_failed

    def success_rate(self) -> float:
        """Get success rate as percentage (0-100)."""
        total = self.pages_total()
        if total == 0:
            return 0.0
        return (self.pages_crawled / total) * 100
