"""
Custom exception hierarchy for RAG ingestion pipeline.

Provides granular error types for different stages of the pipeline,
enabling precise error handling and reporting.
"""


class IngestionError(Exception):
    """Base exception for all ingestion pipeline errors."""

    def __init__(self, message: str, context: dict = None, **kwargs):
        """
        Initialize ingestion error.

        Args:
            message: Error message
            context: Optional context dictionary with additional details
            **kwargs: Additional named arguments for context
        """
        super().__init__(message)
        self.message = message
        self.context = context or {}
        self.context.update(kwargs)

    def __str__(self) -> str:
        """Return formatted error message with context."""
        result = self.message
        if self.context:
            context_str = ", ".join(f"{k}={v}" for k, v in self.context.items())
            result += f" ({context_str})"
        return result


# ============================================================================
# Crawling Errors
# ============================================================================

class CrawlError(IngestionError):
    """Base exception for crawling-related errors."""
    pass


class CrawlTimeoutError(CrawlError):
    """Raised when a page fetch times out."""
    pass


class CrawlNetworkError(CrawlError):
    """Raised when there's a network error during crawling."""
    pass


class CrawlValidationError(CrawlError):
    """Raised when crawled content fails validation."""
    pass


class CrawlRateLimitError(CrawlError):
    """Raised when rate limit is exceeded during crawling."""
    pass


# ============================================================================
# Processing Errors (HTML Extraction)
# ============================================================================

class ProcessingError(IngestionError):
    """Base exception for text processing errors."""
    pass


class ExtractionError(ProcessingError):
    """Raised when text extraction from HTML fails."""
    pass


class CleaningError(ProcessingError):
    """Raised when text cleaning fails."""
    pass


class ValidationError(ProcessingError):
    """Raised when extracted text fails validation."""
    pass


# ============================================================================
# Chunking Errors
# ============================================================================

class ChunkingError(IngestionError):
    """Base exception for text chunking errors."""
    pass


class TokenizationError(ChunkingError):
    """Raised when token counting fails."""
    pass


class ChunkingStrategyError(ChunkingError):
    """Raised when chunking strategy fails to produce valid chunks."""
    pass


# ============================================================================
# Embedding Errors (Cohere API)
# ============================================================================

class EmbeddingError(IngestionError):
    """Base exception for embedding generation errors."""
    pass


class CohereAPIError(EmbeddingError):
    """Raised when Cohere API call fails."""
    pass


class CohereAuthenticationError(CohereAPIError):
    """Raised when Cohere authentication fails (invalid API key)."""
    pass


class CohereQuotaError(EmbeddingError):
    """Raised when Cohere API quota is exceeded."""
    pass


class CohereRateLimitError(EmbeddingError):
    """Raised when Cohere API rate limit is exceeded."""
    pass


class EmbeddingBatchError(EmbeddingError):
    """Raised when batch embedding fails."""
    pass


# ============================================================================
# Storage Errors (Qdrant)
# ============================================================================

class StorageError(IngestionError):
    """Base exception for vector storage errors."""
    pass


class QdrantConnectionError(StorageError):
    """Raised when connection to Qdrant fails."""
    pass


class QdrantAuthenticationError(StorageError):
    """Raised when Qdrant authentication fails."""
    pass


class QdrantCollectionError(StorageError):
    """Raised when Qdrant collection operations fail."""
    pass


class QdrantInsertError(StorageError):
    """Raised when inserting embeddings into Qdrant fails."""
    pass


class QdrantQueryError(StorageError):
    """Raised when querying Qdrant fails."""
    pass


class QdrantQuotaError(StorageError):
    """Raised when Qdrant storage quota is exceeded."""
    pass


# ============================================================================
# Configuration Errors
# ============================================================================

class ConfigurationError(IngestionError):
    """Raised when configuration is invalid or missing."""
    pass


# ============================================================================
# Pipeline Errors
# ============================================================================

class PipelineError(IngestionError):
    """Raised when pipeline execution fails."""
    pass


class PipelineValidationError(PipelineError):
    """Raised when pipeline validation fails."""
    pass


# ============================================================================
# Error Context Builders
# ============================================================================

def build_crawl_error(error_type: type, url: str, message: str, **kwargs) -> IngestionError:
    """Build a crawl error with standard context."""
    context = {"url": url, **kwargs}
    return error_type(message, context=context)


def build_processing_error(error_type: type, url: str, message: str, **kwargs) -> IngestionError:
    """Build a processing error with standard context."""
    context = {"url": url, **kwargs}
    return error_type(message, context=context)


def build_chunking_error(error_type: type, message: str, **kwargs) -> IngestionError:
    """Build a chunking error with standard context."""
    return error_type(message, context=kwargs)


def build_embedding_error(error_type: type, chunk_id: str, message: str, **kwargs) -> IngestionError:
    """Build an embedding error with standard context."""
    context = {"chunk_id": chunk_id, **kwargs}
    return error_type(message, context=context)


def build_storage_error(error_type: type, message: str, **kwargs) -> IngestionError:
    """Build a storage error with standard context."""
    return error_type(message, context=kwargs)
