"""Custom exception hierarchy for RAG ingestion pipeline."""


class IngestionError(Exception):
    """Base exception for all ingestion-related errors."""

    pass


class CrawlError(IngestionError):
    """Base exception for crawling errors."""

    pass


class CrawlTimeoutError(CrawlError):
    """Raised when a page crawl times out."""

    pass


class CrawlNetworkError(CrawlError):
    """Raised when network errors occur during crawling."""

    pass


class CrawlValidationError(CrawlError):
    """Raised when crawl validation fails."""

    pass


class ProcessingError(IngestionError):
    """Base exception for text processing errors."""

    pass


class ExtractionError(ProcessingError):
    """Raised when HTML extraction fails."""

    pass


class CleaningError(ProcessingError):
    """Raised when text cleaning fails."""

    pass


class ChunkingError(IngestionError):
    """Base exception for chunking errors."""

    pass


class TokenizationError(ChunkingError):
    """Raised when tokenization fails."""

    pass


class ChunkingStrategyError(ChunkingError):
    """Raised when chunking strategy fails."""

    pass


class EmbeddingError(IngestionError):
    """Base exception for embedding errors."""

    pass


class CohereAPIError(EmbeddingError):
    """Raised when Cohere API returns an error."""

    pass


class CohereQuotaError(EmbeddingError):
    """Raised when Cohere quota is exceeded."""

    pass


class StorageError(IngestionError):
    """Base exception for storage errors."""

    pass


class QdrantConnectionError(StorageError):
    """Raised when Qdrant connection fails."""

    pass


class QdrantCollectionError(StorageError):
    """Raised when Qdrant collection operations fail."""

    pass


class QdrantQueryError(StorageError):
    """Raised when Qdrant query operations fail."""

    pass
