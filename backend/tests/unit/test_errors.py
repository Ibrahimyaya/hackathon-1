"""Tests for custom exception hierarchy."""

import pytest

from utils.errors import (
    ChunkingError,
    CohereAPIError,
    CohereQuotaError,
    CrawlError,
    CrawlNetworkError,
    CrawlTimeoutError,
    CrawlValidationError,
    EmbeddingError,
    ExtractionError,
    IngestionError,
    ProcessingError,
    QdrantCollectionError,
    QdrantConnectionError,
    QdrantQueryError,
    StorageError,
    TokenizationError,
)


def test_ingestion_error_is_exception():
    """Test that IngestionError is an Exception."""
    assert issubclass(IngestionError, Exception)


def test_crawl_error_hierarchy():
    """Test CrawlError and its subclasses."""
    assert issubclass(CrawlError, IngestionError)
    assert issubclass(CrawlTimeoutError, CrawlError)
    assert issubclass(CrawlNetworkError, CrawlError)
    assert issubclass(CrawlValidationError, CrawlError)


def test_processing_error_hierarchy():
    """Test ProcessingError and its subclasses."""
    assert issubclass(ProcessingError, IngestionError)
    assert issubclass(ExtractionError, ProcessingError)


def test_chunking_error_hierarchy():
    """Test ChunkingError and its subclasses."""
    assert issubclass(ChunkingError, IngestionError)
    assert issubclass(TokenizationError, ChunkingError)


def test_embedding_error_hierarchy():
    """Test EmbeddingError and its subclasses."""
    assert issubclass(EmbeddingError, IngestionError)
    assert issubclass(CohereAPIError, EmbeddingError)
    assert issubclass(CohereQuotaError, EmbeddingError)


def test_storage_error_hierarchy():
    """Test StorageError and its subclasses."""
    assert issubclass(StorageError, IngestionError)
    assert issubclass(QdrantConnectionError, StorageError)
    assert issubclass(QdrantCollectionError, StorageError)
    assert issubclass(QdrantQueryError, StorageError)


def test_crawl_timeout_error_with_message():
    """Test raising CrawlTimeoutError with message."""
    with pytest.raises(CrawlTimeoutError) as exc_info:
        raise CrawlTimeoutError("Timeout crawling https://example.com")
    assert "Timeout" in str(exc_info.value)


def test_cohere_quota_error_with_message():
    """Test raising CohereQuotaError with message."""
    with pytest.raises(CohereQuotaError) as exc_info:
        raise CohereQuotaError("Cohere quota exceeded")
    assert "quota" in str(exc_info.value).lower()


def test_qdrant_connection_error_with_message():
    """Test raising QdrantConnectionError with message."""
    with pytest.raises(QdrantConnectionError) as exc_info:
        raise QdrantConnectionError("Cannot connect to Qdrant")
    assert "Cannot connect" in str(exc_info.value)


def test_catch_base_ingestion_error():
    """Test catching all errors with base IngestionError."""
    errors = [
        CrawlTimeoutError("timeout"),
        TokenizationError("tokenization"),
        CohereAPIError("api"),
        QdrantConnectionError("connection"),
    ]

    for error in errors:
        with pytest.raises(IngestionError):
            raise error
