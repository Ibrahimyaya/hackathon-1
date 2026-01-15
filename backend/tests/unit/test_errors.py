"""
Unit tests for exception hierarchy.

Tests that all custom exceptions work correctly and provide useful error messages.
"""

import pytest
from pathlib import Path

# Add parent directory for imports
import sys
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from utils.errors import (
    IngestionError,
    CrawlError, CrawlTimeoutError, CrawlNetworkError,
    ProcessingError, ExtractionError, CleaningError,
    ChunkingError, TokenizationError,
    EmbeddingError, CohereAPIError, CohereQuotaError,
    StorageError, QdrantConnectionError,
    build_crawl_error, build_processing_error, build_chunking_error,
)


class TestBasicException:
    """Test base exception class."""

    def test_ingestion_error_basic(self):
        """Test basic IngestionError."""
        error = IngestionError("Test error message")
        assert str(error) == "Test error message"
        assert error.message == "Test error message"

    def test_ingestion_error_with_context(self):
        """Test IngestionError with context dictionary."""
        error = IngestionError("Test error", context={"url": "https://example.com", "page": 1})
        assert "url=https://example.com" in str(error)
        assert "page=1" in str(error)

    def test_ingestion_error_with_kwargs(self):
        """Test IngestionError with keyword arguments."""
        error = IngestionError("Test error", url="https://example.com", page=1)
        assert "url=https://example.com" in str(error)
        assert "page=1" in str(error)


class TestCrawlErrors:
    """Test crawl-related exceptions."""

    def test_crawl_error(self):
        """Test basic CrawlError."""
        error = CrawlError("Crawl failed")
        assert isinstance(error, IngestionError)

    def test_crawl_timeout_error(self):
        """Test CrawlTimeoutError."""
        error = CrawlTimeoutError("Request timed out", url="https://example.com")
        assert isinstance(error, CrawlError)
        assert "url=https://example.com" in str(error)

    def test_crawl_network_error(self):
        """Test CrawlNetworkError."""
        error = CrawlNetworkError("Network error", url="https://example.com", retry=1)
        assert isinstance(error, CrawlError)
        assert "retry=1" in str(error)

    def test_build_crawl_error(self):
        """Test build_crawl_error helper."""
        error = build_crawl_error(
            CrawlNetworkError,
            url="https://example.com",
            message="Connection refused"
        )
        assert isinstance(error, CrawlNetworkError)
        assert "url=https://example.com" in str(error)


class TestProcessingErrors:
    """Test text processing exceptions."""

    def test_processing_error(self):
        """Test basic ProcessingError."""
        error = ProcessingError("Processing failed")
        assert isinstance(error, IngestionError)

    def test_extraction_error(self):
        """Test ExtractionError."""
        error = ExtractionError(
            "Failed to extract text",
            url="https://example.com",
            reason="No main content found"
        )
        assert isinstance(error, ProcessingError)
        assert "reason=No main content found" in str(error)

    def test_cleaning_error(self):
        """Test CleaningError."""
        error = CleaningError("Text cleaning failed")
        assert isinstance(error, ProcessingError)

    def test_build_processing_error(self):
        """Test build_processing_error helper."""
        error = build_processing_error(
            ExtractionError,
            url="https://example.com",
            message="Extraction failed",
            reason="Empty content"
        )
        assert isinstance(error, ExtractionError)
        assert "reason=Empty content" in str(error)


class TestChunkingErrors:
    """Test chunking-related exceptions."""

    def test_chunking_error(self):
        """Test basic ChunkingError."""
        error = ChunkingError("Chunking failed")
        assert isinstance(error, IngestionError)

    def test_tokenization_error(self):
        """Test TokenizationError."""
        error = TokenizationError("Token counting failed", text_length=1000)
        assert isinstance(error, ChunkingError)
        assert "text_length=1000" in str(error)

    def test_build_chunking_error(self):
        """Test build_chunking_error helper."""
        error = build_chunking_error(
            TokenizationError,
            message="Token count invalid",
            chunk_index=5
        )
        assert isinstance(error, TokenizationError)
        assert "chunk_index=5" in str(error)


class TestEmbeddingErrors:
    """Test embedding-related exceptions."""

    def test_embedding_error(self):
        """Test basic EmbeddingError."""
        error = EmbeddingError("Embedding failed")
        assert isinstance(error, IngestionError)

    def test_cohere_api_error(self):
        """Test CohereAPIError."""
        error = CohereAPIError("API request failed", status_code=500, chunk_count=10)
        assert isinstance(error, EmbeddingError)
        assert "status_code=500" in str(error)

    def test_cohere_quota_error(self):
        """Test CohereQuotaError."""
        error = CohereQuotaError("Quota exceeded", remaining=0)
        assert isinstance(error, EmbeddingError)
        assert "remaining=0" in str(error)


class TestStorageErrors:
    """Test storage-related exceptions."""

    def test_storage_error(self):
        """Test basic StorageError."""
        error = StorageError("Storage failed")
        assert isinstance(error, IngestionError)

    def test_qdrant_connection_error(self):
        """Test QdrantConnectionError."""
        error = QdrantConnectionError(
            "Cannot connect to Qdrant",
            url="https://test.qdrant.io:6333",
            timeout=30
        )
        assert isinstance(error, StorageError)
        assert "timeout=30" in str(error)


class TestExceptionHierarchy:
    """Test exception hierarchy relationships."""

    def test_all_inherit_from_base(self):
        """Test that all custom exceptions inherit from IngestionError."""
        exceptions = [
            CrawlError(),
            CrawlTimeoutError("test"),
            ProcessingError("test"),
            ExtractionError("test"),
            ChunkingError("test"),
            EmbeddingError("test"),
            StorageError("test"),
        ]

        for exc in exceptions:
            assert isinstance(exc, IngestionError)

    def test_exception_raising(self):
        """Test that exceptions can be raised and caught properly."""
        with pytest.raises(CrawlError):
            raise CrawlTimeoutError("Timeout")

        with pytest.raises(IngestionError):
            raise CohereQuotaError("Quota exceeded")

        with pytest.raises(StorageError):
            raise QdrantConnectionError("Connection failed")


class TestErrorMessages:
    """Test error message formatting."""

    def test_error_message_formatting(self):
        """Test that error messages format correctly with context."""
        error = IngestionError(
            "Pipeline failed",
            context={
                "stage": "embedding",
                "chunk_id": "chunk-123",
                "timestamp": "2026-01-15T10:00:00"
            }
        )

        error_str = str(error)
        assert "Pipeline failed" in error_str
        assert "stage=embedding" in error_str
        assert "chunk_id=chunk-123" in error_str
        assert "timestamp" in error_str

    def test_error_without_context(self):
        """Test that errors without context format cleanly."""
        error = IngestionError("Simple error")
        assert str(error) == "Simple error"
