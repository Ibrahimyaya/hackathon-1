"""
pytest configuration and fixtures for RAG ingestion pipeline tests.

Provides reusable fixtures for config, logging, and mocked API responses.
"""

import pytest
import os
from unittest.mock import Mock, MagicMock
from pathlib import Path
import sys

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.config import Config
from utils.logging import setup_logging, get_logger
from ingestion.models import DocumentChunk, Embedding, VectorSearchResult


# ============================================================================
# Configuration Fixtures
# ============================================================================

@pytest.fixture
def test_config():
    """Provide a test configuration with default values."""
    # Create test .env file
    test_env_content = """
DOCS_URL=https://docs.example.com
CRAWL_MAX_PAGES=10
CRAWL_TIMEOUT_SECONDS=5
CHUNK_MIN_TOKENS=256
CHUNK_MAX_TOKENS=512
COHERE_API_KEY=test-cohere-key
QDRANT_API_KEY=test-qdrant-key
QDRANT_URL=https://test-cluster.qdrant.io:6333
LOG_LEVEL=INFO
LOG_FORMAT=human
"""
    # For testing, we'd normally use environment variables or a test .env
    # For now, return a Config-like dict
    return {
        "docs_url": "https://docs.example.com",
        "crawl_max_pages": 10,
        "crawl_timeout_seconds": 5,
        "chunk_min_tokens": 256,
        "chunk_max_tokens": 512,
        "cohere_api_key": "test-cohere-key",
        "qdrant_api_key": "test-qdrant-key",
        "qdrant_url": "https://test-cluster.qdrant.io:6333",
        "log_level": "INFO",
        "log_format": "human",
    }


# ============================================================================
# Logging Fixtures
# ============================================================================

@pytest.fixture
def test_logger(test_config):
    """Set up test logger with human-readable format."""
    setup_logging(
        log_level=test_config["log_level"],
        log_format=test_config["log_format"],
        debug=False
    )
    return get_logger("test")


# ============================================================================
# Mock API Fixtures
# ============================================================================

@pytest.fixture
def mock_cohere_client():
    """Provide a mocked Cohere API client."""
    mock_client = MagicMock()

    # Mock successful embedding response
    mock_response = MagicMock()
    mock_response.embeddings = [
        [0.1] * 1024 for _ in range(10)  # 10 mock embeddings of dimension 1024
    ]

    mock_client.embed.return_value = mock_response
    return mock_client


@pytest.fixture
def mock_qdrant_client():
    """Provide a mocked Qdrant client."""
    mock_client = MagicMock()

    # Mock collection operations
    mock_client.collection_exists.return_value = True
    mock_client.get_collection.return_value = MagicMock(
        points_count=100,
        vectors_count=100,
    )

    # Mock upsert operation
    mock_client.upsert.return_value = None

    # Mock search operation
    mock_search_result = MagicMock()
    mock_search_result.id = "chunk-1"
    mock_search_result.score = 0.95
    mock_search_result.payload = {
        "chunk_id": "chunk-1",
        "text": "Test chunk text",
        "source_url": "https://example.com",
        "section": "Introduction",
    }

    mock_client.search.return_value = [mock_search_result]

    return mock_client


@pytest.fixture
def mock_requests():
    """Provide mocked requests for HTTP crawling tests."""
    mock_session = MagicMock()

    # Mock successful GET response
    mock_response = MagicMock()
    mock_response.status_code = 200
    mock_response.text = "<html><body><h1>Test</h1><p>Content</p></body></html>"
    mock_response.url = "https://example.com/page1"

    mock_session.get.return_value = mock_response
    return mock_session


# ============================================================================
# Model Fixtures
# ============================================================================

@pytest.fixture
def sample_document_chunk():
    """Provide a sample DocumentChunk for testing."""
    return DocumentChunk(
        text="This is a sample documentation chunk with test content.",
        source_url="https://docs.example.com/guide",
        section="Getting Started",
        chunk_index=0,
        token_count=10,
    )


@pytest.fixture
def sample_chunks():
    """Provide multiple sample DocumentChunks for testing."""
    return [
        DocumentChunk(
            text="Installation guide for the system.",
            source_url="https://docs.example.com/install",
            section="Installation",
            chunk_index=0,
            token_count=6,
        ),
        DocumentChunk(
            text="Configuration options and environment variables.",
            source_url="https://docs.example.com/config",
            section="Configuration",
            chunk_index=0,
            token_count=7,
        ),
        DocumentChunk(
            text="API endpoints and usage examples.",
            source_url="https://docs.example.com/api",
            section="API Reference",
            chunk_index=0,
            token_count=6,
        ),
    ]


@pytest.fixture
def sample_embedding(sample_document_chunk):
    """Provide a sample Embedding for testing."""
    return Embedding(
        vector=[0.1] * 1024,  # 1024-dimensional vector (Cohere standard)
        chunk_id=sample_document_chunk.chunk_id,
        model="embed-english-v3.0",
        metadata={
            "chunk_text": sample_document_chunk.text,
            "source_url": sample_document_chunk.source_url,
        },
    )


@pytest.fixture
def sample_search_results():
    """Provide sample vector search results for testing."""
    return [
        VectorSearchResult(
            chunk_id="chunk-1",
            chunk_text="How to install the system.",
            source_url="https://docs.example.com/install",
            section="Installation",
            similarity_score=0.98,
            rank=0,
        ),
        VectorSearchResult(
            chunk_id="chunk-2",
            chunk_text="Installation requirements and dependencies.",
            source_url="https://docs.example.com/install",
            section="Installation",
            similarity_score=0.92,
            rank=1,
        ),
        VectorSearchResult(
            chunk_id="chunk-3",
            chunk_text="Getting started with the system.",
            source_url="https://docs.example.com/guide",
            section="Getting Started",
            similarity_score=0.75,
            rank=2,
        ),
    ]


# ============================================================================
# Pytest Hooks & Configuration
# ============================================================================

def pytest_configure(config):
    """Configure pytest with custom markers."""
    config.addinivalue_line(
        "markers",
        "integration: mark test as an integration test (requires external services)"
    )
    config.addinivalue_line(
        "markers",
        "unit: mark test as a unit test (no external services)"
    )
    config.addinivalue_line(
        "markers",
        "slow: mark test as slow (takes > 1 second)"
    )


# ============================================================================
# Autouse Fixtures
# ============================================================================

@pytest.fixture(autouse=True)
def reset_logging():
    """Reset logging after each test."""
    yield
    # Reset logging configuration
    import logging
    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)
