"""Tests for configuration loading and validation."""

import os

import pytest

from utils.config import Config, load_config
from utils.errors import IngestionError


def test_config_loads_from_env(config):
    """Test that config loads successfully with required environment variables."""
    assert config.docs_url == "https://test.example.com"
    assert config.cohere_api_key == "test-cohere-key"
    assert config.qdrant_api_key == "test-qdrant-key"
    assert config.qdrant_url == "https://test.qdrant.io"


def test_config_defaults():
    """Test that config uses sensible defaults."""
    config = Config(
        docs_url="https://example.com",
        cohere_api_key="key",
        qdrant_api_key="key",
        qdrant_url="https://qdrant.io",
    )
    assert config.crawl_max_pages == 1000
    assert config.chunk_min_tokens == 256
    assert config.chunk_max_tokens == 512
    assert config.log_level == "INFO"
    assert config.log_format == "json"


def test_config_missing_required_var():
    """Test that missing required variables raise clear error."""
    with pytest.raises(ValueError) as exc_info:
        Config(
            cohere_api_key="key",
            qdrant_api_key="key",
            qdrant_url="https://qdrant.io",
            # Missing docs_url
        )
    assert "docs_url" in str(exc_info.value).lower()


def test_config_invalid_crawl_max_pages():
    """Test that invalid crawl_max_pages raises error."""
    with pytest.raises(ValueError):
        Config(
            docs_url="https://example.com",
            cohere_api_key="key",
            qdrant_api_key="key",
            qdrant_url="https://qdrant.io",
            crawl_max_pages=0,
        )


def test_config_invalid_log_format():
    """Test that invalid log_format raises error."""
    with pytest.raises(ValueError):
        Config(
            docs_url="https://example.com",
            cohere_api_key="key",
            qdrant_api_key="key",
            qdrant_url="https://qdrant.io",
            log_format="invalid",
        )


def test_config_invalid_log_level():
    """Test that invalid log_level raises error."""
    with pytest.raises(ValueError):
        Config(
            docs_url="https://example.com",
            cohere_api_key="key",
            qdrant_api_key="key",
            qdrant_url="https://qdrant.io",
            log_level="INVALID",
        )


def test_load_config_function(config):
    """Test the load_config() helper function."""
    loaded = load_config()
    assert isinstance(loaded, Config)
    assert loaded.docs_url is not None
