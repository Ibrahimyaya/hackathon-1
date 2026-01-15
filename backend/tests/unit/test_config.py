"""
Unit tests for configuration loading and validation.

Tests the Config class with valid and invalid inputs.
"""

import pytest
import os
from pathlib import Path
import tempfile
from pydantic import ValidationError

# Add parent directory for imports
import sys
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from utils.config import Config, load_config


class TestConfigValidation:
    """Test configuration validation."""

    def test_valid_config_with_required_vars(self):
        """Test that valid config loads successfully."""
        # Set environment variables for testing
        os.environ["DOCS_URL"] = "https://docs.example.com"
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test.qdrant.io:6333"

        try:
            config = Config()
            assert config.docs_url == "https://docs.example.com"
            assert config.cohere_api_key == "test-key"
            assert config.qdrant_api_key == "test-key"
        finally:
            # Cleanup
            for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL"]:
                if key in os.environ:
                    del os.environ[key]

    def test_missing_required_docs_url(self):
        """Test that missing DOCS_URL raises validation error."""
        # Clear the variable
        for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL"]:
            if key in os.environ:
                del os.environ[key]

        with pytest.raises(ValidationError):
            Config()

    def test_default_values(self):
        """Test that optional config values have correct defaults."""
        os.environ["DOCS_URL"] = "https://docs.example.com"
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test.qdrant.io:6333"

        try:
            config = Config()
            assert config.crawl_max_pages == 1000
            assert config.crawl_timeout_seconds == 10
            assert config.chunk_min_tokens == 256
            assert config.chunk_max_tokens == 512
            assert config.log_level == "INFO"
            assert config.log_format == "json"
        finally:
            for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL"]:
                if key in os.environ:
                    del os.environ[key]

    def test_valid_log_level(self):
        """Test that valid log levels are accepted."""
        os.environ["DOCS_URL"] = "https://docs.example.com"
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test.qdrant.io:6333"
        os.environ["LOG_LEVEL"] = "DEBUG"

        try:
            config = Config()
            assert config.log_level == "DEBUG"
        finally:
            for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL", "LOG_LEVEL"]:
                if key in os.environ:
                    del os.environ[key]

    def test_invalid_log_level(self):
        """Test that invalid log level raises validation error."""
        os.environ["DOCS_URL"] = "https://docs.example.com"
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test.qdrant.io:6333"
        os.environ["LOG_LEVEL"] = "INVALID"

        try:
            with pytest.raises(ValidationError):
                Config()
        finally:
            for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL", "LOG_LEVEL"]:
                if key in os.environ:
                    del os.environ[key]

    def test_valid_log_format(self):
        """Test that valid log format is accepted."""
        os.environ["DOCS_URL"] = "https://docs.example.com"
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test.qdrant.io:6333"
        os.environ["LOG_FORMAT"] = "human"

        try:
            config = Config()
            assert config.log_format == "human"
        finally:
            for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL", "LOG_FORMAT"]:
                if key in os.environ:
                    del os.environ[key]

    def test_invalid_log_format(self):
        """Test that invalid log format raises validation error."""
        os.environ["DOCS_URL"] = "https://docs.example.com"
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test.qdrant.io:6333"
        os.environ["LOG_FORMAT"] = "xml"

        try:
            with pytest.raises(ValidationError):
                Config()
        finally:
            for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL", "LOG_FORMAT"]:
                if key in os.environ:
                    del os.environ[key]

    def test_chunk_size_validation(self):
        """Test that chunk min and max are validated correctly."""
        os.environ["DOCS_URL"] = "https://docs.example.com"
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test.qdrant.io:6333"
        os.environ["CHUNK_MIN_TOKENS"] = "1000"  # Greater than max
        os.environ["CHUNK_MAX_TOKENS"] = "512"

        try:
            with pytest.raises(ValidationError):
                Config()
        finally:
            for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL",
                        "CHUNK_MIN_TOKENS", "CHUNK_MAX_TOKENS"]:
                if key in os.environ:
                    del os.environ[key]

    def test_numeric_range_validation(self):
        """Test that numeric ranges are validated."""
        os.environ["DOCS_URL"] = "https://docs.example.com"
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test.qdrant.io:6333"
        os.environ["CRAWL_MAX_PAGES"] = "0"  # Invalid: must be >= 1

        try:
            with pytest.raises(ValidationError):
                Config()
        finally:
            for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL", "CRAWL_MAX_PAGES"]:
                if key in os.environ:
                    del os.environ[key]


class TestLoadConfig:
    """Test the load_config helper function."""

    def test_load_config_with_valid_env(self):
        """Test loading config with valid environment."""
        os.environ["DOCS_URL"] = "https://docs.example.com"
        os.environ["COHERE_API_KEY"] = "test-key"
        os.environ["QDRANT_API_KEY"] = "test-key"
        os.environ["QDRANT_URL"] = "https://test.qdrant.io:6333"

        try:
            config = load_config()
            assert config.docs_url == "https://docs.example.com"
        finally:
            for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL"]:
                if key in os.environ:
                    del os.environ[key]

    def test_load_config_with_invalid_env(self):
        """Test that load_config raises ValueError with clear message for missing vars."""
        for key in ["DOCS_URL", "COHERE_API_KEY", "QDRANT_API_KEY", "QDRANT_URL"]:
            if key in os.environ:
                del os.environ[key]

        with pytest.raises(ValueError) as exc_info:
            load_config()

        # Check that error message includes helpful guidance
        assert "Configuration validation failed" in str(exc_info.value)
        assert "Required environment variables" in str(exc_info.value)
