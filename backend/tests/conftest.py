"""Shared pytest fixtures for all tests."""

import os
from unittest.mock import MagicMock

import pytest

from utils.config import Config
from utils.logging import setup_logging


@pytest.fixture
def config():
    """Provide a test configuration object."""
    # Create a minimal config for testing
    os.environ["DOCS_URL"] = "https://test.example.com"
    os.environ["COHERE_API_KEY"] = "test-cohere-key"
    os.environ["QDRANT_API_KEY"] = "test-qdrant-key"
    os.environ["QDRANT_URL"] = "https://test.qdrant.io"
    os.environ["LOG_LEVEL"] = "DEBUG"
    os.environ["LOG_FORMAT"] = "human"

    config_obj = Config()
    return config_obj


@pytest.fixture
def logger(config):
    """Provide a configured logger for tests."""
    return setup_logging(config)


@pytest.fixture
def mock_cohere_client():
    """Provide a mocked Cohere client."""
    mock = MagicMock()
    mock.embed.return_value = MagicMock(
        embeddings=[[0.1, 0.2, 0.3] * 341 + [0.1]]  # 1024-dimensional vector
    )
    return mock


@pytest.fixture
def mock_qdrant_client():
    """Provide a mocked Qdrant client."""
    mock = MagicMock()
    mock.collection_exists.return_value = True
    mock.search.return_value = MagicMock(points=[])
    return mock


@pytest.fixture
def sample_html():
    """Provide sample HTML for testing extraction."""
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Test Page</title>
    </head>
    <body>
        <nav>Navigation Bar</nav>
        <main>
            <h1>Main Heading</h1>
            <p>This is test content.</p>
            <h2>Subheading</h2>
            <p>More content here.</p>
            <code>import test</code>
        </main>
        <footer>Footer</footer>
    </body>
    </html>
    """


@pytest.fixture
def sample_text():
    """Provide sample text for testing chunking."""
    return """
    Introduction to the System

    This is the introduction paragraph. It contains important information about the system.

    First Section

    This section covers the first topic. The content discusses various aspects of the topic.

    Second Section

    This section covers the second topic. Additional details are provided here.

    Conclusion

    This is the conclusion of the document.
    """
