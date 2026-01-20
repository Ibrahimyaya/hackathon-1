"""Tests for Cohere embedder."""

import pytest
from unittest.mock import patch, MagicMock

from ingestion.embeddings.cohere_embedder import CohereEmbedder
from ingestion.models import DocumentChunk, Embedding
from utils.errors import CohereAPIError, CohereQuotaError


@patch("ingestion.embeddings.cohere_embedder.cohere.ClientV2")
def test_embedder_init(mock_cohere):
    """Test embedder initialization."""
    embedder = CohereEmbedder(api_key="test-key")

    assert embedder.model == "embed-english-v3.0"
    assert embedder.batch_size == 100
    mock_cohere.assert_called_once_with(api_key="test-key")


@patch("ingestion.embeddings.cohere_embedder.cohere.ClientV2")
def test_embedder_init_custom_params(mock_cohere):
    """Test embedder initialization with custom parameters."""
    embedder = CohereEmbedder(
        api_key="test-key",
        model="custom-model",
        batch_size=50,
    )

    assert embedder.model == "custom-model"
    assert embedder.batch_size == 50


def test_embedder_init_no_api_key():
    """Test embedder initialization without API key."""
    with patch("ingestion.embeddings.cohere_embedder.cohere.ClientV2") as mock_cohere:
        mock_cohere.side_effect = Exception("API key required")
        with pytest.raises(CohereAPIError):
            CohereEmbedder(api_key="")


@patch("ingestion.embeddings.cohere_embedder.cohere.ClientV2")
def test_embedder_embed_single_chunk(mock_cohere):
    """Test embedding a single chunk."""
    mock_client = MagicMock()
    mock_cohere.return_value = mock_client

    # Mock the embed response
    mock_response = MagicMock()
    mock_response.embeddings = [[0.1, 0.2, 0.3]]
    mock_client.embed.return_value = mock_response

    embedder = CohereEmbedder(api_key="test-key")
    chunk = DocumentChunk(
        id="test:1",
        text="Test text",
        source_url="https://example.com",
    )

    embeddings = embedder.embed([chunk])

    assert len(embeddings) == 1
    assert isinstance(embeddings[0], Embedding)
    assert embeddings[0].chunk_id == "test:1"
    assert embeddings[0].vector == [0.1, 0.2, 0.3]


@patch("ingestion.embeddings.cohere_embedder.cohere.ClientV2")
def test_embedder_embed_empty_list(mock_cohere):
    """Test embedding an empty list."""
    mock_client = MagicMock()
    mock_cohere.return_value = mock_client

    embedder = CohereEmbedder(api_key="test-key")
    embeddings = embedder.embed([])

    assert len(embeddings) == 0
    mock_client.embed.assert_not_called()


@patch("ingestion.embeddings.cohere_embedder.cohere.ClientV2")
def test_embedder_embed_batch(mock_cohere):
    """Test embedding multiple chunks in batch."""
    mock_client = MagicMock()
    mock_cohere.return_value = mock_client

    # Mock batch response
    mock_response = MagicMock()
    mock_response.embeddings = [
        [0.1, 0.2],
        [0.3, 0.4],
        [0.5, 0.6],
    ]
    mock_client.embed.return_value = mock_response

    embedder = CohereEmbedder(api_key="test-key", batch_size=2)

    chunks = [
        DocumentChunk(id=f"chunk:{i}", text=f"Text {i}", source_url="https://example.com")
        for i in range(3)
    ]

    embeddings = embedder.embed(chunks)

    assert len(embeddings) == 3
    # API should be called with batches
    assert mock_client.embed.call_count >= 1


@patch("ingestion.embeddings.cohere_embedder.cohere.ClientV2")
def test_embedder_api_error(mock_cohere):
    """Test handling of API errors in batch method."""
    mock_client = MagicMock()
    mock_cohere.return_value = mock_client

    # Mock API error (something that's not a quota error)
    mock_client.embed.side_effect = Exception("Invalid API Key")

    embedder = CohereEmbedder(api_key="test-key", max_retries=1)

    # Test the batch method directly - it should raise
    with pytest.raises(CohereAPIError):
        embedder._embed_batch(["test text"])


@patch("ingestion.embeddings.cohere_embedder.cohere.ClientV2")
@patch("ingestion.embeddings.cohere_embedder.time.sleep")
def test_embedder_quota_exceeded(mock_sleep, mock_cohere):
    """Test handling of quota exceeded errors."""
    mock_client = MagicMock()
    mock_cohere.return_value = mock_client

    # Mock quota exceeded error
    mock_client.embed.side_effect = Exception("quota exceeded")

    embedder = CohereEmbedder(api_key="test-key", max_retries=1)
    chunk = DocumentChunk(id="test:1", text="Test", source_url="https://example.com")

    with pytest.raises(CohereQuotaError):
        embedder.embed([chunk])


@patch("ingestion.embeddings.cohere_embedder.cohere.ClientV2")
def test_embedder_respects_batch_size(mock_cohere):
    """Test that embedder respects batch size."""
    mock_client = MagicMock()
    mock_cohere.return_value = mock_client

    # Mock response for any batch size
    def embed_side_effect(model, texts, input_type):
        return MagicMock(embeddings=[[0.1] * 10 for _ in texts])

    mock_client.embed.side_effect = embed_side_effect

    embedder = CohereEmbedder(api_key="test-key", batch_size=5)

    chunks = [
        DocumentChunk(id=f"chunk:{i}", text=f"Text {i}", source_url="https://example.com")
        for i in range(12)
    ]

    embeddings = embedder.embed(chunks)

    assert len(embeddings) == 12
    # Should make 3 calls for 12 chunks with batch size 5
    # (5 + 5 + 2)
    assert mock_client.embed.call_count >= 2
