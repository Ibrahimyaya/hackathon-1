"""Tests for Qdrant storage."""

import pytest
from unittest.mock import patch, MagicMock

from ingestion.storage.qdrant_store import QdrantStore
from ingestion.models import Embedding
from utils.errors import QdrantConnectionError, QdrantCollectionError, QdrantQueryError


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_init(mock_client_class):
    """Test store initialization."""
    mock_client = MagicMock()
    mock_client_class.return_value = mock_client

    # Mock collection operations
    mock_client.get_collections.return_value = MagicMock(collections=[])

    store = QdrantStore(
        url="http://localhost:6333",
        api_key="test-key",
        collection_name="test_collection",
        vector_size=1024,
    )

    assert store.collection_name == "test_collection"
    assert store.vector_size == 1024


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_init_connection_error(mock_client_class):
    """Test store initialization with connection error."""
    mock_client_class.side_effect = Exception("Connection failed")

    with pytest.raises(QdrantConnectionError):
        QdrantStore(
            url="http://invalid:6333",
            api_key="test-key",
        )


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_collection_exists(mock_client_class):
    """Test handling when collection already exists."""
    mock_client = MagicMock()
    mock_client_class.return_value = mock_client

    # Mock collection exists
    existing_collection = MagicMock()
    existing_collection.name = "test_collection"
    mock_client.get_collections.return_value = MagicMock(
        collections=[existing_collection]
    )

    store = QdrantStore(
        url="http://localhost:6333",
        api_key="test-key",
        collection_name="test_collection",
    )

    # Should not call create_collection
    mock_client.create_collection.assert_not_called()


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_recreate_collection(mock_client_class):
    """Test recreating an existing collection."""
    mock_client = MagicMock()
    mock_client_class.return_value = mock_client

    # Mock collection exists
    existing_collection = MagicMock()
    existing_collection.name = "test_collection"
    mock_client.get_collections.return_value = MagicMock(
        collections=[existing_collection]
    )

    store = QdrantStore(
        url="http://localhost:6333",
        api_key="test-key",
        collection_name="test_collection",
        recreate_collection=True,
    )

    # Should call delete then create
    mock_client.delete_collection.assert_called_once()
    mock_client.create_collection.assert_called_once()


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_upsert(mock_client_class):
    """Test upserting embeddings."""
    mock_client = MagicMock()
    mock_client_class.return_value = mock_client

    # Mock collection operations
    mock_client.get_collections.return_value = MagicMock(collections=[])

    store = QdrantStore(
        url="http://localhost:6333",
        api_key="test-key",
    )

    embeddings = [
        Embedding(chunk_id="chunk:1", vector=[0.1] * 1024),
        Embedding(chunk_id="chunk:2", vector=[0.2] * 1024),
    ]

    count = store.upsert(embeddings)

    assert count == 2
    mock_client.upsert.assert_called_once()


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_upsert_empty_list(mock_client_class):
    """Test upserting empty list."""
    mock_client = MagicMock()
    mock_client_class.return_value = mock_client

    # Mock collection operations
    mock_client.get_collections.return_value = MagicMock(collections=[])

    store = QdrantStore(
        url="http://localhost:6333",
        api_key="test-key",
    )

    count = store.upsert([])

    assert count == 0
    mock_client.upsert.assert_not_called()


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_upsert_error(mock_client_class):
    """Test handling of upsert errors."""
    mock_client = MagicMock()
    mock_client_class.return_value = mock_client

    # Mock collection operations
    mock_client.get_collections.return_value = MagicMock(collections=[])
    mock_client.upsert.side_effect = Exception("Upsert failed")

    store = QdrantStore(
        url="http://localhost:6333",
        api_key="test-key",
    )

    embeddings = [Embedding(chunk_id="chunk:1", vector=[0.1] * 1024)]

    with pytest.raises(QdrantCollectionError):
        store.upsert(embeddings)


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_search(mock_client_class):
    """Test searching for similar vectors."""
    mock_client = MagicMock()
    mock_client_class.return_value = mock_client

    # Mock collection and search operations
    mock_client.get_collections.return_value = MagicMock(collections=[])

    # Mock search results
    result1 = MagicMock()
    result1.score = 0.95
    result1.payload = {"chunk_id": "chunk:1", "text": "Text 1"}

    result2 = MagicMock()
    result2.score = 0.87
    result2.payload = {"chunk_id": "chunk:2", "text": "Text 2"}

    mock_client.search.return_value = [result1, result2]

    store = QdrantStore(
        url="http://localhost:6333",
        api_key="test-key",
    )

    results = store.search([0.1] * 1024, limit=2)

    assert len(results) == 2
    assert results[0][0] == "chunk:1"  # chunk_id
    assert results[0][1] == 0.95  # score
    assert results[1][0] == "chunk:2"


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_search_error(mock_client_class):
    """Test handling of search errors."""
    mock_client = MagicMock()
    mock_client_class.return_value = mock_client

    # Mock collection operations
    mock_client.get_collections.return_value = MagicMock(collections=[])
    mock_client.search.side_effect = Exception("Search failed")

    store = QdrantStore(
        url="http://localhost:6333",
        api_key="test-key",
    )

    with pytest.raises(QdrantQueryError):
        store.search([0.1] * 1024)


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_get_collection_info(mock_client_class):
    """Test getting collection information."""
    mock_client = MagicMock()
    mock_client_class.return_value = mock_client

    # Mock collection operations
    mock_client.get_collections.return_value = MagicMock(collections=[])

    # Mock collection info
    mock_info = MagicMock()
    mock_info.name = "test_collection"
    mock_info.vectors_count = 1000
    mock_info.points_count = 1000
    mock_client.get_collection.return_value = mock_info

    store = QdrantStore(
        url="http://localhost:6333",
        api_key="test-key",
        collection_name="test_collection",
    )

    info = store.get_collection_info()

    assert info["name"] == "test_collection"
    assert info["vectors_count"] == 1000
    assert info["points_count"] == 1000
    assert info["vector_size"] == 1024


@patch("ingestion.storage.qdrant_store.QdrantClient")
def test_store_get_collection_info_error(mock_client_class):
    """Test handling of collection info errors."""
    mock_client = MagicMock()
    mock_client_class.return_value = mock_client

    # Mock collection operations
    mock_client.get_collections.return_value = MagicMock(collections=[])
    mock_client.get_collection.side_effect = Exception("Info failed")

    store = QdrantStore(
        url="http://localhost:6333",
        api_key="test-key",
    )

    with pytest.raises(QdrantConnectionError):
        store.get_collection_info()
