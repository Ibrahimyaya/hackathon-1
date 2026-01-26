"""Unit tests for RAG query engine."""

import pytest
from unittest.mock import Mock, MagicMock, patch

from rag.models import RAGResponse, RetrievedChunk
from rag.query_engine import RAGQueryEngine
from utils.config import Config


@pytest.fixture
def mock_config():
    """Create a mock config."""
    config = Mock(spec=Config)
    config.claude_model = "claude-3-5-haiku-20241022"
    config.claude_max_tokens = 2000
    config.claude_temperature = 0.7
    config.rag_top_k = 5
    config.rag_score_threshold = 0.3
    config.rag_max_history_turns = 5
    return config


@pytest.fixture
def mock_embedder():
    """Create a mock embedder."""
    embedder = Mock()
    embedder.embed_query = Mock(return_value=[0.1, 0.2, 0.3])  # Dummy vector
    return embedder


@pytest.fixture
def mock_store():
    """Create a mock vector store."""
    store = Mock()
    # Return dummy search results
    store.search = Mock(return_value=[
        ("chunk_1", 0.9, {"text": "Result 1", "source_url": "url1", "section": "S1"}),
        ("chunk_2", 0.8, {"text": "Result 2", "source_url": "url2", "section": "S2"}),
    ])
    return store


@pytest.fixture
def mock_claude_client():
    """Create a mock Claude client."""
    client = Mock()
    # Mock a response
    response = Mock()
    response.content = [Mock(text="Sample response")]
    client.messages.create = Mock(return_value=response)
    return client


@pytest.fixture
def query_engine(mock_config, mock_embedder, mock_store, mock_claude_client):
    """Create a RAGQueryEngine with mocked dependencies."""
    return RAGQueryEngine(
        config=mock_config,
        embedder=mock_embedder,
        store=mock_store,
        claude_client=mock_claude_client,
    )


class TestRAGQueryEngineInitialization:
    """Tests for RAGQueryEngine initialization."""

    def test_engine_initialization(self, query_engine):
        """Test that engine initializes correctly."""
        assert query_engine.conversation_history == []
        assert query_engine.config is not None
        assert query_engine.embedder is not None
        assert query_engine.store is not None
        assert query_engine.claude_client is not None


class TestRAGQueryEngineConversationHistory:
    """Tests for conversation history management."""

    def test_add_to_history(self, query_engine):
        """Test adding Q&A to history."""
        query_engine.add_to_history("Question 1", "Answer 1")

        assert len(query_engine.conversation_history) == 1
        assert query_engine.conversation_history[0]["question"] == "Question 1"
        assert query_engine.conversation_history[0]["answer"] == "Answer 1"

    def test_history_limit(self, query_engine, mock_config):
        """Test that history respects max turns limit."""
        mock_config.rag_max_history_turns = 3

        # Add 5 turns
        for i in range(5):
            query_engine.add_to_history(f"Q{i}", f"A{i}")

        # Should only keep last 3
        assert len(query_engine.conversation_history) == 3
        assert query_engine.conversation_history[0]["question"] == "Q2"
        assert query_engine.conversation_history[-1]["question"] == "Q4"

    def test_clear_history(self, query_engine):
        """Test clearing conversation history."""
        query_engine.add_to_history("Q1", "A1")
        query_engine.add_to_history("Q2", "A2")

        assert len(query_engine.conversation_history) == 2

        query_engine.clear_history()

        assert len(query_engine.conversation_history) == 0

    def test_get_history(self, query_engine):
        """Test getting history (should return copy)."""
        query_engine.add_to_history("Q1", "A1")
        query_engine.add_to_history("Q2", "A2")

        history = query_engine.get_history()

        assert len(history) == 2
        assert history is not query_engine.conversation_history  # Should be a copy


class TestRAGQueryEngineRetrieval:
    """Tests for chunk retrieval."""

    def test_retrieve_chunks(self, query_engine, mock_store):
        """Test that _retrieve_chunks calls store correctly."""
        query_vector = [0.1, 0.2, 0.3]

        chunks = query_engine._retrieve_chunks(query_vector, top_k=5)

        # Verify store was called correctly
        mock_store.search.assert_called_once()
        call_kwargs = mock_store.search.call_args[1]
        assert call_kwargs["limit"] == 5

        # Verify chunks were created
        assert len(chunks) == 2
        assert isinstance(chunks[0], RetrievedChunk)
        assert chunks[0].text == "Result 1"
        assert chunks[0].score == 0.9

    def test_retrieve_chunks_with_threshold(self, query_engine, mock_store, mock_config):
        """Test that score threshold is passed to store."""
        mock_config.rag_score_threshold = 0.5
        query_vector = [0.1, 0.2, 0.3]

        query_engine._retrieve_chunks(query_vector, top_k=5)

        # Verify threshold was passed
        call_kwargs = mock_store.search.call_args[1]
        assert call_kwargs["score_threshold"] == 0.5


class TestRAGQueryEngineEmbedding:
    """Tests for query embedding."""

    def test_embed_query(self, query_engine, mock_embedder):
        """Test that _embed_query calls embedder correctly."""
        query = "What is this?"

        vector = query_engine._embed_query(query)

        mock_embedder.embed_query.assert_called_once_with(query)
        assert vector == [0.1, 0.2, 0.3]


class TestRAGQueryEngineResponse:
    """Tests for response generation."""

    def test_generate_response(self, query_engine, mock_claude_client):
        """Test that _generate_response calls Claude correctly."""
        prompt = "Test prompt"

        response_text = query_engine._generate_response(prompt)

        mock_claude_client.messages.create.assert_called_once()
        call_kwargs = mock_claude_client.messages.create.call_args[1]

        # Verify parameters
        assert call_kwargs["max_tokens"] == 2000
        assert call_kwargs["temperature"] == 0.7
        assert call_kwargs["messages"][0]["content"] == prompt

    def test_generate_response_extracts_text(self, query_engine, mock_claude_client):
        """Test that response extracts text from Claude response."""
        response_text = query_engine._generate_response("prompt")

        assert response_text == "Sample response"


class TestRAGQueryEngineFullFlow:
    """Tests for full query processing flow."""

    def test_query_method_integration(self, query_engine, mock_embedder, mock_store, mock_claude_client):
        """Test the full query() method flow."""
        # This is a simplified integration test
        query_engine.query("What is Python?", top_k=5)

        # Verify all steps were called
        mock_embedder.embed_query.assert_called_once()
        mock_store.search.assert_called_once()
        mock_claude_client.messages.create.assert_called_once()

        # Verify history was updated
        assert len(query_engine.conversation_history) == 1

    def test_query_returns_rag_response(self, query_engine):
        """Test that query() returns a RAGResponse."""
        response = query_engine.query("Question?")

        assert isinstance(response, RAGResponse)
        assert response.query == "Question?"
        assert response.model == "claude-3-5-haiku-20241022"
        assert 0.0 <= response.confidence <= 1.0

    def test_query_custom_top_k(self, query_engine, mock_store):
        """Test that query respects custom top_k."""
        query_engine.query("Question?", top_k=10)

        # Verify custom top_k was used
        call_kwargs = mock_store.search.call_args[1]
        assert call_kwargs["limit"] == 10
