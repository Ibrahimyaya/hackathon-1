"""Integration tests for RAG chatbot."""

import pytest
from unittest.mock import Mock, MagicMock, patch
import sys
from io import StringIO

# Need to mock rich before importing chat
sys.modules['rich'] = MagicMock()
sys.modules['rich.console'] = MagicMock()
sys.modules['rich.markdown'] = MagicMock()
sys.modules['rich.panel'] = MagicMock()
sys.modules['rich.table'] = MagicMock()


@pytest.fixture
def mock_config():
    """Create a mock config for testing."""
    config = Mock()
    config.cohere_api_key = "test_key"
    config.cohere_model = "embed-english-v3.0"
    config.cohere_batch_size = 100
    config.qdrant_url = "http://localhost:6333"
    config.qdrant_api_key = "test_key"
    config.qdrant_collection_name = "test_collection"
    config.qdrant_vector_size = 1024
    config.claude_api_key = "sk-test"
    config.claude_model = "claude-3-5-haiku-20241022"
    config.claude_max_tokens = 2000
    config.claude_temperature = 0.7
    config.rag_top_k = 5
    config.rag_score_threshold = 0.3
    config.rag_max_history_turns = 5
    config.rag_context_window = 8000
    config.log_level = "INFO"
    config.log_format = "json"
    return config


class TestRAGChatbotCommandHandling:
    """Tests for RAG chatbot command handling."""

    @patch('backend.chat.load_config')
    @patch('backend.chat.setup_logging')
    @patch('backend.chat.CohereEmbedder')
    @patch('backend.chat.QdrantStore')
    @patch('backend.chat.anthropic.Anthropic')
    @patch('backend.chat.RAGQueryEngine')
    def test_chatbot_initialization(
        self,
        mock_query_engine_class,
        mock_anthropic,
        mock_qdrant,
        mock_cohere,
        mock_logging,
        mock_load_config,
        mock_config,
    ):
        """Test that chatbot initializes correctly."""
        mock_load_config.return_value = mock_config

        from backend.chat import RAGChatbot

        chatbot = RAGChatbot()

        # Verify clients were initialized
        assert chatbot.config is not None
        assert chatbot.embedder is not None
        assert chatbot.store is not None
        assert chatbot.claude_client is not None
        assert chatbot.engine is not None

    @patch('backend.chat.load_config')
    @patch('backend.chat.setup_logging')
    @patch('backend.chat.CohereEmbedder')
    @patch('backend.chat.QdrantStore')
    @patch('backend.chat.anthropic.Anthropic')
    @patch('backend.chat.RAGQueryEngine')
    def test_chatbot_command_help(
        self,
        mock_query_engine_class,
        mock_anthropic,
        mock_qdrant,
        mock_cohere,
        mock_logging,
        mock_load_config,
        mock_config,
    ):
        """Test /help command."""
        mock_load_config.return_value = mock_config

        from backend.chat import RAGChatbot

        chatbot = RAGChatbot()
        chatbot._show_help()  # Should not raise

    @patch('backend.chat.load_config')
    @patch('backend.chat.setup_logging')
    @patch('backend.chat.CohereEmbedder')
    @patch('backend.chat.QdrantStore')
    @patch('backend.chat.anthropic.Anthropic')
    @patch('backend.chat.RAGQueryEngine')
    def test_chatbot_clear_history(
        self,
        mock_query_engine_class,
        mock_anthropic,
        mock_qdrant,
        mock_cohere,
        mock_logging,
        mock_load_config,
        mock_config,
    ):
        """Test /clear command."""
        mock_load_config.return_value = mock_config
        mock_engine = MagicMock()
        mock_query_engine_class.return_value = mock_engine

        from backend.chat import RAGChatbot

        chatbot = RAGChatbot()
        chatbot._clear_history()

        # Verify clear_history was called on engine
        mock_engine.clear_history.assert_called_once()

    @patch('backend.chat.load_config')
    @patch('backend.chat.setup_logging')
    @patch('backend.chat.CohereEmbedder')
    @patch('backend.chat.QdrantStore')
    @patch('backend.chat.anthropic.Anthropic')
    @patch('backend.chat.RAGQueryEngine')
    def test_chatbot_show_history(
        self,
        mock_query_engine_class,
        mock_anthropic,
        mock_qdrant,
        mock_cohere,
        mock_logging,
        mock_load_config,
        mock_config,
    ):
        """Test /history command."""
        mock_load_config.return_value = mock_config
        mock_engine = MagicMock()
        mock_engine.get_history.return_value = [
            {"question": "Q1", "answer": "A1"},
            {"question": "Q2", "answer": "A2"},
        ]
        mock_query_engine_class.return_value = mock_engine

        from backend.chat import RAGChatbot

        chatbot = RAGChatbot()
        chatbot._show_history()

        # Verify get_history was called
        mock_engine.get_history.assert_called_once()

    @patch('backend.chat.load_config')
    @patch('backend.chat.setup_logging')
    @patch('backend.chat.CohereEmbedder')
    @patch('backend.chat.QdrantStore')
    @patch('backend.chat.anthropic.Anthropic')
    @patch('backend.chat.RAGQueryEngine')
    def test_chatbot_toggle_sources(
        self,
        mock_query_engine_class,
        mock_anthropic,
        mock_qdrant,
        mock_cohere,
        mock_logging,
        mock_load_config,
        mock_config,
    ):
        """Test /sources command."""
        mock_load_config.return_value = mock_config

        from backend.chat import RAGChatbot

        chatbot = RAGChatbot()

        # Initial state: sources enabled
        assert chatbot.show_sources is True

        chatbot._toggle_sources()
        assert chatbot.show_sources is False

        chatbot._toggle_sources()
        assert chatbot.show_sources is True

    @patch('backend.chat.load_config')
    @patch('backend.chat.setup_logging')
    @patch('backend.chat.CohereEmbedder')
    @patch('backend.chat.QdrantStore')
    @patch('backend.chat.anthropic.Anthropic')
    @patch('backend.chat.RAGQueryEngine')
    def test_chatbot_handle_invalid_command(
        self,
        mock_query_engine_class,
        mock_anthropic,
        mock_qdrant,
        mock_cohere,
        mock_logging,
        mock_load_config,
        mock_config,
    ):
        """Test handling of invalid command."""
        mock_load_config.return_value = mock_config

        from backend.chat import RAGChatbot

        chatbot = RAGChatbot()
        chatbot._handle_command("/invalid")  # Should not raise

    @patch('backend.chat.load_config')
    @patch('backend.chat.setup_logging')
    @patch('backend.chat.CohereEmbedder')
    @patch('backend.chat.QdrantStore')
    @patch('backend.chat.anthropic.Anthropic')
    @patch('backend.chat.RAGQueryEngine')
    def test_chatbot_quit_command(
        self,
        mock_query_engine_class,
        mock_anthropic,
        mock_qdrant,
        mock_cohere,
        mock_logging,
        mock_load_config,
        mock_config,
    ):
        """Test /quit command."""
        mock_load_config.return_value = mock_config

        from backend.chat import RAGChatbot

        chatbot = RAGChatbot()
        assert chatbot.is_running is True

        chatbot._handle_command("/quit")
        assert chatbot.is_running is False


class TestRAGChatbotQueryProcessing:
    """Tests for query processing."""

    @patch('backend.chat.load_config')
    @patch('backend.chat.setup_logging')
    @patch('backend.chat.CohereEmbedder')
    @patch('backend.chat.QdrantStore')
    @patch('backend.chat.anthropic.Anthropic')
    @patch('backend.chat.RAGQueryEngine')
    def test_chatbot_process_query(
        self,
        mock_query_engine_class,
        mock_anthropic,
        mock_qdrant,
        mock_cohere,
        mock_logging,
        mock_load_config,
        mock_config,
    ):
        """Test query processing."""
        mock_load_config.return_value = mock_config
        mock_engine = MagicMock()

        from backend.rag.models import RAGResponse, RetrievedChunk

        chunk = RetrievedChunk(
            text="Response text",
            source_url="https://example.com",
            section="Section",
            score=0.9,
            chunk_id="chunk_1",
        )

        mock_response = RAGResponse(
            answer="The answer is here",
            sources=[chunk],
            confidence=0.9,
            query="Question?",
            model="claude-3-5-haiku-20241022",
        )

        mock_engine.query.return_value = mock_response
        mock_query_engine_class.return_value = mock_engine

        from backend.chat import RAGChatbot

        chatbot = RAGChatbot()
        chatbot._process_query("Question?")

        # Verify engine was called
        mock_engine.query.assert_called_once_with("Question?")

    @patch('backend.chat.load_config')
    @patch('backend.chat.setup_logging')
    @patch('backend.chat.CohereEmbedder')
    @patch('backend.chat.QdrantStore')
    @patch('backend.chat.anthropic.Anthropic')
    @patch('backend.chat.RAGQueryEngine')
    def test_chatbot_handles_api_error(
        self,
        mock_query_engine_class,
        mock_anthropic,
        mock_qdrant,
        mock_cohere,
        mock_logging,
        mock_load_config,
        mock_config,
    ):
        """Test handling of API errors."""
        import anthropic

        mock_load_config.return_value = mock_config
        mock_engine = MagicMock()
        mock_engine.query.side_effect = anthropic.APIError("Test error")
        mock_query_engine_class.return_value = mock_engine

        from backend.chat import RAGChatbot

        chatbot = RAGChatbot()
        chatbot._process_query("Question?")  # Should not raise


class TestRAGChatbotBanner:
    """Tests for banner display."""

    @patch('backend.chat.load_config')
    @patch('backend.chat.setup_logging')
    @patch('backend.chat.CohereEmbedder')
    @patch('backend.chat.QdrantStore')
    @patch('backend.chat.anthropic.Anthropic')
    @patch('backend.chat.RAGQueryEngine')
    def test_print_banner(
        self,
        mock_query_engine_class,
        mock_anthropic,
        mock_qdrant,
        mock_cohere,
        mock_logging,
        mock_load_config,
        mock_config,
    ):
        """Test banner printing."""
        mock_load_config.return_value = mock_config

        from backend.chat import RAGChatbot

        chatbot = RAGChatbot()
        chatbot._print_banner()  # Should not raise
