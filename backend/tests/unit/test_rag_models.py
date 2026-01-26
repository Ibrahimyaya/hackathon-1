"""Unit tests for RAG response models."""

import pytest

from rag.models import RAGResponse, RetrievedChunk


class TestRetrievedChunk:
    """Tests for RetrievedChunk model."""

    def test_retrieved_chunk_creation(self):
        """Test creating a RetrievedChunk."""
        chunk = RetrievedChunk(
            text="This is a sample chunk",
            source_url="https://example.com/page",
            section="Getting Started",
            score=0.87,
            chunk_id="chunk_001",
        )

        assert chunk.text == "This is a sample chunk"
        assert chunk.source_url == "https://example.com/page"
        assert chunk.section == "Getting Started"
        assert chunk.score == 0.87
        assert chunk.chunk_id == "chunk_001"

    def test_retrieved_chunk_repr(self):
        """Test RetrievedChunk string representation."""
        chunk = RetrievedChunk(
            text="Sample",
            source_url="https://example.com",
            section="Intro",
            score=0.75,
            chunk_id="chunk_1",
        )

        repr_str = repr(chunk)
        assert "RetrievedChunk" in repr_str
        assert "example.com" in repr_str
        assert "Intro" in repr_str

    def test_retrieved_chunk_low_score(self):
        """Test RetrievedChunk with low score."""
        chunk = RetrievedChunk(
            text="Low relevance chunk",
            source_url="https://example.com",
            section="Other",
            score=0.25,
            chunk_id="chunk_2",
        )

        assert chunk.score == 0.25


class TestRAGResponse:
    """Tests for RAGResponse model."""

    def test_rag_response_creation(self):
        """Test creating a RAGResponse."""
        chunk = RetrievedChunk(
            text="Sample chunk",
            source_url="https://example.com",
            section="Intro",
            score=0.85,
            chunk_id="chunk_1",
        )

        response = RAGResponse(
            answer="This is the answer",
            sources=[chunk],
            confidence=0.85,
            query="What is this?",
            model="claude-3-5-haiku-20241022",
        )

        assert response.answer == "This is the answer"
        assert len(response.sources) == 1
        assert response.confidence == 0.85
        assert response.query == "What is this?"
        assert response.model == "claude-3-5-haiku-20241022"

    def test_rag_response_no_sources(self):
        """Test RAGResponse with no sources."""
        response = RAGResponse(
            answer="No relevant sources found",
            sources=[],
            confidence=0.2,
            query="Obscure question",
            model="claude-3-5-haiku-20241022",
        )

        assert response.sources == []
        assert response.confidence == 0.2

    def test_rag_response_format_cli_output(self):
        """Test CLI output formatting."""
        chunk1 = RetrievedChunk(
            text="Sample chunk 1",
            source_url="https://docs.example.com/page1",
            section="Installation",
            score=0.92,
            chunk_id="chunk_1",
        )

        chunk2 = RetrievedChunk(
            text="Sample chunk 2",
            source_url="https://docs.example.com/page2",
            section="Configuration",
            score=0.85,
            chunk_id="chunk_2",
        )

        response = RAGResponse(
            answer="To install, follow these steps:\n1. Clone the repo\n2. Run setup",
            sources=[chunk1, chunk2],
            confidence=0.87,
            query="How do I install?",
            model="claude-3-5-haiku-20241022",
        )

        output = response.format_cli_output()

        # Verify output contains expected sections
        assert "Confidence" in output
        assert "87%" in output
        assert "To install" in output
        assert "Sources" in output
        assert "Installation" in output
        assert "Configuration" in output
        assert "example.com" in output

    def test_rag_response_format_cli_output_no_sources(self):
        """Test CLI output formatting with no sources."""
        response = RAGResponse(
            answer="Answer without sources",
            sources=[],
            confidence=0.2,
            query="Query",
            model="claude-3-5-haiku-20241022",
        )

        output = response.format_cli_output()

        assert "Answer without sources" in output
        assert "No sources found" in output

    def test_rag_response_high_confidence(self):
        """Test high confidence score formatting."""
        chunk = RetrievedChunk(
            text="Relevant chunk",
            source_url="https://example.com",
            section="Main",
            score=0.98,
            chunk_id="chunk_1",
        )

        response = RAGResponse(
            answer="High confidence answer",
            sources=[chunk],
            confidence=0.98,
            query="Question",
            model="claude-3-5-haiku-20241022",
        )

        output = response.format_cli_output()
        assert "98%" in output

    def test_rag_response_low_confidence(self):
        """Test low confidence score formatting."""
        response = RAGResponse(
            answer="Low confidence answer",
            sources=[],
            confidence=0.15,
            query="Question",
            model="claude-3-5-haiku-20241022",
        )

        output = response.format_cli_output()
        assert "15%" in output
