"""Unit tests for RAG prompt builder."""

import pytest

from rag.models import RetrievedChunk
from rag.prompt_builder import build_rag_prompt, calculate_confidence


class TestCalculateConfidence:
    """Tests for confidence score calculation."""

    def test_empty_scores(self):
        """Test with empty retrieval scores."""
        confidence = calculate_confidence([])
        assert confidence == 0.0

    def test_single_perfect_score(self):
        """Test with single perfect score."""
        confidence = calculate_confidence([1.0])
        assert confidence == 1.0

    def test_single_low_score(self):
        """Test with single low score."""
        confidence = calculate_confidence([0.3])
        assert 0.25 <= confidence <= 0.35  # Around 0.3

    def test_multiple_high_scores(self):
        """Test with multiple high scores."""
        confidence = calculate_confidence([0.95, 0.92, 0.89])
        assert 0.9 <= confidence <= 1.0

    def test_multiple_mixed_scores(self):
        """Test with mixed scores (weighted average)."""
        # Top score: 0.8 (70% weight), avg: 0.5 (30% weight)
        # Result: 0.8 * 0.7 + 0.5 * 0.3 = 0.56 + 0.15 = 0.71
        confidence = calculate_confidence([0.8, 0.4, 0.4])
        assert 0.70 <= confidence <= 0.72

    def test_confidence_capped_at_one(self):
        """Test that confidence doesn't exceed 1.0."""
        confidence = calculate_confidence([1.0, 1.0, 1.0])
        assert confidence <= 1.0

    def test_confidence_weighted_formula(self):
        """Test the weighted formula: 0.7 * top + 0.3 * avg."""
        scores = [0.9, 0.7, 0.5]
        confidence = calculate_confidence(scores)
        # top = 0.9, avg = 0.7
        # expected = 0.9 * 0.7 + 0.7 * 0.3 = 0.63 + 0.21 = 0.84
        assert 0.83 <= confidence <= 0.85


class TestBuildRAGPrompt:
    """Tests for RAG prompt building."""

    def test_prompt_with_single_chunk(self):
        """Test prompt building with a single chunk."""
        chunk = RetrievedChunk(
            text="Installation steps go here",
            source_url="https://example.com/install",
            section="Installation",
            score=0.9,
            chunk_id="chunk_1",
        )

        prompt = build_rag_prompt("How do I install?", [chunk], [])

        # Verify prompt structure
        assert "How do I install?" in prompt
        assert "Installation steps go here" in prompt
        assert "https://example.com/install" in prompt
        assert "helpful AI assistant" in prompt

    def test_prompt_with_multiple_chunks(self):
        """Test prompt building with multiple chunks."""
        chunk1 = RetrievedChunk(
            text="Step 1: Clone repo",
            source_url="https://example.com/step1",
            section="Intro",
            score=0.9,
            chunk_id="chunk_1",
        )

        chunk2 = RetrievedChunk(
            text="Step 2: Install dependencies",
            source_url="https://example.com/step2",
            section="Setup",
            score=0.85,
            chunk_id="chunk_2",
        )

        prompt = build_rag_prompt("How do I install?", [chunk1, chunk2], [])

        assert "Step 1: Clone repo" in prompt
        assert "Step 2: Install dependencies" in prompt
        assert "example.com/step1" in prompt
        assert "example.com/step2" in prompt

    def test_prompt_with_conversation_history(self):
        """Test prompt includes conversation history."""
        chunk = RetrievedChunk(
            text="Answer to first question",
            source_url="https://example.com",
            section="General",
            score=0.8,
            chunk_id="chunk_1",
        )

        history = [
            {"question": "What is X?", "answer": "X is a thing"},
            {"question": "How does Y work?", "answer": "Y works like this"},
        ]

        prompt = build_rag_prompt("Tell me more about Z", [chunk], history)

        # Verify history is included
        assert "What is X?" in prompt
        assert "X is a thing" in prompt
        assert "How does Y work?" in prompt
        assert "Tell me more about Z" in prompt

    def test_prompt_with_no_chunks(self):
        """Test prompt building with no chunks."""
        prompt = build_rag_prompt("Question?", [], [])

        assert "Question?" in prompt
        assert "helpful AI assistant" in prompt
        # No context since no chunks

    def test_prompt_with_empty_section(self):
        """Test chunk with empty section name."""
        chunk = RetrievedChunk(
            text="Content",
            source_url="https://example.com",
            section="",
            score=0.8,
            chunk_id="chunk_1",
        )

        prompt = build_rag_prompt("Question?", [chunk], [])

        assert "Content" in prompt
        assert "https://example.com" in prompt

    def test_prompt_history_limit(self):
        """Test that only last 5 history turns are included."""
        chunk = RetrievedChunk(
            text="Content",
            source_url="https://example.com",
            section="General",
            score=0.8,
            chunk_id="chunk_1",
        )

        # Create history with 7 turns
        history = [
            {"question": f"Q{i}", "answer": f"A{i}"}
            for i in range(1, 8)
        ]

        prompt = build_rag_prompt("Current question", [chunk], history)

        # Should include last 5 turns (Q3-Q7)
        assert "Q7" in prompt  # Most recent
        assert "Q3" in prompt  # 5th from end
        assert "Q2" not in prompt  # More than 5 turns back
        assert "Q1" not in prompt  # More than 5 turns back

    def test_prompt_format_instructions_present(self):
        """Test that prompt includes helpful instructions."""
        chunk = RetrievedChunk(
            text="Documentation content",
            source_url="https://example.com",
            section="Docs",
            score=0.8,
            chunk_id="chunk_1",
        )

        prompt = build_rag_prompt("Question?", [chunk], [])

        # Should instruct to say if not enough info
        assert "honestly" in prompt.lower() or "don't know" in prompt.lower()
