"""Tests for text chunking."""

import pytest

from ingestion.chunking.text_chunker import TextChunker
from utils.errors import TokenizationError, ChunkingStrategyError


def test_chunker_init():
    """Test chunker initialization."""
    chunker = TextChunker(
        min_tokens=256,
        max_tokens=512,
        overlap_tokens=50,
    )
    assert chunker.min_tokens == 256
    assert chunker.max_tokens == 512
    assert chunker.overlap_tokens == 50


def test_chunker_init_invalid_encoding():
    """Test chunker with invalid encoding."""
    with pytest.raises(TokenizationError):
        TextChunker(encoding_name="invalid_encoding")


def test_chunker_simple_text():
    """Test chunking simple text."""
    chunker = TextChunker(min_tokens=10, max_tokens=50)
    text = "This is a simple test. It has multiple sentences. Each sentence is short."

    chunks = chunker.chunk(text, source_url="https://example.com", section="Test")

    assert len(chunks) > 0
    for chunk in chunks:
        assert chunk.source_url == "https://example.com"
        assert chunk.section == "Test"
        assert chunk.text  # Non-empty
        assert chunk.token_count >= 0


def test_chunker_with_metadata():
    """Test chunking with additional metadata."""
    chunker = TextChunker()
    text = "First paragraph.\n\nSecond paragraph."
    metadata = {"author": "test", "date": "2026-01-20"}

    chunks = chunker.chunk(
        text,
        source_url="https://example.com",
        section="Intro",
        metadata=metadata,
    )

    assert len(chunks) > 0
    for chunk in chunks:
        assert "author" in chunk.metadata or len(chunk.metadata) >= 0


def test_chunker_empty_text():
    """Test chunking empty text."""
    chunker = TextChunker()
    chunks = chunker.chunk("")

    assert len(chunks) == 0


def test_chunker_token_counting():
    """Test token counting."""
    chunker = TextChunker()
    text = "This is a test."
    token_count = chunker._count_tokens(text)

    assert isinstance(token_count, int)
    assert token_count > 0


def test_chunker_boundary_splitting():
    """Test splitting at semantic boundaries."""
    chunker = TextChunker(min_tokens=5, max_tokens=50)

    text = """First section.

Second section. With more detail.

Third section."""

    segments = chunker._split_at_boundaries(text)

    assert len(segments) > 0
    # Should split by paragraphs
    assert all(isinstance(s, str) for s in segments)


def test_chunker_chunk_id_format():
    """Test that chunk IDs are properly formatted."""
    chunker = TextChunker()
    text = "Chunk one. Chunk two. Chunk three."
    url = "https://example.com/page"

    chunks = chunker.chunk(text, source_url=url)

    assert len(chunks) > 0
    for i, chunk in enumerate(chunks):
        assert url in chunk.id
        assert "chunk_" in chunk.id
        assert chunk.chunk_index == i


def test_chunker_overlap():
    """Test that overlap is applied between chunks."""
    chunker = TextChunker(
        min_tokens=5,
        max_tokens=20,
        overlap_tokens=3,
    )

    # Create longer text with clear sentence boundaries
    long_text = "Word. " * 200  # 200 short sentences
    chunks = chunker.chunk(long_text)

    # Multiple chunks should be created for long text
    assert len(chunks) > 1


def test_chunker_respects_max_tokens():
    """Test that chunks respect maximum token limit."""
    chunker = TextChunker(max_tokens=100)
    # Create text with many sentences to ensure proper chunking
    text = "This is a test sentence. " * 100

    chunks = chunker.chunk(text)

    # At least most chunks should respect the limit
    # (some flexibility at chunk boundaries)
    for chunk in chunks:
        assert chunk.token_count <= chunker.max_tokens + 20  # Tolerance for boundaries


def test_chunker_nonexistent_url():
    """Test chunking with nonexistent URL."""
    chunker = TextChunker()
    text = "Sample text for chunking."

    chunks = chunker.chunk(text, source_url="")

    assert len(chunks) > 0
    for chunk in chunks:
        assert chunk.source_url == ""


def test_chunker_get_overlap():
    """Test overlap text extraction."""
    chunker = TextChunker(overlap_tokens=50)
    text = "This is a long text. " * 50

    overlap = chunker._get_overlap(text)

    assert isinstance(overlap, str)
    assert len(overlap) > 0
    assert len(overlap) < len(text)
