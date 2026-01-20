"""Tests for HTML text extraction and cleaning."""

import pytest

from ingestion.processors.text_cleaner import TextCleaner
from utils.errors import ExtractionError


@pytest.fixture
def cleaner():
    """Provide a TextCleaner instance."""
    return TextCleaner()


def test_extract_text_simple_html(cleaner, sample_html):
    """Test text extraction from simple HTML."""
    text, metadata = cleaner.extract_text(sample_html, url="https://example.com/page")

    assert "Main Heading" in text
    assert "This is test content" in text
    assert "Subheading" in text
    assert metadata["url"] == "https://example.com/page"
    assert metadata["character_count"] > 0


def test_extract_text_removes_nav_footer(cleaner):
    """Test that nav and footer are removed."""
    html = """
    <html>
    <body>
    <nav>Navigation</nav>
    <main>Main Content</main>
    <footer>Footer</footer>
    </body>
    </html>
    """

    text, metadata = cleaner.extract_text(html)

    assert "Navigation" not in text
    assert "Footer" not in text
    assert "Main Content" in text


def test_extract_text_preserves_structure(cleaner):
    """Test that text structure is preserved."""
    html = """
    <html>
    <body>
    <h1>Title</h1>
    <p>Paragraph 1</p>
    <h2>Section</h2>
    <p>Paragraph 2</p>
    </body>
    </html>
    """

    text, metadata = cleaner.extract_text(html)

    # Should have multiple lines due to block elements
    lines = text.strip().split("\n")
    assert len(lines) > 1


def test_clean_text_multiple_spaces(cleaner):
    """Test that multiple spaces are normalized."""
    text = "Multiple    spaces    here"
    cleaned = cleaner._clean_text(text)
    assert "    " not in cleaned
    assert "Multiple spaces here" == cleaned


def test_clean_text_multiple_newlines(cleaner):
    """Test that multiple newlines are normalized."""
    text = "Line 1\n\n\n\nLine 2"
    cleaned = cleaner._clean_text(text)
    assert cleaned == "Line 1\nLine 2"


def test_extract_metadata_title(cleaner):
    """Test that page title is extracted."""
    html = """
    <html>
    <head><title>Page Title</title></head>
    <body><main>Content</main></body>
    </html>
    """

    text, metadata = cleaner.extract_text(html)
    assert metadata["title"] == "Page Title"


def test_extract_metadata_description(cleaner):
    """Test that meta description is extracted."""
    html = """
    <html>
    <head>
        <meta name="description" content="Page description">
    </head>
    <body><main>Content</main></body>
    </html>
    """

    text, metadata = cleaner.extract_text(html)
    assert metadata["description"] == "Page description"


def test_extract_metadata_headings(cleaner):
    """Test that heading hierarchy is extracted."""
    html = """
    <html>
    <body>
    <main>
        <h1>Main Title</h1>
        <h2>Section 1</h2>
        <h3>Subsection</h3>
    </main>
    </body>
    </html>
    """

    text, metadata = cleaner.extract_text(html)
    assert len(metadata["headings"]) > 0
    assert any(h["text"] == "Main Title" for h in metadata["headings"])


def test_extract_text_empty_html(cleaner):
    """Test extraction from minimal HTML."""
    html = "<html><body></body></html>"
    text, metadata = cleaner.extract_text(html)
    assert metadata["character_count"] >= 0


def test_extract_text_with_code(cleaner):
    """Test that code blocks are preserved."""
    html = """
    <html>
    <body>
    <main>
        <p>Example:</p>
        <code>def hello(): pass</code>
    </main>
    </body>
    </html>
    """

    text, metadata = cleaner.extract_text(html)
    assert "def hello" in text
    assert "pass" in text
