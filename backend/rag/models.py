"""Data models for RAG query responses."""

from dataclasses import dataclass
from typing import List


@dataclass
class RetrievedChunk:
    """A chunk retrieved from the vector database."""

    text: str
    source_url: str
    section: str
    score: float
    chunk_id: str

    def __repr__(self) -> str:
        """String representation for CLI display."""
        return f"RetrievedChunk(url={self.source_url}, section={self.section}, score={self.score:.3f})"


@dataclass
class RAGResponse:
    """Response from RAG query engine."""

    answer: str
    sources: List[RetrievedChunk]
    confidence: float  # 0.0-1.0
    query: str
    model: str  # Claude model used

    def format_cli_output(self) -> str:
        """Format response for CLI display with sources and confidence."""
        lines = []

        # Add confidence indicator
        confidence_pct = int(self.confidence * 100)
        confidence_bar = "█" * (confidence_pct // 10) + "░" * (10 - confidence_pct // 10)
        lines.append(f"[Confidence: {confidence_bar} {confidence_pct}%]")
        lines.append("")

        # Add answer
        lines.append(self.answer)
        lines.append("")

        # Add sources
        if self.sources:
            lines.append("📚 Sources:")
            for chunk in self.sources:
                score_pct = int(chunk.score * 100)
                lines.append(f"  • {chunk.section} (score: {score_pct}%)")
                lines.append(f"    {chunk.source_url}")
        else:
            lines.append("No sources found for this query.")

        return "\n".join(lines)
