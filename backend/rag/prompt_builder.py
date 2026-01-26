"""Utilities for building RAG prompts."""

from typing import Dict, List

from rag.models import RetrievedChunk


def build_rag_prompt(
    query: str,
    chunks: List[RetrievedChunk],
    history: List[Dict[str, str]],
) -> str:
    """Build prompt with context and conversation history.

    Args:
        query: User's current question.
        chunks: Retrieved documentation chunks.
        history: List of previous Q&A turns, each with 'question' and 'answer' keys.

    Returns:
        Formatted prompt string for Claude.
    """
    # Format retrieved chunks as context
    context_parts = []
    for chunk in chunks:
        section_line = f"[Source: {chunk.source_url}]"
        if chunk.section:
            section_line += f" [Section: {chunk.section}]"
        context_parts.append(f"{section_line}\n{chunk.text}")

    context = "\n\n---\n\n".join(context_parts)

    # Format conversation history
    history_text = ""
    if history:
        history_lines = []
        for turn in history[-5:]:  # Last 5 turns for context
            history_lines.append(f"User: {turn['question']}")
            history_lines.append(f"Assistant: {turn['answer']}")
        history_text = "\n\n".join(history_lines)

    # Build final prompt
    prompt_parts = [
        "You are a helpful AI assistant. Answer the user's question based on the provided documentation context. If the context doesn't contain enough information to answer the question, say so honestly."
    ]

    if context:
        prompt_parts.append(f"\nDocumentation Context:\n{context}")

    if history_text:
        prompt_parts.append(f"\nPrevious Conversation:\n{history_text}")

    prompt_parts.append(f"\nCurrent Question: {query}")

    return "\n".join(prompt_parts)


def calculate_confidence(retrieval_scores: List[float]) -> float:
    """Calculate confidence based on retrieval scores.

    Uses a weighted combination of top score and average score:
    - Top score (70% weight): Best match indicates relevance
    - Average score (30% weight): Overall quality of results

    Args:
        retrieval_scores: List of similarity scores from vector search.

    Returns:
        Confidence score between 0.0 and 1.0.
    """
    if not retrieval_scores:
        return 0.0

    top_score = max(retrieval_scores)
    avg_score = sum(retrieval_scores) / len(retrieval_scores)

    # Weighted combination: prefer high top score but also consider overall quality
    confidence = (top_score * 0.7) + (avg_score * 0.3)
    return min(1.0, confidence)
