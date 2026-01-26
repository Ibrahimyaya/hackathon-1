"""
Pydantic models for chat API requests and responses.
"""

from pydantic import BaseModel, Field
from typing import list


class Message(BaseModel):
    """A single message in the conversation history."""

    role: str = Field(..., description="'user' or 'assistant'")
    content: str = Field(..., description="Message content")


class ChatRequest(BaseModel):
    """Request body for the /api/chat endpoint."""

    query: str = Field(..., min_length=1, description="User question about documentation")
    session_id: str = Field(default="default", description="Session identifier for tracking")
    history: list[Message] = Field(default_factory=list, description="Conversation history")


class Source(BaseModel):
    """A source document referenced in the response."""

    text: str = Field(..., description="Excerpt from the source")
    url: str = Field(..., description="URL to the documentation page")
    section: str = Field(default="", description="Section/heading in the documentation")


class ChatResponse(BaseModel):
    """Response body for the /api/chat endpoint."""

    answer: str = Field(..., description="AI-generated response to the query")
    sources: list[Source] = Field(default_factory=list, description="Source citations")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Confidence score (0.0-1.0)")
    query: str = Field(..., description="The original query")


class HealthResponse(BaseModel):
    """Response body for the /health endpoint."""

    status: str = Field(..., description="'ok' if service is healthy")
    version: str = Field(..., description="API version")
