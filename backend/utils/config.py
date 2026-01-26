"""Configuration management using pydantic for environment variable validation."""

import os
from pathlib import Path
from typing import Optional

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Config(BaseSettings):
    """Application configuration loaded from environment variables."""

    # Crawling Configuration
    docs_url: str = Field(..., description="Target Docusaurus site root URL")
    crawl_max_pages: int = Field(
        default=1000, description="Maximum number of pages to crawl per session"
    )
    crawl_timeout_seconds: int = Field(default=10, description="Timeout per page crawl (seconds)")
    crawl_follow_external_links: bool = Field(
        default=False, description="Follow external links (stay on same domain if False)"
    )
    crawl_delay_ms: int = Field(
        default=500, description="Delay between crawl requests (milliseconds)"
    )

    # Chunking Configuration
    chunk_min_tokens: int = Field(default=256, description="Minimum chunk size (tokens)")
    chunk_max_tokens: int = Field(default=512, description="Maximum chunk size (tokens)")
    chunk_overlap_tokens: int = Field(default=50, description="Overlap between chunks (tokens)")

    # Cohere API Configuration
    cohere_api_key: str = Field(..., description="Cohere API key (required)")
    cohere_model: str = Field(default="embed-english-v3.0", description="Cohere model identifier")
    cohere_batch_size: int = Field(default=100, description="Batch size for embedding requests")

    # Qdrant Configuration
    qdrant_api_key: str = Field(..., description="Qdrant Cloud API key (required)")
    qdrant_url: str = Field(..., description="Qdrant Cloud endpoint (required)")
    qdrant_collection_name: str = Field(default="docs_chunks", description="Collection name")
    qdrant_vector_size: int = Field(
        default=1024, description="Vector dimension (must match Cohere model output)"
    )
    qdrant_recreate_collection: bool = Field(
        default=False, description="Recreate collection on init (dev only - data loss!)"
    )

    # Claude API Configuration
    claude_api_key: str = Field(..., description="Anthropic Claude API key (required)")
    claude_model: str = Field(
        default="claude-3-5-haiku-20241022", description="Claude model ID (haiku, sonnet, opus)"
    )
    claude_max_tokens: int = Field(default=2000, description="Maximum response tokens")
    claude_temperature: float = Field(default=0.7, description="Sampling temperature (0.0-1.0)")

    # RAG Configuration
    rag_top_k: int = Field(default=5, description="Number of chunks to retrieve")
    rag_score_threshold: float = Field(default=0.3, description="Minimum similarity score")
    rag_max_history_turns: int = Field(default=5, description="Maximum conversation history turns")
    rag_context_window: int = Field(default=8000, description="Max tokens for context")

    # Logging & Debugging
    log_level: str = Field(default="INFO", description="Log level (DEBUG, INFO, WARNING, ERROR)")
    log_format: str = Field(
        default="json", description="Log format (json for structured, human for readable)"
    )

    model_config = SettingsConfigDict(
        env_file=[".env", "../.env", "../../.env"],
        case_sensitive=False,
        extra="ignore",
    )

    @field_validator("crawl_max_pages")
    @classmethod
    def validate_max_pages(cls, v):
        """Validate that max_pages is positive."""
        if v <= 0:
            raise ValueError("crawl_max_pages must be greater than 0")
        return v

    @field_validator("chunk_min_tokens", "chunk_max_tokens", "chunk_overlap_tokens")
    @classmethod
    def validate_token_counts(cls, v):
        """Validate that token counts are positive."""
        if v <= 0:
            raise ValueError("Token counts must be greater than 0")
        return v

    @field_validator("log_format")
    @classmethod
    def validate_log_format(cls, v):
        """Validate log format."""
        if v not in ("json", "human"):
            raise ValueError("log_format must be 'json' or 'human'")
        return v

    @field_validator("log_level")
    @classmethod
    def validate_log_level(cls, v):
        """Validate log level."""
        valid_levels = ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")
        if v not in valid_levels:
            raise ValueError(f"log_level must be one of {valid_levels}")
        return v

    @field_validator("claude_temperature")
    @classmethod
    def validate_claude_temperature(cls, v):
        """Validate Claude temperature is between 0 and 1."""
        if not (0.0 <= v <= 1.0):
            raise ValueError("claude_temperature must be between 0.0 and 1.0")
        return v

    @field_validator("claude_max_tokens")
    @classmethod
    def validate_claude_max_tokens(cls, v):
        """Validate Claude max tokens is positive."""
        if v <= 0:
            raise ValueError("claude_max_tokens must be greater than 0")
        return v

    @field_validator("rag_top_k")
    @classmethod
    def validate_rag_top_k(cls, v):
        """Validate RAG top_k is positive."""
        if v <= 0:
            raise ValueError("rag_top_k must be greater than 0")
        return v

    @field_validator("rag_score_threshold")
    @classmethod
    def validate_rag_score_threshold(cls, v):
        """Validate RAG score threshold is between 0 and 1."""
        if not (0.0 <= v <= 1.0):
            raise ValueError("rag_score_threshold must be between 0.0 and 1.0")
        return v

    @field_validator("rag_max_history_turns")
    @classmethod
    def validate_rag_max_history_turns(cls, v):
        """Validate RAG max history turns is positive."""
        if v <= 0:
            raise ValueError("rag_max_history_turns must be greater than 0")
        return v

    @field_validator("rag_context_window")
    @classmethod
    def validate_rag_context_window(cls, v):
        """Validate RAG context window is positive."""
        if v <= 0:
            raise ValueError("rag_context_window must be greater than 0")
        return v


def load_config() -> Config:
    """Load and validate configuration from environment variables.

    Returns:
        Config: Validated configuration object.

    Raises:
        ValueError: If required environment variables are missing or invalid.
    """
    try:
        config = Config()
        return config
    except Exception as e:
        raise ValueError(
            f"Configuration validation failed. Please check your .env file:\n{str(e)}"
        ) from e
