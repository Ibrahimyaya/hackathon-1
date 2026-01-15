"""
Configuration loading and validation for RAG ingestion pipeline.

Uses pydantic for type-safe environment variable validation with clear error messages.
"""

from pydantic import BaseModel, Field, field_validator, ValidationError
from pydantic_settings import BaseSettings
import os
from typing import Optional


class Config(BaseSettings):
    """Application configuration loaded from environment variables."""

    # Crawling Configuration
    docs_url: str = Field(
        ...,
        description="Target Docusaurus documentation site root URL",
        json_schema_extra={"example": "https://docs.docusaurus.io"}
    )
    crawl_max_pages: int = Field(
        default=1000,
        ge=1,
        le=100000,
        description="Maximum number of pages to crawl per session"
    )
    crawl_timeout_seconds: int = Field(
        default=10,
        ge=1,
        le=300,
        description="Per-page HTTP request timeout in seconds"
    )
    crawl_follow_external_links: bool = Field(
        default=False,
        description="Whether to follow external links (only same-domain if False)"
    )

    # Chunking Configuration
    chunk_min_tokens: int = Field(
        default=256,
        ge=50,
        le=512,
        description="Minimum chunk size in tokens"
    )
    chunk_max_tokens: int = Field(
        default=512,
        ge=128,
        le=2048,
        description="Maximum chunk size in tokens"
    )
    chunk_overlap_tokens: int = Field(
        default=50,
        ge=0,
        le=256,
        description="Token overlap between chunks for context preservation"
    )

    # Cohere API Configuration
    cohere_api_key: str = Field(
        ...,
        description="Cohere API key for embeddings (required)"
    )
    cohere_model: str = Field(
        default="embed-english-v3.0",
        description="Cohere embedding model identifier"
    )
    cohere_batch_size: int = Field(
        default=100,
        ge=1,
        le=100,
        description="Batch size for Cohere embedding requests"
    )

    # Qdrant Configuration
    qdrant_api_key: str = Field(
        ...,
        description="Qdrant Cloud API key (required)"
    )
    qdrant_url: str = Field(
        ...,
        description="Qdrant Cloud URL endpoint",
        json_schema_extra={"example": "https://abcd1234.qdrant.io:6333"}
    )
    qdrant_collection_name: str = Field(
        default="docs_chunks",
        description="Collection name for storing documentation embeddings"
    )
    qdrant_vector_size: int = Field(
        default=1024,
        description="Vector dimension size (must match embedding model output)"
    )
    qdrant_recreate_collection: bool = Field(
        default=False,
        description="Recreate collection if it exists (dev/testing only)"
    )

    # Logging & Debugging
    log_level: str = Field(
        default="INFO",
        description="Logging level: DEBUG, INFO, WARNING, ERROR, CRITICAL"
    )
    log_format: str = Field(
        default="json",
        description="Logging format: json for structured, human for readable"
    )
    debug: bool = Field(
        default=False,
        description="Enable verbose debug logging"
    )

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False

    @field_validator("log_level")
    @classmethod
    def validate_log_level(cls, v: str) -> str:
        """Validate log level is one of the standard Python logging levels."""
        valid_levels = {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"}
        if v.upper() not in valid_levels:
            raise ValueError(f"log_level must be one of {valid_levels}")
        return v.upper()

    @field_validator("log_format")
    @classmethod
    def validate_log_format(cls, v: str) -> str:
        """Validate log format is either json or human."""
        valid_formats = {"json", "human"}
        if v.lower() not in valid_formats:
            raise ValueError(f"log_format must be one of {valid_formats}")
        return v.lower()

    @field_validator("cohere_model")
    @classmethod
    def validate_cohere_model(cls, v: str) -> str:
        """Validate Cohere model is a known embedding model."""
        if not v or len(v) < 3:
            raise ValueError("cohere_model must be a valid model name")
        return v

    @field_validator("chunk_min_tokens", mode="after")
    def validate_chunk_sizes(self) -> int:
        """Ensure min tokens < max tokens."""
        if self.chunk_min_tokens >= self.chunk_max_tokens:
            raise ValueError(
                f"chunk_min_tokens ({self.chunk_min_tokens}) must be less than "
                f"chunk_max_tokens ({self.chunk_max_tokens})"
            )
        return self.chunk_min_tokens


def load_config() -> Config:
    """
    Load and validate configuration from environment variables.

    Returns:
        Config: Validated configuration object

    Raises:
        ValueError: If required environment variables are missing or invalid
    """
    try:
        return Config()
    except ValidationError as e:
        # Format pydantic validation errors with helpful context
        error_lines = ["Configuration validation failed:"]
        for error in e.errors():
            field = error["loc"][0]
            msg = error["msg"]
            error_lines.append(f"  {field}: {msg}")

        error_lines.append("\nRequired environment variables:")
        error_lines.append("  - DOCS_URL")
        error_lines.append("  - COHERE_API_KEY")
        error_lines.append("  - QDRANT_API_KEY")
        error_lines.append("  - QDRANT_URL")
        error_lines.append("\nSee .env.example for all available configuration options.")

        raise ValueError("\n".join(error_lines)) from e


if __name__ == "__main__":
    # For testing configuration loading
    try:
        config = load_config()
        print("✓ Configuration loaded successfully")
        print(f"  Docs URL: {config.docs_url}")
        print(f"  Cohere Model: {config.cohere_model}")
        print(f"  Qdrant Collection: {config.qdrant_collection_name}")
        print(f"  Log Level: {config.log_level}")
    except ValueError as e:
        print(f"✗ Configuration error:\n{e}")
        exit(1)
