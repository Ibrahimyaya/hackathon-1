# RAG Documentation Ingestion Pipeline

A Python-based pipeline for ingesting documentation from Docusaurus sites, chunking content, generating embeddings using Cohere, and storing them in Qdrant Cloud for vector similarity search.

## Overview

This system enables developers to:
1. **Crawl** Docusaurus documentation sites and extract clean text
2. **Chunk** text into semantically meaningful segments
3. **Embed** chunks using Cohere's embedding models
4. **Store** embeddings in Qdrant for efficient similarity search

## Quick Start

### Prerequisites

- Python 3.11+
- `uv` package manager (https://astral.sh/uv)
- Cohere API key (free tier available at https://cohere.com/)
- Qdrant Cloud account (free tier available at https://qdrant.io/)

### Installation

1. **Clone and navigate to backend directory**:
   ```bash
   cd backend
   ```

2. **Create environment file**:
   ```bash
   cp .env.example .env
   ```

3. **Fill in required credentials** in `.env`:
   ```bash
   COHERE_API_KEY=your_key_here
   QDRANT_API_KEY=your_key_here
   QDRANT_URL=https://your-cluster.qdrant.io:6333
   DOCS_URL=https://docs.your-site.com
   ```

4. **Install dependencies**:
   ```bash
   uv sync
   ```

   Or install with dev dependencies for testing:
   ```bash
   uv sync --all-extras
   ```

### Running the Pipeline

#### Full End-to-End Pipeline

```bash
python main.py --full-pipeline
```

This runs all stages: crawl → extract → chunk → embed → store

#### Individual Stages

```bash
# Crawl and extract documentation
python main.py --crawl-extract

# Chunk and generate embeddings (requires extracted text)
python main.py --chunk-embed

# Ingest into vector database (requires embeddings)
python main.py --ingest

# Test query execution (requires stored embeddings)
python main.py --test-queries
```

#### Command-Line Help

```bash
python main.py --help
```

## Project Structure

```
backend/
├── pyproject.toml              # Project config and dependencies
├── .env.example                # Configuration template
├── .gitignore                  # Git ignore patterns
├── README.md                   # This file
├── main.py                     # CLI entry point and pipeline orchestration
│
├── ingestion/                  # Core ingestion package
│   ├── __init__.py
│   ├── models.py               # DocumentChunk, Embedding dataclasses
│   ├── crawlers/
│   │   ├── __init__.py
│   │   └── docusaurus_crawler.py    # Docusaurus site crawler
│   │
│   ├── processors/
│   │   ├── __init__.py
│   │   └── text_cleaner.py          # HTML to text extraction
│   │
│   ├── chunking/
│   │   ├── __init__.py
│   │   └── text_chunker.py          # Semantic text chunking
│   │
│   ├── embeddings/
│   │   ├── __init__.py
│   │   └── cohere_embedder.py       # Cohere API client
│   │
│   └── storage/
│       ├── __init__.py
│       └── qdrant_store.py          # Qdrant indexing & search
│
├── utils/
│   ├── __init__.py
│   ├── config.py               # Configuration loading
│   ├── logging.py              # Structured logging
│   └── errors.py               # Exception hierarchy
│
└── tests/
    ├── conftest.py             # pytest fixtures
    ├── unit/                   # Unit tests
    │   ├── test_config.py
    │   ├── test_errors.py
    │   ├── test_docusaurus_crawler.py
    │   ├── test_text_cleaner.py
    │   ├── test_text_chunker.py
    │   ├── test_cohere_embedder.py
    │   └── test_qdrant_store.py
    │
    └── integration/            # Integration tests
        ├── test_crawl_to_extract.py
        ├── test_chunk_embed_integration.py
        ├── test_qdrant_integration.py
        ├── test_full_pipeline.py
        └── test_queries.py
```

## Configuration

All configuration is managed via environment variables in `.env`. See `.env.example` for a complete list of variables with descriptions.

**Key Configuration Variables:**

| Variable | Purpose | Default | Required |
|----------|---------|---------|----------|
| `DOCS_URL` | Target Docusaurus site | - | ✅ Yes |
| `COHERE_API_KEY` | Cohere API authentication | - | ✅ Yes |
| `QDRANT_API_KEY` | Qdrant authentication | - | ✅ Yes |
| `QDRANT_URL` | Qdrant Cloud endpoint | - | ✅ Yes |
| `CRAWL_MAX_PAGES` | Max pages to crawl | 1000 | No |
| `CHUNK_MAX_TOKENS` | Max tokens per chunk | 512 | No |
| `LOG_LEVEL` | Logging verbosity | INFO | No |

## Error Handling

The pipeline handles errors gracefully:

- **Crawl errors**: Retries transient errors (3x with backoff); skips permanent failures
- **Processing errors**: Logs and skips unparseable content
- **Embedding errors**: Handles rate limits and quota exceeded with clear messaging
- **Storage errors**: Validates Qdrant connectivity; creates collection if needed

All errors are logged with full context (URL, chunk ID, stage) for debugging.

## Testing

### Run All Tests

```bash
pytest
```

### Run Specific Test Category

```bash
# Unit tests only
pytest tests/unit/

# Integration tests only
pytest tests/integration/

# Specific test file
pytest tests/unit/test_config.py

# With coverage
pytest --cov=ingestion --cov=utils
```

### Test Requirements

- Unit tests use mocked APIs (no real credentials needed)
- Integration tests require valid Cohere API key and Qdrant Cloud access
- Test fixtures are defined in `tests/conftest.py`

## Logging

The pipeline uses structured JSON logging for production and human-readable logging for development.

**Toggle logging format via environment variable:**

```bash
LOG_FORMAT=json    # Structured JSON (production)
LOG_FORMAT=human   # Human-readable (development)
```

**Set logging level:**

```bash
LOG_LEVEL=DEBUG    # Verbose debug output
LOG_LEVEL=INFO     # Standard info (default)
LOG_LEVEL=WARNING  # Warnings and errors only
```

## Supported Documentation Sites

The crawler is designed for Docusaurus documentation sites. It:

- Discovers pages via `sitemap.xml` (if available) or breadth-first crawl
- Extracts content from standard Docusaurus HTML structure
- Preserves semantic structure (headings, code blocks, lists)
- Handles nested sections and deeply linked pages

**Tested with:**
- Docusaurus v2 and v3
- Standard HTML documentation sites

## API Limits

**Cohere Free Tier:**
- 100 API calls/minute
- 10,000 embeddings/month
- Embedding model: `embed-english-v3.0` (1024-dimensional vectors)

**Qdrant Cloud Free Tier:**
- Up to 100k points per collection
- 1 collection
- Full API access

## Troubleshooting

### "API key invalid" Error

```
ERROR: Cohere API key validation failed
→ Check COHERE_API_KEY in .env
→ Verify key is copied correctly from https://cohere.com/
```

### "Qdrant connection failed"

```
ERROR: Cannot connect to Qdrant at {QDRANT_URL}
→ Verify QDRANT_URL format: https://<cluster>.qdrant.io:6333
→ Check QDRANT_API_KEY is correct
→ Verify internet connectivity
```

### "No pages discovered"

```
ERROR: Crawler found 0 pages at {DOCS_URL}
→ Verify DOCS_URL points to a valid Docusaurus site
→ Check site is publicly accessible
→ Try crawling manually in browser
```

### Rate Limiting

```
WARNING: Cohere API rate limit exceeded
→ Pipeline pauses and waits
→ Check Cohere dashboard for quota usage
→ Reduce COHERE_BATCH_SIZE if needed
```

## Contributing

To extend the pipeline:

1. **Add new embeddings model**: Extend `ingestion/embeddings/`
2. **Support new documentation format**: Create new crawler in `ingestion/crawlers/`
3. **Implement custom chunking strategy**: Extend `ingestion/chunking/text_chunker.py`
4. **Add new vector database**: Create new storage adapter in `ingestion/storage/`

## License

MIT License - See LICENSE file for details

## Support

For issues, questions, or contributions:
- Check existing tests in `tests/` for usage examples
- Review `spec.md` in the parent directory for feature requirements
- Refer to `plan.md` for architecture and design decisions
