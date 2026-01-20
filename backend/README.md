# RAG Documentation Ingestion Pipeline

A Python-based pipeline for extracting documentation from Docusaurus sites, chunking content, generating embeddings via Cohere, and storing them in Qdrant for vector similarity search.

## Quick Start

### Prerequisites

- **Python 3.11+**
- **uv** (Python package manager) or pip
- **Cohere API Key**: Get free tier at https://cohere.com/
- **Qdrant Cloud Account**: Get free tier at https://qdrant.io/

### Setup

1. **Configure Environment**
   ```bash
   cp .env.example .env
   # Edit .env with your Cohere API key, Qdrant credentials, and docs URL
   ```

2. **Install Dependencies**
   ```bash
   # Using uv:
   uv sync
   
   # Or using pip:
   pip install -e ".[dev]"
   ```

3. **Validate Configuration**
   ```bash
   python main.py --validate
   ```

### Usage

#### Full Pipeline (Crawl → Chunk → Embed → Store)
```bash
python main.py --full-pipeline
```

#### Individual Stages
```bash
# Crawl and extract documentation
python main.py --crawl-extract

# Chunk and generate embeddings
python main.py --chunk-embed

# Ingest into Qdrant
python main.py --ingest

# Run test queries
python main.py --test-queries
```

## Project Structure

```
backend/
├── pyproject.toml              # Python project configuration
├── .env.example                # Configuration template
├── .gitignore                  # Git ignore patterns
├── README.md                   # This file
├── main.py                     # Entry point and orchestration
│
├── ingestion/                  # Core ingestion pipeline
│   ├── __init__.py
│   ├── models.py               # Data classes (DocumentChunk, Embedding)
│   ├── crawlers/
│   │   ├── __init__.py
│   │   └── docusaurus_crawler.py   # Docusaurus site crawler
│   ├── processors/
│   │   ├── __init__.py
│   │   └── text_cleaner.py         # HTML to text extraction
│   ├── chunking/
│   │   ├── __init__.py
│   │   └── text_chunker.py         # Semantic text chunking
│   ├── embeddings/
│   │   ├── __init__.py
│   │   └── cohere_embedder.py      # Cohere embeddings client
│   └── storage/
│       ├── __init__.py
│       └── qdrant_store.py         # Qdrant vector storage
│
├── utils/
│   ├── __init__.py
│   ├── config.py                   # Configuration loading and validation
│   ├── logging.py                  # Structured JSON logging
│   └── errors.py                   # Custom exception hierarchy
│
└── tests/
    ├── conftest.py                 # pytest fixtures
    ├── unit/
    │   ├── test_config.py
    │   ├── test_errors.py
    │   ├── test_docusaurus_crawler.py
    │   ├── test_text_cleaner.py
    │   ├── test_text_chunker.py
    │   ├── test_cohere_embedder.py
    │   └── test_qdrant_store.py
    └── integration/
        ├── test_crawl_to_extract.py
        ├── test_chunk_embed_integration.py
        ├── test_qdrant_integration.py
        ├── test_full_pipeline.py
        └── test_queries.py
```

## Configuration

All configuration is managed via environment variables in `.env`. Key settings:

### Crawling
- `DOCS_URL`: Target Docusaurus site (required)
- `CRAWL_MAX_PAGES`: Maximum pages to crawl (default: 1000)
- `CRAWL_TIMEOUT_SECONDS`: Per-page timeout (default: 10)

### Chunking
- `CHUNK_MIN_TOKENS`: Minimum chunk size (default: 256)
- `CHUNK_MAX_TOKENS`: Maximum chunk size (default: 512)
- `CHUNK_OVERLAP_TOKENS`: Context overlap (default: 50)

### Embeddings (Cohere)
- `COHERE_API_KEY`: Your Cohere API key (required)
- `COHERE_MODEL`: Model identifier (default: embed-english-v3.0)
- `COHERE_BATCH_SIZE`: Batch size (default: 100)

### Storage (Qdrant)
- `QDRANT_API_KEY`: Your Qdrant API key (required)
- `QDRANT_URL`: Qdrant endpoint (required)
- `QDRANT_COLLECTION_NAME`: Collection name (default: docs_chunks)
- `QDRANT_VECTOR_SIZE`: Vector dimension (default: 1024)

### Logging
- `LOG_LEVEL`: DEBUG, INFO, WARNING, ERROR, CRITICAL (default: INFO)
- `LOG_FORMAT`: json or human (default: json)

## Running Tests

```bash
# All tests
pytest

# Unit tests only
pytest tests/unit/

# Integration tests only
pytest tests/integration/

# With coverage
pytest --cov=. --cov-report=html
```

## Pipeline Stages

### 1. Crawl & Extract
- Discovers all public pages from target Docusaurus site
- Fetches HTML content with retry logic
- Extracts clean text while preserving structure
- Outputs: List of DocumentChunk objects with URLs and text

### 2. Chunk & Embed
- Splits extracted text at logical boundaries (sections, paragraphs)
- Counts tokens and ensures chunks respect size limits
- Generates embeddings via Cohere API
- Handles rate limiting and quota errors gracefully
- Outputs: DocumentChunk objects with vector embeddings

### 3. Store & Index
- Connects to Qdrant and validates/creates collection
- Batch upserts embeddings with metadata
- Implements similarity search for retrieval
- Outputs: Stored vectors indexed in Qdrant

### 4. Validate & Query
- Validates full pipeline end-to-end
- Executes test queries for relevance verification
- Generates summary statistics and reports

## Error Handling

The pipeline is designed to be resilient:

- **Crawling**: Retries transient errors (3x with exponential backoff), skips permanent failures
- **Chunking**: Validates token counts, skips malformed chunks with logging
- **Embedding**: Handles rate limits and quota errors, batches requests for efficiency
- **Storage**: Validates Qdrant connection on startup, creates collection if missing
- **Pipeline**: Top-level exception handler logs context and provides summary stats

All errors are logged with full context for debugging.

## Development

### Local Qdrant Setup
For development without Qdrant Cloud:
```bash
docker run -p 6333:6333 qdrant/qdrant
```
Then set `QDRANT_URL=http://localhost:6333` in `.env`.

### Code Quality
```bash
# Format code
black backend/

# Lint
ruff check backend/

# Type check
mypy backend/
```

## Troubleshooting

### "Connection refused" for Qdrant
- Verify `QDRANT_URL` is correct and Qdrant is running
- Check API key is valid
- For Docker: ensure container is running (`docker ps`)

### "API Key invalid" for Cohere
- Verify `COHERE_API_KEY` is set correctly in `.env`
- Check key is valid at https://cohere.com/

### "No pages crawled"
- Verify `DOCS_URL` is correct and publicly accessible
- Check network connectivity and firewall rules
- Verify the site is a Docusaurus site (has sitemap.xml or compatible structure)

## Success Criteria

- ✅ >95% of public pages discovered
- ✅ >90% of original content preserved in text extraction
- ✅ >95% of chunks within configured token limits
- ✅ All embeddings successfully stored in Qdrant
- ✅ Similarity search returns relevant results within 100ms
- ✅ All tests pass

## References

- [Docusaurus Documentation](https://docusaurus.io/)
- [Cohere API Reference](https://docs.cohere.com/)
- [Qdrant Documentation](https://qdrant.tech/documentation/)
- [Semantic Chunking Guide](https://docs.anthropic.com/)
