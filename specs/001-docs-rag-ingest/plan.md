# Implementation Plan: Deploy Documentation to Vector Database for RAG

**Feature Branch**: `001-docs-rag-ingest`
**Created**: 2026-01-15
**Status**: Draft (awaiting approval)
**Spec Reference**: [spec.md](spec.md)

## Architecture Overview

The RAG documentation ingestion system is a Python-based pipeline that extracts documentation from deployed Docusaurus sites, chunks the content, generates embeddings using Cohere, and stores them in Qdrant Cloud for vector similarity search. The implementation follows a modular, task-oriented approach aligned with the spec's user stories.

**Key Design Philosophy:**
- Separation of concerns: Each pipeline stage (crawl, chunk, embed, store) is independently testable and reusable
- Configuration-driven: All external dependencies managed via `.env`
- Error resilience: Graceful degradation with detailed logging at each stage
- Minimal viable structure: Start with a single `main.py` entry point that orchestrates modular functions

---

## Module Structure

The backend will be organized as:

```
backend/
├── pyproject.toml              # uv project configuration (Python 3.11+)
├── .env.example                # Template for environment variables
├── main.py                      # Entry point with full pipeline orchestration
│
├── ingestion/                  # Core ingestion package
│   ├── __init__.py
│   ├── crawlers/
│   │   ├── __init__.py
│   │   └── docusaurus_crawler.py     # URL crawling & HTML extraction
│   │
│   ├── processors/
│   │   ├── __init__.py
│   │   └── text_cleaner.py           # Text extraction & cleaning from HTML
│   │
│   ├── chunking/
│   │   ├── __init__.py
│   │   └── text_chunker.py           # Semantic chunking with token counting
│   │
│   ├── embeddings/
│   │   ├── __init__.py
│   │   └── cohere_embedder.py        # Cohere API client wrapper
│   │
│   ├── storage/
│   │   ├── __init__.py
│   │   └── qdrant_store.py           # Qdrant collection management & queries
│   │
│   └── models.py                      # Data classes (DocumentChunk, Embedding)
│
├── utils/
│   ├── __init__.py
│   ├── config.py                      # Environment & configuration loading
│   ├── logging.py                     # Structured logging setup
│   └── errors.py                      # Custom exception hierarchy
│
└── tests/
    ├── unit/
    │   ├── test_text_cleaner.py
    │   ├── test_text_chunker.py
    │   └── test_models.py
    ├── integration/
    │   ├── test_crawl_to_chunk.py
    │   └── test_embed_to_store.py
    └── conftest.py                    # pytest fixtures
```

**Rationale:**
- **ingestion/** contains the five core modules corresponding to P1 user stories (crawl, chunk, embed, store), plus shared data models
- **utils/** centralizes cross-cutting concerns (config, logging, errors)
- **tests/** follows pytest conventions with unit and integration test separation
- **main.py** is the single orchestration entry point, with supporting modules reusable independently

---

## Configuration & Environment

**Environment Variables (.env)**

```env
# Crawling Configuration
DOCS_URL=https://docs.example.com           # Target Docusaurus site root
CRAWL_MAX_PAGES=1000                        # Max pages to crawl per session
CRAWL_TIMEOUT_SECONDS=10                    # Per-page timeout
CRAWL_FOLLOW_EXTERNAL_LINKS=false           # Only follow same-domain links

# Chunking Configuration
CHUNK_MIN_TOKENS=256                        # Minimum chunk size
CHUNK_MAX_TOKENS=512                        # Maximum chunk size (recommended)
CHUNK_OVERLAP_TOKENS=50                     # Overlap for context preservation

# Cohere API Configuration
COHERE_API_KEY=<secret>                     # Cohere API key (required)
COHERE_MODEL=embed-english-v3.0            # Embedding model identifier
COHERE_BATCH_SIZE=100                       # Batch embed requests

# Qdrant Configuration
QDRANT_API_KEY=<secret>                     # Qdrant Cloud API key
QDRANT_URL=https://<cluster>.qdrant.io:6333  # Qdrant Cloud endpoint
QDRANT_COLLECTION_NAME=docs_chunks          # Collection for storing embeddings
QDRANT_VECTOR_SIZE=1024                     # Cohere embed-english-v3.0 dimension
QDRANT_RECREATE_COLLECTION=false            # Recreate collection on init (dev only)

# Logging & Debugging
LOG_LEVEL=INFO                              # DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_FORMAT=json                             # json or human (structured logging)
```

**Configuration Loading Strategy:**
- Use Python `pydantic` for type-safe environment variable validation
- Create a `Config` class that loads and validates all settings on startup
- Fail fast with clear error messages if required variables are missing

---

## Error Handling Strategy

**Error Hierarchy (custom exceptions in `utils/errors.py`):**

```
IngestionError (base)
├── CrawlError
│   ├── CrawlTimeoutError
│   ├── CrawlNetworkError
│   └── CrawlValidationError
├── ProcessingError
│   ├── ExtractionError
│   └── CleaningError
├── ChunkingError
│   ├── TokenizationError
│   └── ChunkingStrategyError
├── EmbeddingError
│   ├── CohereAPIError
│   └── CohereQuotaError
└── StorageError
    ├── QdrantConnectionError
    ├── QdrantCollectionError
    └── QdrantQueryError
```

**Error Handling Patterns:**

1. **Crawling Stage**: Retry transient errors (network timeouts, 5xx) up to 3 times with exponential backoff. Log and skip unreachable pages but continue pipeline.

2. **Processing & Chunking**: Validate output (non-empty text, reasonable token counts). If validation fails, log warning and skip chunk but continue.

3. **Embedding Generation**: Handle Cohere API errors (rate limits, quota exceeded). If quota exceeded, pause and offer manual retry. Batch failures are logged per chunk.

4. **Storage**: Validate Qdrant connection on startup. If collection doesn't exist, create it with correct schema. Connection failures halt the pipeline with clear error messages.

5. **Pipeline-Level**: Wrap `main()` in top-level exception handler that:
   - Logs full exception trace with context (stage, document URL, chunk ID)
   - Reports summary stats (pages crawled, chunks generated, embeddings stored, failures)
   - Exits with non-zero code on critical failures (missing API keys, no pages crawled)

---

## Implementation Tasks (Breakdown by Priority)

### Phase 1: Core Infrastructure (Prerequisite)

**Task 1.1: Project Initialization**
- Create `backend/` directory structure
- Initialize `pyproject.toml` with `uv` (Python 3.11+, minimal dependencies: requests, beautifulsoup4, pydantic, cohere-python, qdrant-client)
- Create `.env.example` template with all required variables
- Status: One-time setup, no dependencies

**Task 1.2: Configuration & Logging Foundation**
- Implement `utils/config.py` with pydantic Config class
- Implement `utils/logging.py` with structured JSON logging setup
- Implement `utils/errors.py` with exception hierarchy
- Status: Blocks all other tasks; test with unit tests for validation
- Dependencies: Task 1.1 (config, pyproject.toml in place)

---

### Phase 2: User Story P1.1 - Crawl & Extract (Highest Priority)

**Task 2.1: Implement Docusaurus Crawler**
- Create `ingestion/crawlers/docusaurus_crawler.py`
- Functionality:
  - Discover all public documentation pages (via sitemap.xml or BFS crawl)
  - Fetch HTML content with proper User-Agent and timeout handling
  - Log crawl progress (pages discovered, failed URLs)
- Dependencies: requests, beautifulsoup4, Task 1.1
- Tests: Unit tests with mocked responses; integration test against a sample Docusaurus site
- Acceptance: >95% of public pages discovered, no unhandled exceptions

**Task 2.2: Implement HTML-to-Text Extraction**
- Create `ingestion/processors/text_cleaner.py`
- Functionality:
  - Remove navigation boilerplate (sidebars, headers, footers) using heuristics or HTML structure
  - Extract main content area and preserve semantic structure (headings, code blocks, lists)
  - Clean text (normalize whitespace, handle special characters)
  - Preserve source metadata (URL, page section/heading hierarchy)
- Dependencies: beautifulsoup4, Task 1.1
- Tests: Unit tests with sample HTML; validate that information loss is minimal
- Acceptance: >90% of original content preserved, clean readable text

**Task 2.3: Integrate Crawler & Processor**
- Wire `main.py` to call crawler, then processor
- Log output: URLs processed, character counts, errors
- Status: End-to-end crawl → text extraction working
- Dependencies: Task 2.1, Task 2.2, Task 1.2

---

### Phase 3: User Story P1.2 - Chunk & Embed (Critical Path)

**Task 3.1: Implement Text Chunking**
- Create `ingestion/chunking/text_chunker.py` with models.py for DocumentChunk
- Functionality:
  - Split text at logical boundaries (sentences, paragraphs, section headers)
  - Count tokens using a tokenizer (tiktoken for rough counts)
  - Ensure chunks respect configurable limits (256–512 tokens default)
  - Create DocumentChunk objects with metadata (source URL, section, chunk index, token count)
- Dependencies: tiktoken, Task 1.1
- Tests: Unit tests with sample text; verify token counts and boundaries
- Acceptance: >95% of chunks within configured limits, logical splitting at section boundaries

**Task 3.2: Implement Cohere Embeddings Client**
- Create `ingestion/embeddings/cohere_embedder.py`
- Functionality:
  - Wrap Cohere API client (embed-english-v3.0 model)
  - Batch requests for efficiency (up to 100 chunks per API call)
  - Handle rate limiting and quota exceeded errors gracefully
  - Log embedding generation stats (total chunks, successful embeds, failures, latency)
- Dependencies: cohere-python, Task 1.1, Task 1.2
- Tests: Mock Cohere API responses; test batch logic and error handling
- Acceptance: All chunks receive embeddings or explicit failure log

**Task 3.3: Integrate Chunking & Embeddings**
- Wire `main.py` to call chunker, then embedder
- Log output: chunks generated, token statistics, embedding stats
- Status: End-to-end crawl → chunk → embed working
- Dependencies: Task 3.1, Task 3.2, Task 2.3

---

### Phase 4: User Story P1.3 - Store & Index (Essential for Completion)

**Task 4.1: Implement Qdrant Storage**
- Create `ingestion/storage/qdrant_store.py`
- Functionality:
  - Initialize/validate Qdrant collection with correct vector dimension (1024 for Cohere)
  - Batch insert embeddings with payload (chunk text, source URL, section, position, token count)
  - Implement vector similarity search query method (used in testing, not core ingestion)
  - Handle collection recreation (for dev/testing) and graceful upgrades
- Dependencies: qdrant-client, Task 1.1, Task 1.2
- Tests: Integration tests with Qdrant Cloud (or local Qdrant Docker instance for testing)
- Acceptance: All embeddings indexed, queries return relevant results within 100ms

**Task 4.2: Integrate Storage into Pipeline**
- Wire `main.py` to call embedder, then storage
- Log output: total documents stored, metadata validation
- Status: Complete end-to-end pipeline: crawl → chunk → embed → store
- Dependencies: Task 4.1, Task 3.3

---

### Phase 5: User Story P1.4 - End-to-End Validation (P2 - Secondary)

**Task 5.1: Implement Pipeline Orchestration in main.py**
- Create main function that:
  - Loads configuration from .env
  - Validates all external dependencies (Cohere API key, Qdrant connectivity)
  - Executes pipeline stages sequentially
  - Collects and reports final statistics (pages crawled, chunks created, embeddings stored, failures)
  - Implements retry logic for transient failures
- Dependencies: All prior tasks
- Tests: Integration test with real or mocked endpoints
- Acceptance: Full pipeline runs to completion, reports comprehensive stats

**Task 5.2: Add Test Queries & Validation**
- Implement basic vector similarity search test queries
- Example: "How to install the system?" should return relevant chunks
- Log top-3 results with scores
- Tests: Manual testing against sample documentation
- Acceptance: Returned results are topically relevant
- Dependencies: Task 5.1

---

## Dependencies & Prerequisites

**Python Dependencies (to be added to pyproject.toml):**
- `requests>=2.31.0` — HTTP client for crawling
- `beautifulsoup4>=4.12.0` — HTML parsing
- `pydantic>=2.4.0` — Configuration validation
- `cohere>=5.0.0` — Cohere SDK
- `qdrant-client>=2.6.0` — Qdrant Python client
- `tiktoken>=0.5.0` — Token counting
- `python-dotenv>=1.0.0` — .env file loading
- `structlog>=24.0.0` — Structured logging (optional but recommended)

**Development Dependencies:**
- `pytest>=7.4.0` — Testing
- `pytest-cov>=4.1.0` — Coverage reporting
- `pytest-asyncio>=0.21.0` — Async test support (if needed)
- `mock>=5.1.0` — Mocking for unit tests

**External Services (Required to be configured):**
1. **Cohere API**: Free tier available; register at https://cohere.com/
2. **Qdrant Cloud**: Free tier available; sign up at https://qdrant.io/
3. **Target Documentation Site**: Must be publicly accessible Docusaurus site

**Prerequisite Validations:**
- Cohere API key must be valid (test with simple embed request)
- Qdrant endpoint must be reachable (test with connection)
- Target docs URL must return valid HTML
- If any prerequisite fails, exit with clear error message before crawling

---

## Risks & Mitigation

**Risk 1: Rate Limiting from Cohere or Target Documentation Site**
- **Blast Radius**: Embedding generation halts; crawling crawls too fast
- **Mitigation**:
  - Implement exponential backoff for Cohere requests
  - Add configurable crawl delay (e.g., 500ms between requests)
  - Implement request batching (Cohere supports 100 chunks per request)
  - Log rate limit responses and offer manual retry
- **Kill Switch**: Skip rate-limited chunks and continue; report counts in final summary

**Risk 2: Large Documentation Sites Exceed API Quotas or Memory Limits**
- **Blast Radius**: Pipeline runs out of quota mid-crawl or OOM
- **Mitigation**:
  - Add `CRAWL_MAX_PAGES` configuration to limit ingestion scope
  - Implement pagination/resumable ingestion (track last successful URL)
  - Stream processing where possible (don't load all pages in memory)
  - Monitor token usage and quota before embedding
- **Kill Switch**: Pause before quota exhausted; provide resume mechanism

**Risk 3: Qdrant Collection Size Exceeds Free Tier Limits**
- **Blast Radius**: Cannot store additional embeddings
- **Mitigation**:
  - Document free tier limits clearly in .env.example
  - Implement collection size checking before insert
  - Suggest upgrade to paid tier with clear messaging
- **Kill Switch**: Warn user and pause; do not silently drop data

**Risk 4: HTML Extraction Quality Varies Across Documentation Styles**
- **Blast Radius**: Text extraction breaks for non-standard HTML structures
- **Mitigation**:
  - Use robust HTML parsing (BeautifulSoup + heuristics)
  - Consider fallback to trafilatura if BeautifulSoup extraction is sparse
  - Log extraction quality metrics (content ratio, cleanup impact)
  - Manual review of sample extracted text in tests
- **Kill Switch**: Allow configuration of content selector override (CSS classes/IDs)

**Risk 5: Network Transient Failures During Long Crawl Sessions**
- **Blast Radius**: Entire pipeline restarts or fails partway through
- **Mitigation**:
  - Implement retry logic with exponential backoff (3 retries default)
  - Log all failures with URL and error context
  - Save intermediate results (checkpoint crawled URLs, generated chunks)
  - Implement resume-from-checkpoint mechanism
- **Kill Switch**: Graceful shutdown with summary; user can rerun targeting failed URLs

---

## Key Architectural Decisions

This plan incorporates the following significant architectural choices:

1. **Modular Pipeline Design**: Each stage (crawl, chunk, embed, store) is a separate module with clear interfaces, enabling independent testing and potential parallelization in future iterations.

2. **Single Entry Point with Orchestration**: `main.py` serves as the command-line orchestrator, calling functions from each module sequentially. This simplifies initial implementation while keeping modules reusable.

3. **Configuration-Driven Behavior**: All external dependencies (URLs, API keys, limits) are environment-driven via `.env`, following the constitution's secrets management principle.

4. **Error Resilience Over Failure Halt**: The pipeline logs and skips individual failures (failed pages, unparseable chunks) rather than halting completely, supporting large-scale ingestion with partial degradation.

5. **Semantic Chunking with Token Awareness**: Chunks respect logical boundaries (sections, paragraphs) while staying within token limits, balancing embedding quality with retrieval relevance.

---

## Next Steps After Approval

Upon user approval, implementation will:
1. Create tasks.md with granular, independently testable tasks from the phase breakdown above
2. Begin Phase 1 (Project Initialization) immediately
3. Execute phases sequentially, with integration points at each phase boundary
4. Record progress in PHR records for traceability and learning

This plan is ready for review and approval.
