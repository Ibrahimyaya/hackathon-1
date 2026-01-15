# Implementation Tasks: Deploy Documentation to Vector Database for RAG

**Feature Branch**: `001-docs-rag-ingest`
**Status**: Ready for implementation
**Plan Reference**: [plan.md](plan.md)
**Spec Reference**: [spec.md](spec.md)

---

## Task Execution Order & Dependencies

Tasks are grouped by implementation phase. **Each task is independently testable** — completion of a task should result in runnable, tested code even if subsequent phases are incomplete.

**Critical Path**: Task 1.1 → Task 1.2 → Task 2.1 → Task 2.2 → Task 2.3 → Task 3.1 → Task 3.2 → Task 3.3 → Task 4.1 → Task 4.2 → Task 5.1 → Task 5.2

---

## Phase 1: Core Infrastructure

### Task 1.1: Initialize Python Project with uv

**Objective**: Set up project structure, initialize `uv` package manager, and define all dependencies.

**Acceptance Criteria**:
- [ ] `backend/` directory exists with proper Python package structure
- [ ] `pyproject.toml` specifies Python 3.11+, lists all required dependencies
- [ ] `.env.example` template includes all 13+ configuration variables with descriptions
- [ ] `uv sync` or `uv install` successfully installs all dependencies without errors
- [ ] No import errors when running `python -c "import requests; import pydantic; import qdrant_client; import cohere"`

**Files to Create**:
- `backend/pyproject.toml` — Project config with dependencies
- `backend/.env.example` — Environment variable template
- `backend/.gitignore` — Python-specific ignores (venv/, __pycache__/, *.pyc, .env, .DS_Store)
- `backend/README.md` — Quick setup guide

**Dependencies**:
- None (bootstrap task)

**Test Strategy**:
- Manual: Run `uv sync` and verify no errors
- Verify imports work: `python -c "import requests; import pydantic; import qdrant_client; import cohere"`

**Estimated Effort**: 30 minutes

---

### Task 1.2: Implement Configuration & Logging Foundation

**Objective**: Create reusable configuration loading, logging setup, and custom exception hierarchy.

**Acceptance Criteria**:
- [ ] `utils/config.py` loads all .env variables with type validation (pydantic)
- [ ] Missing required variables raise clear error with guidance
- [ ] `utils/logging.py` sets up structured JSON logging (or human-readable for development)
- [ ] `utils/errors.py` defines exception hierarchy (IngestionError with 8+ subclasses per plan)
- [ ] Unit tests pass: test_config validates loading and error cases
- [ ] Unit tests pass: test_logging validates format and output
- [ ] Logging can be toggled between JSON and human via `LOG_FORMAT` env var

**Files to Create**:
- `backend/ingestion/__init__.py` — Package marker
- `backend/utils/__init__.py` — Package marker
- `backend/utils/config.py` — Config loader with pydantic
- `backend/utils/logging.py` — Logging setup
- `backend/utils/errors.py` — Exception hierarchy
- `backend/tests/__init__.py` — Tests package marker
- `backend/tests/unit/__init__.py`
- `backend/tests/conftest.py` — pytest fixtures
- `backend/tests/unit/test_config.py` — Config tests
- `backend/tests/unit/test_errors.py` — Error hierarchy tests (basic)

**Dependencies**:
- Task 1.1 (pyproject.toml, dependencies installed)

**Test Strategy**:
- Unit tests with pytest; test config loading, validation, and error cases
- Test logging output format

**Estimated Effort**: 45 minutes

---

## Phase 2: Crawling & Text Extraction

### Task 2.1: Implement Docusaurus Crawler

**Objective**: Build crawler that discovers and fetches all public pages from a Docusaurus documentation site.

**Acceptance Criteria**:
- [ ] `ingestion/crawlers/docusaurus_crawler.py` exports `DocusaurusCrawler` class with `crawl(url, max_pages)` method
- [ ] Crawler discovers pages via sitemap.xml (if available) or BFS from root URL
- [ ] Crawler fetches HTML with proper User-Agent and timeout (configurable)
- [ ] Returns list of (url, html_content) tuples for all discovered pages
- [ ] Handles network errors gracefully: retries transient errors (3x with exponential backoff), logs and skips permanent failures
- [ ] Logs progress: pages discovered, pages fetched, failed URLs, total duration
- [ ] Unit tests pass: test with mocked HTTP responses (mock requests library)
- [ ] Integration test passes: crawl real Docusaurus site (e.g., docs.docusaurus.io); validate >95% discovery

**Files to Create**:
- `backend/ingestion/crawlers/__init__.py`
- `backend/ingestion/crawlers/docusaurus_crawler.py` — Crawler class
- `backend/tests/unit/test_docusaurus_crawler.py` — Unit tests with mocks
- `backend/tests/integration/test_crawl_integration.py` — Integration test

**Dependencies**:
- Task 1.1 (requests, beautifulsoup4 installed)
- Task 1.2 (config, logging, errors available)

**Test Strategy**:
- Unit: Mock HTTP responses; test discovery logic, retry logic, error handling
- Integration: Crawl a real Docusaurus site; verify page count >95% of expected

**Estimated Effort**: 1 hour

---

### Task 2.2: Implement HTML-to-Text Extractor

**Objective**: Clean HTML pages and extract main content text, preserving structure and metadata.

**Acceptance Criteria**:
- [ ] `ingestion/processors/text_cleaner.py` exports `TextCleaner` class with `extract_text(html, url)` method
- [ ] Removes navigation boilerplate (sidebars, headers, footers) using BeautifulSoup + heuristics
- [ ] Extracts main content and preserves structure: headings, code blocks, lists, paragraphs
- [ ] Returns dict with keys: `text` (cleaned), `url`, `section_hierarchy` (breadcrumb), `metadata`
- [ ] Handles malformed HTML gracefully (no crashes)
- [ ] Text is readable and >90% of original content preserved (manual spot check)
- [ ] Unit tests pass: test with sample HTML snippets (various styles)
- [ ] Integration test passes: extract text from crawled pages, spot-check readability

**Files to Create**:
- `backend/ingestion/processors/__init__.py`
- `backend/ingestion/processors/text_cleaner.py` — Text extractor
- `backend/tests/unit/test_text_cleaner.py` — Unit tests with HTML samples
- `backend/tests/integration/test_extract_integration.py` — Integration test (optional: use crawled pages)

**Dependencies**:
- Task 1.1 (beautifulsoup4 installed)
- Task 1.2 (config, logging available)

**Test Strategy**:
- Unit: Test with various HTML structures (blog, API docs, wiki style)
- Integration: Extract text from real pages; review samples manually for quality

**Estimated Effort**: 1 hour

---

### Task 2.3: Integrate Crawler & Extractor into Pipeline

**Objective**: Wire crawling and extraction stages into `main.py` and verify end-to-end text extraction.

**Acceptance Criteria**:
- [ ] `backend/main.py` has `crawl_and_extract()` function that:
  - Loads `DOCS_URL` and `CRAWL_MAX_PAGES` from config
  - Calls `DocusaurusCrawler.crawl()` to get pages
  - Calls `TextCleaner.extract_text()` on each page
  - Returns list of extracted text with metadata
- [ ] Logs progress: pages crawled, pages extracted, errors, total time
- [ ] Logs summary stats: total text characters, average text per page
- [ ] Integration test passes: full crawl → extract on sample site
- [ ] Can be run via `python main.py --crawl-extract` (argument parsing in main)

**Files to Modify/Create**:
- `backend/main.py` — Add crawl_and_extract() function and arg parsing
- `backend/tests/integration/test_crawl_to_extract.py` — End-to-end test

**Dependencies**:
- Task 2.1 (crawler implemented)
- Task 2.2 (extractor implemented)

**Test Strategy**:
- Integration: Run full pipeline on sample docs; verify output structure and content quality

**Estimated Effort**: 30 minutes

---

## Phase 3: Chunking & Embedding

### Task 3.1: Implement Text Chunking

**Objective**: Split extracted text into semantically meaningful chunks with token-aware sizing.

**Acceptance Criteria**:
- [ ] `ingestion/chunking/text_chunker.py` exports `TextChunker` class with `chunk(text, metadata)` method
- [ ] `ingestion/models.py` defines `DocumentChunk` dataclass with fields: id, text, source_url, section, chunk_index, token_count
- [ ] Chunks are split at logical boundaries (paragraph breaks, section headers)
- [ ] Token counting using tiktoken (estimate for embedding model)
- [ ] Respects `CHUNK_MIN_TOKENS` and `CHUNK_MAX_TOKENS` config (default 256–512)
- [ ] Returns list of DocumentChunk objects with sequential chunk_index per source
- [ ] Unit tests pass: test chunking with various text samples (prose, code, mixed)
- [ ] >95% of chunks within configured token limits
- [ ] Chunk boundaries are logical (not mid-sentence, mid-code block)

**Files to Create**:
- `backend/ingestion/models.py` — DocumentChunk, Embedding data classes
- `backend/ingestion/chunking/__init__.py`
- `backend/ingestion/chunking/text_chunker.py` — Chunking logic
- `backend/tests/unit/test_text_chunker.py` — Unit tests
- `backend/tests/unit/test_models.py` — Model validation tests

**Dependencies**:
- Task 1.1 (tiktoken installed)
- Task 1.2 (config, logging available)

**Test Strategy**:
- Unit: Test with prose, code, mixed content; verify token counts and chunk boundaries

**Estimated Effort**: 1 hour

---

### Task 3.2: Implement Cohere Embeddings Client

**Objective**: Wrap Cohere API to batch-generate embeddings for text chunks.

**Acceptance Criteria**:
- [ ] `ingestion/embeddings/cohere_embedder.py` exports `CohereEmbedder` class with `embed(chunks)` method
- [ ] Takes list of DocumentChunks; returns list of Embedding objects (with vector, chunk_id, metadata)
- [ ] Batches requests: up to `COHERE_BATCH_SIZE` (default 100) chunks per API call
- [ ] Handles Cohere API errors: rate limit (backoff), quota exceeded (pause and report), network errors (retry)
- [ ] Logs embedding stats: total chunks, successful, failed, latency per batch
- [ ] Vector dimension is 1024 (embed-english-v3.0 model)
- [ ] Unit tests pass: mock Cohere responses, test batching and error handling
- [ ] All chunks receive embeddings or failures are logged (no silent drops)

**Files to Create**:
- `backend/ingestion/embeddings/__init__.py`
- `backend/ingestion/embeddings/cohere_embedder.py` — Cohere client
- `backend/tests/unit/test_cohere_embedder.py` — Unit tests with mocks

**Dependencies**:
- Task 1.1 (cohere-python installed)
- Task 1.2 (config, logging, errors available)
- Task 3.1 (DocumentChunk, Embedding models)

**Test Strategy**:
- Unit: Mock Cohere API; test batching, error handling (rate limit, quota, network)

**Estimated Effort**: 1 hour

---

### Task 3.3: Integrate Chunking & Embedding Pipeline

**Objective**: Wire chunking and embedding into full pipeline and test end-to-end.

**Acceptance Criteria**:
- [ ] `backend/main.py` has `chunk_and_embed()` function that:
  - Takes extracted texts from `crawl_and_extract()`
  - Calls `TextChunker.chunk()` on each text
  - Calls `CohereEmbedder.embed()` on all chunks
  - Returns list of embeddings with metadata
- [ ] Logs progress: chunks created, embedding stats, total time
- [ ] Summary stats: total chunks, tokens, embedding latency
- [ ] Can be run via `python main.py --chunk-embed` (with crawl/extract data cached or piped)
- [ ] Integration test passes: extract → chunk → embed on sample docs

**Files to Modify/Create**:
- `backend/main.py` — Add chunk_and_embed() function
- `backend/tests/integration/test_chunk_embed_integration.py` — End-to-end test

**Dependencies**:
- Task 3.1 (chunker implemented)
- Task 3.2 (embedder implemented)
- Task 2.3 (crawl/extract working)

**Test Strategy**:
- Integration: Extract sample docs, chunk, embed; verify output structure and embedding dimensions

**Estimated Effort**: 30 minutes

---

## Phase 4: Storage & Retrieval

### Task 4.1: Implement Qdrant Storage & Search

**Objective**: Store embeddings and metadata in Qdrant, implement similarity search.

**Acceptance Criteria**:
- [ ] `ingestion/storage/qdrant_store.py` exports `QdrantStore` class with:
  - `__init__(config)` — connects to Qdrant, creates/validates collection
  - `upsert(embeddings)` — batch-inserts embeddings with payload (text, url, section, etc.)
  - `search(query_embedding, k=5)` — returns top-k similar chunks with scores
- [ ] Collection has correct schema: vector dimension 1024, indexed metadata fields (url, section)
- [ ] Handles Qdrant errors: connection failures, collection errors, query failures
- [ ] Logs storage stats: chunks upserted, query latencies
- [ ] Unit tests pass: mock Qdrant, test upsert and search logic
- [ ] Integration test passes: store real embeddings, execute test queries

**Files to Create**:
- `backend/ingestion/storage/__init__.py`
- `backend/ingestion/storage/qdrant_store.py` — Qdrant client
- `backend/tests/unit/test_qdrant_store.py` — Unit tests with mocks
- `backend/tests/integration/test_qdrant_integration.py` — Integration test (requires live Qdrant)

**Dependencies**:
- Task 1.1 (qdrant-client installed)
- Task 1.2 (config, logging, errors available)
- Task 3.1 (Embedding model)

**Test Strategy**:
- Unit: Mock Qdrant; test collection creation, upsert, search
- Integration: Use Qdrant Cloud or local Docker instance; store real embeddings, execute queries

**Estimated Effort**: 1.5 hours

---

### Task 4.2: Integrate Storage into Full Pipeline

**Objective**: Complete end-to-end pipeline: crawl → chunk → embed → store.

**Acceptance Criteria**:
- [ ] `backend/main.py` has `ingest_pipeline()` function that:
  - Calls `crawl_and_extract()`, `chunk_and_embed()`, `QdrantStore.upsert()`
  - Orchestrates full pipeline sequentially
  - Validates prerequisites (config, API keys, connectivity)
  - Handles errors at each stage, logs comprehensive summary
- [ ] Summary stats reported: pages crawled, chunks created, embeddings generated, documents stored, failures
- [ ] Can be run via `python main.py --ingest` with all required env vars set
- [ ] Full pipeline completes without unhandled exceptions (or logs and exits gracefully)
- [ ] Integration test passes: full pipeline on sample docs

**Files to Modify/Create**:
- `backend/main.py` — Add ingest_pipeline(), update arg parsing
- `backend/tests/integration/test_full_pipeline.py` — Full end-to-end test

**Dependencies**:
- Task 4.1 (storage implemented)
- Task 3.3 (chunking/embedding working)

**Test Strategy**:
- Integration: Full pipeline with sample docs; verify all data reaches Qdrant

**Estimated Effort**: 45 minutes

---

## Phase 5: Validation & Retrieval Testing

### Task 5.1: Implement Pipeline Orchestration & Validation

**Objective**: Add startup validation, error handling, and comprehensive reporting to main().

**Acceptance Criteria**:
- [ ] `backend/main.py` `main()` function:
  - Loads and validates config (all required env vars present and valid)
  - Tests Cohere API connectivity (quick embed request)
  - Tests Qdrant connectivity (collection query)
  - Calls `ingest_pipeline()` if all prerequisites pass
  - Wraps in top-level exception handler with full context logging
  - Prints final summary: pages, chunks, embeddings, failures, total time
- [ ] Can be invoked via `python main.py` with proper CLI argument parsing
- [ ] Exit codes: 0 on success, non-zero on failure
- [ ] Unit tests pass: test config validation, error cases
- [ ] Integration test passes: full pipeline with real endpoints

**Files to Modify/Create**:
- `backend/main.py` — Finalize main() and arg parsing
- `backend/tests/unit/test_main_validation.py` — Unit tests for validation

**Dependencies**:
- Task 4.2 (full pipeline integrated)
- All prior tasks (config, logging, errors)

**Test Strategy**:
- Unit: Test validation logic, error handling
- Integration: Run full main() with real Qdrant

**Estimated Effort**: 45 minutes

---

### Task 5.2: Add Test Queries & Validation

**Objective**: Implement test queries to validate that stored embeddings are retrievable and relevant.

**Acceptance Criteria**:
- [ ] `backend/main.py` has `--test-queries` mode that:
  - Loads pre-defined test queries (e.g., "How to install?", "Configuration options")
  - Generates embeddings for each query using Cohere
  - Searches Qdrant for top-3 most similar chunks
  - Prints results with scores and chunk text
  - Logs whether results are relevant (manual assessment or heuristic)
- [ ] Retrieves and displays results with scores, chunk text, source URLs
- [ ] Integration test passes: test queries on stored embeddings return relevant results
- [ ] >80% of test queries return at least one directly relevant result in top-5

**Files to Modify/Create**:
- `backend/main.py` — Add test_queries() function and mode
- `backend/tests/integration/test_queries.py` — Query test suite

**Dependencies**:
- Task 5.1 (full pipeline working)

**Test Strategy**:
- Manual testing: Run test queries, review results for relevance
- Integration test: Automated test queries with basic relevance checks

**Estimated Effort**: 1 hour

---

## Summary

**Total Tasks**: 12
**Total Estimated Effort**: 9–10 hours of implementation and testing
**Critical Path**: Task 1.1 → 1.2 → 2.1 → 2.2 → 2.3 → 3.1 → 3.2 → 3.3 → 4.1 → 4.2 → 5.1 → 5.2

**Definition of Done (per task)**:
1. All acceptance criteria marked complete
2. Unit tests pass (if applicable)
3. Integration tests pass (if applicable)
4. Code is committed to feature branch
5. PHR recorded for traceability

**Blockers & Constraints**:
- Cohere API key and quota required (free tier available)
- Qdrant Cloud account required (free tier available)
- Target Docusaurus site must be publicly accessible
- Network connectivity required for external APIs
