# Tasks: Deploy Documentation to Vector Database for RAG

**Feature Branch**: `001-docs-rag-ingest`
**Status**: Ready for implementation
**Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

---

## Executive Summary

- **Total Tasks**: 26 tasks across 6 phases
- **Estimated Effort**: 9–10 hours
- **Critical Path**: Phase 1 → Phase 2 → Phase 3 (US1) → Phase 4 (US2) → Phase 5 (US3) → Phase 6 (US4)
- **MVP Scope**: User Story 1 (Crawl & Extract) = Phase 1 + Phase 2 + Phase 3 (minimal viable)
- **Parallelization**: US2, US3 can run in parallel after Phase 2 completes

### User Stories Mapped to Phases

| User Story | Priority | Phase | Description |
|------------|----------|-------|-------------|
| US1: Crawl & Extract | P1 | Phase 3 | Discover and extract documentation from Docusaurus sites |
| US2: Chunk & Embed | P1 | Phase 4 | Split text into chunks and generate embeddings via Cohere |
| US3: Store & Index | P1 | Phase 5 | Persist embeddings in Qdrant and implement similarity search |
| US4: Validate Pipeline | P2 | Phase 6 | End-to-end validation and test query execution |

---

## Phase 1: Setup & Project Initialization

**Goal**: Initialize Python project structure with `uv`, dependencies, and configuration management.
**Duration**: 30 minutes
**Blocking**: All subsequent phases

### Checklist

- [ ] T001 Create backend/ directory structure and initialize pyproject.toml with uv configuration
- [ ] T002 Create .env.example template with all 13+ configuration variables with descriptions
- [ ] T003 Create backend/.gitignore with Python-specific patterns (venv/, __pycache__/, *.pyc, .env, .DS_Store)
- [ ] T004 Create backend/README.md with quick setup guide and prerequisites

**Success Criteria**:
- [ ] `uv sync` or `uv install` installs all dependencies without errors
- [ ] No import errors: `python -c "import requests; import pydantic; import qdrant_client; import cohere"`
- [ ] All environment variables documented in .env.example with descriptions

---

## Phase 2: Foundational Infrastructure

**Goal**: Establish config loading, logging, error handling, and data models used by all pipeline stages.
**Duration**: 45 minutes
**Blocking**: All user story phases

### Checklist

- [ ] T005 Implement utils/config.py with pydantic Config class for environment variable validation
- [ ] T006 Implement utils/logging.py with structured JSON logging (toggle via LOG_FORMAT env var)
- [ ] T007 Implement utils/errors.py with exception hierarchy (IngestionError, CrawlError, ProcessingError, ChunkingError, EmbeddingError, StorageError)
- [ ] T008 Create ingestion/models.py with DocumentChunk and Embedding dataclasses
- [ ] T009 Create backend/tests/conftest.py with pytest fixtures for config, logging, mocked APIs
- [ ] T010 Create backend/tests/unit/test_config.py: validate config loading, missing vars raise clear errors
- [ ] T011 Create backend/tests/unit/test_errors.py: test exception hierarchy and error messages

**Success Criteria**:
- [ ] Missing required variables raise clear error with guidance
- [ ] Logging output can toggle between JSON and human-readable
- [ ] All tests pass: `pytest backend/tests/unit/`
- [ ] Config loaded successfully in test environment

---

## Phase 3: User Story 1 - Crawl and Index Documentation Site (P1)

**Goal**: Discover and extract all public pages from a Docusaurus documentation site.
**Duration**: 1.5 hours
**Dependencies**: Phase 1, Phase 2
**Independent Test**: Execute crawler on Docusaurus site; verify >95% page discovery and extract clean text.

### Checklist

- [ ] T012 [P] [US1] Create ingestion/crawlers/docusaurus_crawler.py: Implement DocusaurusCrawler class with crawl(url, max_pages) method
- [ ] T013 [US1] Implement crawler to discover pages via sitemap.xml or breadth-first search
- [ ] T014 [US1] Implement crawl retry logic: exponential backoff (3x) for transient errors, skip permanent failures
- [ ] T015 [US1] Implement crawl logging: progress (pages discovered, fetched), errors, total duration
- [ ] T016 [P] [US1] Create ingestion/processors/text_cleaner.py: Implement TextCleaner class with extract_text(html, url) method
- [ ] T017 [US1] Implement HTML-to-text extraction: remove boilerplate, preserve structure (headings, code, lists)
- [ ] T018 [US1] Implement text cleaning: normalize whitespace, handle special characters, preserve metadata (URL, section hierarchy)
- [ ] T019 [P] [US1] Create backend/tests/unit/test_docusaurus_crawler.py: Unit tests with mocked HTTP responses
- [ ] T020 [US1] Create backend/tests/unit/test_text_cleaner.py: Unit tests with sample HTML (various styles)
- [ ] T021 [US1] Create backend/main.py: Implement crawl_and_extract() function that wires crawler → processor
- [ ] T022 [US1] Update backend/main.py: Add logging of extracted text stats (URLs, character counts, errors)
- [ ] T023 [P] [US1] Create backend/tests/integration/test_crawl_to_extract.py: End-to-end test on sample Docusaurus site

**Success Criteria - US1**:
- [ ] >95% of public pages discovered
- [ ] >90% of original content preserved in text extraction
- [ ] All tests pass: `pytest backend/tests/unit/test_docusaurus_crawler.py backend/tests/unit/test_text_cleaner.py`
- [ ] Integration test passes: crawl and extract real documentation
- [ ] No unhandled exceptions during crawl

---

## Phase 4: User Story 2 - Chunk Text and Generate Embeddings (P1)

**Goal**: Split extracted text into semantically meaningful chunks and generate embeddings via Cohere.
**Duration**: 1.5 hours
**Dependencies**: Phase 1, Phase 2, Phase 3 (for text input)
**Independent Test**: Chunk sample text, generate embeddings, validate token counts and semantic similarity.
**Parallel Opportunity**: Can start T024–T028 immediately after Phase 2 (no US1 dependency for units tests with mocks).

### Checklist

- [ ] T024 [P] [US2] Create ingestion/chunking/text_chunker.py: Implement TextChunker class with chunk(text, metadata) method
- [ ] T025 [US2] Implement text chunking: split at logical boundaries (paragraphs, section headers)
- [ ] T026 [US2] Implement token counting using tiktoken; ensure chunks respect CHUNK_MIN_TOKENS and CHUNK_MAX_TOKENS config
- [ ] T027 [US2] Implement DocumentChunk creation with metadata: id, text, source_url, section, chunk_index, token_count
- [ ] T028 [P] [US2] Create ingestion/embeddings/cohere_embedder.py: Implement CohereEmbedder class with embed(chunks) method
- [ ] T029 [US2] Implement Cohere API batching: up to COHERE_BATCH_SIZE (default 100) chunks per request
- [ ] T030 [US2] Implement error handling: rate limits (backoff), quota exceeded (pause and report), network errors (retry)
- [ ] T031 [US2] Implement embedding logging: total chunks, successful, failed, latency per batch
- [ ] T032 [P] [US2] Create backend/tests/unit/test_text_chunker.py: Unit tests with sample text (prose, code, mixed)
- [ ] T033 [US2] Create backend/tests/unit/test_cohere_embedder.py: Unit tests with mocked Cohere API responses
- [ ] T034 [US2] Update backend/main.py: Implement chunk_and_embed() function that wires chunker → embedder
- [ ] T035 [US2] Update backend/main.py: Add logging of chunking/embedding stats (total chunks, tokens, latency)
- [ ] T036 [P] [US2] Create backend/tests/integration/test_chunk_embed_integration.py: End-to-end test with extracted text

**Success Criteria - US2**:
- [ ] >95% of chunks within configured token limits (256–512)
- [ ] Chunk boundaries are logical (not mid-sentence or mid-code block)
- [ ] All chunks receive embeddings or failures are logged
- [ ] All tests pass: `pytest backend/tests/unit/test_text_chunker.py backend/tests/unit/test_cohere_embedder.py`
- [ ] Integration test passes: extract → chunk → embed real documentation

---

## Phase 5: User Story 3 - Store and Index Embeddings in Vector Database (P1)

**Goal**: Persist embeddings and metadata in Qdrant with efficient similarity search.
**Duration**: 1.5 hours
**Dependencies**: Phase 1, Phase 2, Phase 4 (for embeddings)
**Independent Test**: Store embeddings in Qdrant, execute similarity search queries, validate relevance.
**Parallel Opportunity**: Can start T037–T041 immediately after Phase 2 (no US2 dependency for unit tests with mocks).

### Checklist

- [ ] T037 [P] [US3] Create ingestion/storage/qdrant_store.py: Implement QdrantStore class with __init__, upsert(), search() methods
- [ ] T038 [US3] Implement Qdrant collection initialization: correct schema (vector dimension 1024, indexed metadata fields)
- [ ] T039 [US3] Implement batch upsert: insert embeddings with payload (text, url, section, position, token_count)
- [ ] T040 [US3] Implement similarity search: return top-k results with scores and metadata
- [ ] T041 [US3] Implement error handling: connection failures, collection errors, query failures with clear logging
- [ ] T042 [P] [US3] Create backend/tests/unit/test_qdrant_store.py: Unit tests with mocked Qdrant client
- [ ] T043 [US3] Update backend/main.py: Implement ingest_pipeline() function that wires crawler → chunker → embedder → storage
- [ ] T044 [US3] Update backend/main.py: Add logging of storage stats (total chunks upserted, query latencies)
- [ ] T045 [P] [US3] Create backend/tests/integration/test_qdrant_integration.py: End-to-end test with Qdrant Cloud/Docker

**Success Criteria - US3**:
- [ ] All embeddings successfully stored in Qdrant
- [ ] Similarity search returns top-k results within 100ms
- [ ] Returned results are topically relevant to queries
- [ ] All tests pass: `pytest backend/tests/unit/test_qdrant_store.py`
- [ ] Integration test passes: store real embeddings, execute queries

---

## Phase 6: User Story 4 - Validate End-to-End Pipeline (P2)

**Goal**: Validate full pipeline (crawl → chunk → embed → store) with test queries and reporting.
**Duration**: 1.5 hours
**Dependencies**: Phase 1–5 (all prior phases)
**Independent Test**: Run full pipeline on sample docs, execute test queries, verify results are relevant.

### Checklist

- [ ] T046 [US4] Update backend/main.py: Implement full main() function with config validation (Cohere key, Qdrant connectivity)
- [ ] T047 [US4] Implement prerequisite checks: Cohere API key validation, Qdrant connectivity test, target URL reachability
- [ ] T048 [US4] Implement top-level exception handler: log context (stage, URL, chunk ID), report summary stats, exit with proper code
- [ ] T049 [US4] Implement pipeline summary reporting: pages crawled, chunks created, embeddings generated, documents stored, failures
- [ ] T050 [US4] Update backend/main.py: Implement test_queries() function with pre-defined test queries (e.g., "How to install?", "Configuration")
- [ ] T051 [US4] Implement test query execution: generate embeddings for queries, search Qdrant, display top-3 results with scores
- [ ] T052 [US4] Implement CLI argument parsing: --crawl-extract, --chunk-embed, --ingest, --test-queries, --full-pipeline
- [ ] T053 [US4] Create backend/tests/integration/test_full_pipeline.py: End-to-end test with all stages, validates complete flow
- [ ] T054 [P] [US4] Create backend/tests/integration/test_queries.py: Test query execution, verify >80% of queries return relevant results

**Success Criteria - US4**:
- [ ] Full pipeline executes without unhandled exceptions
- [ ] Summary statistics are comprehensive and accurate
- [ ] Test queries return relevant results in top-5
- [ ] All prerequisites are validated before pipeline starts
- [ ] All tests pass: `pytest backend/tests/integration/test_full_pipeline.py backend/tests/integration/test_queries.py`
- [ ] Exit codes are correct (0 on success, non-zero on failure)

---

## Dependencies & Parallel Execution

### Critical Path (Sequential)

```
T001–T004 (Phase 1) → T005–T011 (Phase 2) → T012–T023 (Phase 3: US1)
                                         ├→ T024–T036 (Phase 4: US2) [Can start after Phase 2]
                                         ├→ T037–T045 (Phase 5: US3) [Can start after Phase 2]
                                         └→ T046–T054 (Phase 6: US4) [Requires Phases 3–5]
```

### Parallelization Opportunities

**After Phase 2 is complete**, you can work on US2, US3 in parallel:
- Task T024–T036 (US2 chunking & embedding) [P] tasks can run in parallel with P tasks from other US
- Task T037–T045 (US3 storage) [P] tasks can run in parallel with P tasks from other US

**Example parallel execution**:
```bash
# Terminal 1: US1 Phase 3
pytest backend/tests/unit/test_docusaurus_crawler.py
pytest backend/tests/unit/test_text_cleaner.py

# Terminal 2 (simultaneous): US2 unit tests
pytest backend/tests/unit/test_text_chunker.py

# Terminal 3 (simultaneous): US3 unit tests
pytest backend/tests/unit/test_qdrant_store.py
```

---

## Implementation Strategy

### MVP (Minimum Viable Product)
**Scope**: User Story 1 only (crawl & extract)
**Tasks**: T001–T023 (Phase 1–3)
**Duration**: ~2 hours
**Value**: Developers can crawl Docusaurus sites and extract clean text
**Deliverable**: `python backend/main.py --crawl-extract` works end-to-end

### Incremental Delivery
1. **Increment 1 (MVP)**: US1 complete (T001–T023)
2. **Increment 2**: US2 added (T024–T036), enables embedding generation
3. **Increment 3**: US3 added (T037–T045), full pipeline to vector storage
4. **Increment 4**: US4 added (T046–T054), end-to-end validation and test queries

---

## Task Execution Quick Reference

### By Phase

| Phase | Tasks | Duration | Blocking |
|-------|-------|----------|----------|
| Phase 1: Setup | T001–T004 | 30 min | Yes (blocks all) |
| Phase 2: Foundation | T005–T011 | 45 min | Yes (blocks US) |
| Phase 3: US1 | T012–T023 | 1.5 hrs | MVP |
| Phase 4: US2 | T024–T036 | 1.5 hrs | None (after Ph2) |
| Phase 5: US3 | T037–T045 | 1.5 hrs | None (after Ph2) |
| Phase 6: US4 | T046–T054 | 1.5 hrs | All prior |

### By User Story

| User Story | Tasks | Count | Duration |
|------------|-------|-------|----------|
| US1: Crawl | T012–T023 | 12 | 1.5 hrs |
| US2: Chunk | T024–T036 | 13 | 1.5 hrs |
| US3: Store | T037–T045 | 9 | 1.5 hrs |
| US4: Validate | T046–T054 | 9 | 1.5 hrs |

---

## File Structure After All Tasks

```
backend/
├── pyproject.toml
├── .env.example
├── .gitignore
├── README.md
├── main.py
│
├── ingestion/
│   ├── __init__.py
│   ├── models.py
│   ├── crawlers/
│   │   ├── __init__.py
│   │   └── docusaurus_crawler.py
│   ├── processors/
│   │   ├── __init__.py
│   │   └── text_cleaner.py
│   ├── chunking/
│   │   ├── __init__.py
│   │   └── text_chunker.py
│   ├── embeddings/
│   │   ├── __init__.py
│   │   └── cohere_embedder.py
│   └── storage/
│       ├── __init__.py
│       └── qdrant_store.py
│
├── utils/
│   ├── __init__.py
│   ├── config.py
│   ├── logging.py
│   └── errors.py
│
└── tests/
    ├── conftest.py
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

---

## Definition of Done (Per Task)

A task is complete when:
1. All acceptance criteria are marked complete
2. Unit tests pass (if applicable): `pytest`
3. Code is committed to feature branch
4. PHR recorded (for major tasks, typically Phase boundaries)
5. No unhandled exceptions or warnings

---

## Notes

- **Configuration**: All tasks use environment variables from `.env`; see Phase 1 (T002) for template
- **Testing Strategy**: Unit tests use mocks; integration tests use real (or Docker) Qdrant instance
- **Error Handling**: Each module catches and logs errors gracefully; pipeline continues unless critical prerequisite fails
- **Logging**: Structured JSON logging (see Phase 2, T006) for production debugging
- **Python Version**: 3.11+ (specified in pyproject.toml)
- **Dev Loop**: After Phase 2, can test individual modules with `pytest backend/tests/unit/` before integration
