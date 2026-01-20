"""RAG documentation ingestion pipeline orchestrator."""

import argparse
import logging
import sys
import time
from datetime import datetime

from ingestion.chunking.text_chunker import TextChunker
from ingestion.crawlers.docusaurus_crawler import DocusaurusCrawler
from ingestion.embeddings.cohere_embedder import CohereEmbedder
from ingestion.models import PipelineStats
from ingestion.processors.text_cleaner import TextCleaner
from ingestion.storage.qdrant_store import QdrantStore
from utils.config import load_config
from utils.errors import IngestionError
from utils.logging import setup_logging

logger = logging.getLogger(__name__)


class RAGPipeline:
    """Orchestrates the complete RAG ingestion pipeline."""

    def __init__(self, config):
        """Initialize the pipeline with configuration.

        Args:
            config: Configuration object from utils.config
        """
        self.config = config
        self.stats = PipelineStats()
        self.start_time = time.time()

    def validate_prerequisites(self) -> bool:
        """Validate that all prerequisites are met.

        Returns:
            True if all prerequisites are valid
        """
        logger.info("Validating prerequisites...")

        # Check Cohere API key
        if not self.config.cohere_api_key:
            logger.error("COHERE_API_KEY not set")
            return False

        # Check Qdrant configuration
        if not self.config.qdrant_url or not self.config.qdrant_api_key:
            logger.error("QDRANT_URL and QDRANT_API_KEY must be set")
            return False

        # Check target docs URL
        if not self.config.docs_url:
            logger.error("DOCS_URL not set")
            return False

        # Try to validate Qdrant connection
        try:
            store = QdrantStore(
                url=self.config.qdrant_url,
                api_key=self.config.qdrant_api_key,
                collection_name=self.config.qdrant_collection_name,
                vector_size=self.config.qdrant_vector_size,
                recreate_collection=self.config.qdrant_recreate_collection,
            )
            info = store.get_collection_info()
            logger.info(f"Qdrant collection info: {info}")
        except Exception as e:
            logger.error(f"Qdrant connection validation failed: {str(e)}")
            return False

        logger.info("Prerequisites validation passed")
        return True

    def crawl_and_extract(self) -> list:
        """Crawl documentation site and extract text.

        Returns:
            List of (url, cleaned_text, metadata) tuples
        """
        logger.info(f"Starting crawl of {self.config.docs_url}")

        try:
            crawler = DocusaurusCrawler(
                base_url=self.config.docs_url,
                max_pages=self.config.crawl_max_pages,
                timeout_seconds=self.config.crawl_timeout_seconds,
                follow_external_links=self.config.crawl_follow_external_links,
                crawl_delay_ms=self.config.crawl_delay_ms,
            )

            pages = crawler.crawl()
            self.stats.pages_crawled = len(pages)
            logger.info(f"Crawled {len(pages)} pages")

            # Extract text from each page
            cleaner = TextCleaner()
            extracted = []

            for url, html in pages:
                try:
                    text, metadata = cleaner.extract_text(html, url)
                    if text:
                        extracted.append((url, text, metadata))
                    else:
                        self.stats.pages_failed += 1
                except Exception as e:
                    logger.warning(f"Failed to extract text from {url}: {str(e)}")
                    self.stats.pages_failed += 1

            logger.info(f"Successfully extracted text from {len(extracted)} pages")
            return extracted

        except Exception as e:
            logger.error(f"Crawl and extract failed: {str(e)}")
            raise

    def chunk_and_embed(self, extracted_pages: list) -> list:
        """Chunk extracted text and generate embeddings.

        Args:
            extracted_pages: List of (url, text, metadata) tuples

        Returns:
            List of (chunk, embedding) tuples
        """
        logger.info(f"Chunking and embedding {len(extracted_pages)} pages")

        try:
            chunker = TextChunker(
                min_tokens=self.config.chunk_min_tokens,
                max_tokens=self.config.chunk_max_tokens,
                overlap_tokens=self.config.chunk_overlap_tokens,
            )

            embedder = CohereEmbedder(
                api_key=self.config.cohere_api_key,
                model=self.config.cohere_model,
                batch_size=self.config.cohere_batch_size,
            )

            all_chunks = []
            results = []

            # Chunk all pages
            for url, text, metadata in extracted_pages:
                try:
                    chunks = chunker.chunk(
                        text=text,
                        source_url=url,
                        section=metadata.get("title", ""),
                        metadata=metadata,
                    )
                    all_chunks.extend(chunks)
                    self.stats.chunks_created += len(chunks)
                except Exception as e:
                    logger.warning(f"Failed to chunk {url}: {str(e)}")
                    self.stats.chunks_failed += 1

            logger.info(f"Created {len(all_chunks)} chunks")

            # Generate embeddings
            embeddings = embedder.embed(all_chunks)
            self.stats.embeddings_generated = len(embeddings)

            # Pair chunks with embeddings
            for chunk, embedding in zip(all_chunks, embeddings):
                results.append((chunk, embedding))

            return results

        except Exception as e:
            logger.error(f"Chunk and embed failed: {str(e)}")
            raise

    def store_embeddings(self, chunk_embedding_pairs: list) -> int:
        """Store embeddings in Qdrant.

        Args:
            chunk_embedding_pairs: List of (chunk, embedding) tuples

        Returns:
            Number of embeddings stored
        """
        logger.info(f"Storing {len(chunk_embedding_pairs)} embeddings in Qdrant")

        try:
            store = QdrantStore(
                url=self.config.qdrant_url,
                api_key=self.config.qdrant_api_key,
                collection_name=self.config.qdrant_collection_name,
                vector_size=self.config.qdrant_vector_size,
            )

            embeddings = [embedding for _, embedding in chunk_embedding_pairs]
            count = store.upsert(embeddings)
            self.stats.chunks_stored = count

            logger.info(f"Stored {count} embeddings successfully")
            return count

        except Exception as e:
            logger.error(f"Storage failed: {str(e)}")
            self.stats.chunks_failed_storage = len(chunk_embedding_pairs)
            raise

    def run_full_pipeline(self):
        """Execute the complete ingestion pipeline."""
        logger.info("Starting full RAG ingestion pipeline")

        try:
            # Crawl and extract
            extracted = self.crawl_and_extract()
            if not extracted:
                logger.error("No pages extracted, aborting pipeline")
                return False

            # Chunk and embed
            chunk_embedding_pairs = self.chunk_and_embed(extracted)
            if not chunk_embedding_pairs:
                logger.error("No chunks created, aborting pipeline")
                return False

            # Store embeddings
            stored = self.store_embeddings(chunk_embedding_pairs)
            if stored == 0:
                logger.error("No embeddings stored")
                return False

            # Calculate statistics
            self.stats.total_duration_seconds = time.time() - self.start_time

            self._print_summary()
            return True

        except Exception as e:
            logger.error(f"Pipeline failed: {str(e)}")
            return False

    def test_queries(self, test_queries: list = None):
        """Execute test queries against the indexed embeddings.

        Args:
            test_queries: List of test query strings
        """
        if test_queries is None:
            test_queries = [
                "How to install?",
                "Configuration options",
                "Getting started",
                "API reference",
                "Troubleshooting",
            ]

        logger.info(f"Running {len(test_queries)} test queries")

        try:
            embedder = CohereEmbedder(
                api_key=self.config.cohere_api_key,
                model=self.config.cohere_model,
            )

            store = QdrantStore(
                url=self.config.qdrant_url,
                api_key=self.config.qdrant_api_key,
                collection_name=self.config.qdrant_collection_name,
                vector_size=self.config.qdrant_vector_size,
            )

            # Create simple chunks for embedding queries
            from ingestion.models import DocumentChunk

            for query in test_queries:
                logger.info(f"\nQuery: {query}")

                try:
                    # Embed the query
                    query_chunk = DocumentChunk(
                        id="query",
                        text=query,
                        source_url="",
                    )

                    query_embeddings = embedder.embed([query_chunk])
                    if not query_embeddings:
                        logger.warning(f"Failed to embed query: {query}")
                        continue

                    # Search
                    query_vector = query_embeddings[0].vector
                    results = store.search(query_vector, limit=3)

                    # Display results
                    for i, (chunk_id, score, payload) in enumerate(results, 1):
                        text = payload.get("text", "")[:100]
                        logger.info(f"  {i}. Score: {score:.4f}, Text: {text}...")

                except Exception as e:
                    logger.warning(f"Query failed: {str(e)}")

        except Exception as e:
            logger.error(f"Test queries failed: {str(e)}")

    def _print_summary(self):
        """Print pipeline execution summary."""
        logger.info("\n" + "=" * 60)
        logger.info("PIPELINE SUMMARY")
        logger.info("=" * 60)
        logger.info(f"Pages crawled: {self.stats.pages_crawled}")
        logger.info(f"Pages failed: {self.stats.pages_failed}")
        logger.info(f"Chunks created: {self.stats.chunks_created}")
        logger.info(f"Chunks failed: {self.stats.chunks_failed}")
        logger.info(f"Embeddings generated: {self.stats.embeddings_generated}")
        logger.info(f"Chunks stored: {self.stats.chunks_stored}")
        logger.info(f"Chunks storage failed: {self.stats.chunks_failed_storage}")
        logger.info(f"Total duration: {self.stats.total_duration_seconds:.2f}s")
        logger.info("=" * 60)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="RAG Documentation Ingestion Pipeline")
    parser.add_argument(
        "--crawl-extract",
        action="store_true",
        help="Run crawl and extraction only",
    )
    parser.add_argument(
        "--chunk-embed",
        action="store_true",
        help="Run chunking and embedding only",
    )
    parser.add_argument(
        "--full-pipeline",
        action="store_true",
        help="Run full pipeline (default)",
    )
    parser.add_argument(
        "--test-queries",
        action="store_true",
        help="Run test queries",
    )

    args = parser.parse_args()

    # Load configuration
    try:
        config = load_config()
    except ValueError as e:
        logger.error(f"Configuration error: {str(e)}")
        return 1

    # Setup logging
    setup_logging(config.log_level, config.log_format)

    # Create pipeline
    pipeline = RAGPipeline(config)

    # Validate prerequisites
    if not pipeline.validate_prerequisites():
        logger.error("Prerequisites validation failed")
        return 1

    # Run requested operations
    success = False

    if args.crawl_extract:
        try:
            pipeline.crawl_and_extract()
            success = True
        except Exception as e:
            logger.error(f"Crawl and extract failed: {str(e)}")

    elif args.chunk_embed:
        logger.warning("Chunk and embed requires input data (not implemented as standalone)")

    elif args.test_queries:
        try:
            pipeline.test_queries()
            success = True
        except Exception as e:
            logger.error(f"Test queries failed: {str(e)}")

    else:
        # Default: run full pipeline
        success = pipeline.run_full_pipeline()

    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
