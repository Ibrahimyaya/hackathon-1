"""Qdrant vector database storage and retrieval."""

import logging
from typing import List, Tuple

from qdrant_client import QdrantClient
from qdrant_client.http import models

from ingestion.models import Embedding
from utils.errors import QdrantCollectionError, QdrantConnectionError, QdrantQueryError

logger = logging.getLogger(__name__)


class QdrantStore:
    """Manages vector storage and similarity search in Qdrant."""

    def __init__(
        self,
        url: str,
        api_key: str,
        collection_name: str = "docs_chunks",
        vector_size: int = 1024,
        recreate_collection: bool = False,
    ):
        """Initialize the Qdrant store.

        Args:
            url: Qdrant endpoint URL
            api_key: API key for authentication
            collection_name: Name of the collection
            vector_size: Dimension of vectors
            recreate_collection: Whether to recreate collection on init

        Raises:
            QdrantConnectionError: If connection fails
        """
        try:
            self.client = QdrantClient(url=url, api_key=api_key)
            self.collection_name = collection_name
            self.vector_size = vector_size

            # Initialize collection
            self._init_collection(recreate_collection)

            logger.info(f"Connected to Qdrant at {url}")

        except Exception as e:
            raise QdrantConnectionError(f"Failed to connect to Qdrant: {str(e)}") from e

    def _init_collection(self, recreate: bool = False):
        """Initialize or verify collection exists.

        Args:
            recreate: Whether to recreate the collection

        Raises:
            QdrantCollectionError: If initialization fails
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name in collection_names:
                if recreate:
                    logger.warning(f"Recreating collection {self.collection_name}")
                    self.client.delete_collection(self.collection_name)
                else:
                    logger.info(f"Collection {self.collection_name} already exists")
                    return

            # Create collection
            logger.info(f"Creating collection {self.collection_name} with vector size {self.vector_size}")

            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.vector_size,
                    distance=models.Distance.COSINE,
                ),
            )

            logger.info(f"Collection {self.collection_name} created successfully")

        except Exception as e:
            raise QdrantCollectionError(f"Failed to initialize collection: {str(e)}") from e

    def upsert(self, embeddings: List[Embedding], chunk_metadata: dict = None) -> int:
        """Upsert embeddings into the collection.

        Args:
            embeddings: List of Embedding objects
            chunk_metadata: Metadata associated with chunks

        Returns:
            Number of embeddings upserted

        Raises:
            QdrantConnectionError: If connection fails
            QdrantCollectionError: If upsert fails
        """
        if not embeddings:
            return 0

        if chunk_metadata is None:
            chunk_metadata = {}

        try:
            points = []

            for i, embedding in enumerate(embeddings):
                # Parse chunk_id to extract metadata
                chunk_id_parts = embedding.chunk_id.split(":")
                chunk_id = embedding.chunk_id

                # Create point with payload
                point = models.PointStruct(
                    id=i,  # Use index as ID (Qdrant will auto-assign if needed)
                    vector=embedding.vector,
                    payload={
                        "chunk_id": chunk_id,
                        "text": chunk_metadata.get("text", ""),
                        "source_url": chunk_metadata.get("source_url", ""),
                        "section": chunk_metadata.get("section", ""),
                        "chunk_index": chunk_metadata.get("chunk_index", 0),
                        "token_count": chunk_metadata.get("token_count", 0),
                    },
                )
                points.append(point)

            # Upsert points
            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )

            logger.info(f"Upserted {len(embeddings)} embeddings to {self.collection_name}")

            return len(embeddings)

        except Exception as e:
            raise QdrantCollectionError(f"Failed to upsert embeddings: {str(e)}") from e

    def search(
        self, query_vector: List[float], limit: int = 10, score_threshold: float = 0.0
    ) -> List[Tuple[str, float, dict]]:
        """Search for similar vectors.

        Args:
            query_vector: Query vector
            limit: Number of results to return
            score_threshold: Minimum score threshold

        Returns:
            List of tuples (chunk_id, score, payload)

        Raises:
            QdrantConnectionError: If connection fails
            QdrantQueryError: If query fails
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                score_threshold=score_threshold,
            )

            # Convert results to list of tuples
            search_results = []
            for result in results:
                chunk_id = result.payload.get("chunk_id", "")
                score = result.score
                payload = result.payload

                search_results.append((chunk_id, score, payload))

            logger.debug(f"Search returned {len(search_results)} results")

            return search_results

        except Exception as e:
            raise QdrantQueryError(f"Search query failed: {str(e)}") from e

    def get_collection_info(self) -> dict:
        """Get collection information.

        Returns:
            Dictionary with collection metadata

        Raises:
            QdrantConnectionError: If query fails
        """
        try:
            info = self.client.get_collection(self.collection_name)

            return {
                "name": info.name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "vector_size": self.vector_size,
            }

        except Exception as e:
            raise QdrantConnectionError(f"Failed to get collection info: {str(e)}") from e
