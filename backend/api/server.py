"""
FastAPI server for RAG chatbot API.

Provides HTTP endpoints for querying the RAG system:
- POST /api/chat: Chat with the RAG chatbot
- GET /health: Health check endpoint
- GET /: Root endpoint with documentation link
"""

import logging
import os
from contextlib import asynccontextmanager

import anthropic
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from ingestion.embeddings.cohere_embedder import CohereEmbedder
from ingestion.storage.qdrant_store import QdrantStore
from rag.query_engine import RAGQueryEngine
from utils.config import Config
from utils.errors import CohereAPIError, CohereQuotaError

from api.models import ChatRequest, ChatResponse, HealthResponse, Message, Source

logger = logging.getLogger(__name__)

# Global variables for app lifecycle
config: Config | None = None
query_engine: RAGQueryEngine | None = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifecycle manager for FastAPI startup and shutdown."""
    # Startup
    global config, query_engine

    logger.info("Starting RAG Chat API server...")

    try:
        # Load configuration
        config = Config()
        logger.info(f"Configuration loaded successfully")

        # Initialize dependencies
        embedder = CohereEmbedder(config)
        store = QdrantStore(config)
        claude_client = anthropic.Anthropic(api_key=config.claude_api_key)

        # Initialize query engine
        query_engine = RAGQueryEngine(config, embedder, store, claude_client)
        logger.info("RAG Query Engine initialized")

    except Exception as e:
        logger.error(f"Failed to initialize server: {str(e)}")
        raise

    yield

    # Shutdown (cleanup if needed)
    logger.info("Shutting down RAG Chat API server...")


def create_app() -> FastAPI:
    """Create and configure the FastAPI application."""
    app = FastAPI(
        title="RAG Chat API",
        description="RAG-powered chat API for documentation Q&A",
        version="1.0.0",
        lifespan=lifespan,
    )

    # Configure CORS
    cors_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
    cors_origins = [origin.strip() for origin in cors_origins]

    logger.info(f"Configuring CORS for origins: {cors_origins}")

    app.add_middleware(
        CORSMiddleware,
        allow_origins=cors_origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Routes
    @app.get("/")
    async def root():
        """Root endpoint with API documentation link."""
        return {
            "message": "RAG Chat API",
            "docs": "/docs",
            "openapi": "/openapi.json",
        }

    @app.get("/health", response_model=HealthResponse)
    async def health():
        """Health check endpoint."""
        if query_engine is None:
            raise HTTPException(status_code=503, detail="Query engine not initialized")
        return HealthResponse(status="ok", version="1.0.0")

    @app.post("/api/chat", response_model=ChatResponse)
    async def chat(request: ChatRequest):
        """
        Chat endpoint for RAG queries.

        Accepts a user query and optional conversation history, returns an AI-generated
        response with source citations and confidence score.
        """
        if query_engine is None:
            raise HTTPException(status_code=503, detail="Query engine not initialized")

        try:
            logger.info(f"Received chat request (session: {request.session_id})")

            # Process the query
            response = query_engine.query(request.query)

            # Convert RetrievedChunk objects to Source objects
            sources = [
                Source(
                    text=chunk.text,
                    url=chunk.source_url,
                    section=chunk.section,
                )
                for chunk in response.sources
            ]

            # Return response
            return ChatResponse(
                answer=response.answer,
                sources=sources,
                confidence=response.confidence,
                query=request.query,
            )

        except CohereQuotaError:
            logger.error("Cohere API quota exceeded")
            raise HTTPException(status_code=429, detail="API quota exceeded. Please try again later.")
        except CohereAPIError as e:
            logger.error(f"Cohere API error: {str(e)}")
            raise HTTPException(status_code=500, detail="Embedding service error")
        except anthropic.APIError as e:
            logger.error(f"Claude API error: {str(e)}")
            raise HTTPException(status_code=500, detail="Response generation error")
        except Exception as e:
            logger.error(f"Unexpected error in chat endpoint: {str(e)}")
            raise HTTPException(status_code=500, detail="Internal server error")

    return app


# Create the app instance
app = create_app()

if __name__ == "__main__":
    import uvicorn

    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    # Run the server
    host = os.getenv("API_HOST", "0.0.0.0")
    port = int(os.getenv("API_PORT", 8000))

    logger.info(f"Starting server on {host}:{port}")
    uvicorn.run(app, host=host, port=port)
