#!/bin/bash
# Start script for RAG Chat API server

set -e

# Set defaults
API_HOST="${API_HOST:-0.0.0.0}"
API_PORT="${API_PORT:-8000}"

echo "Starting RAG Chat API Server..."
echo "  Host: $API_HOST"
echo "  Port: $API_PORT"

# Run uvicorn server
exec python -m uvicorn api.server:app \
    --host "$API_HOST" \
    --port "$API_PORT" \
    --workers 1 \
    --log-level info
