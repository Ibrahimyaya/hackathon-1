# Chat Widget Implementation Guide

This document provides setup and deployment instructions for the RAG Chat Widget feature.

## Overview

The chat widget adds a floating chat bubble to the ROS 2 documentation site that connects to the RAG backend for documentation Q&A. The implementation is complete and ready for testing and deployment.

## Files Created

### Backend (FastAPI)
- `backend/api/__init__.py` - API package initialization
- `backend/api/models.py` - Pydantic request/response models
- `backend/api/server.py` - FastAPI application with endpoints
- `backend/Dockerfile` - Multi-stage Docker build
- `backend/start.sh` - Production startup script
- `backend/railway.toml` - Railway deployment configuration
- `backend/render.yaml` - Render deployment configuration

### Frontend (React)
- `src/components/ChatWidget/index.tsx` - Main chat widget component
- `src/components/ChatWidget/ChatBubble.tsx` - Floating chat bubble
- `src/components/ChatWidget/ChatWindow.tsx` - Chat window interface
- `src/components/ChatWidget/MessageList.tsx` - Message display component
- `src/css/components/chat-widget.css` - Complete styling system

### Configuration Updates
- `backend/pyproject.toml` - Added FastAPI dependencies
- `backend/.env.example` - Added API configuration options
- `docusaurus.config.js` - Added custom fields for chat API URL
- `src/css/custom.css` - Imported chat widget styles
- `src/swizzled/Layout/index.tsx` - Integrated ChatWidget component

## API Specification

### Endpoints

#### POST /api/chat
Send a message to the RAG chatbot and receive a response with sources and confidence score.

**Request:**
```json
{
  "query": "What is ROS 2?",
  "session_id": "session-unique-id",
  "history": [
    {
      "role": "user",
      "content": "Previous question"
    },
    {
      "role": "assistant",
      "content": "Previous answer"
    }
  ]
}
```

**Response (200 OK):**
```json
{
  "answer": "ROS 2 is...",
  "sources": [
    {
      "text": "excerpt from documentation",
      "url": "https://docs.example.com/section",
      "section": "Section Title"
    }
  ],
  "confidence": 0.85,
  "query": "What is ROS 2?"
}
```

**Error Responses:**
- 429: API quota exceeded (Cohere limit)
- 500: Server error (embedding or generation failure)
- 503: Service unavailable (engine not initialized)

#### GET /health
Health check endpoint for monitoring.

**Response (200 OK):**
```json
{
  "status": "ok",
  "version": "1.0.0"
}
```

#### GET /
Root endpoint with links to documentation.

## Local Development Setup

### 1. Install Dependencies

```bash
# Backend
cd backend
pip install -e ".[dev]"

# Frontend
cd ..
npm install
```

### 2. Environment Configuration

Ensure your `.env` file includes:

```bash
# Backend API
API_HOST=0.0.0.0
API_PORT=8000
CORS_ORIGINS=http://localhost:3000

# Frontend
REACT_APP_CHAT_API_URL=http://localhost:8000

# All existing RAG configuration (Cohere, Qdrant, Claude, etc.)
```

### 3. Start the Backend API

```bash
# Terminal 1: Start backend
cd backend
python -m uvicorn api.server:app --reload --host 0.0.0.0 --port 8000

# Expected output:
# INFO:     Uvicorn running on http://0.0.0.0:8000
# INFO:     Application startup complete
```

### 4. Start the Frontend

```bash
# Terminal 2: Start Docusaurus dev server
npm start

# Expected output:
# [INFO] Starting the development server...
# [SUCCESS] Docusaurus server started on http://localhost:3000/docs
```

### 5. Test the Chat Widget

1. Open browser: http://localhost:3000/docs
2. Click the floating chat bubble (bottom-right corner)
3. Type a question: "What is ROS 2?"
4. Verify the response appears with sources and confidence score
5. Test mobile responsiveness (F12 → toggle device toolbar)

## Deployment

### Option A: Deploy to Railway

1. **Create Railway Project**
   - Sign up at https://railway.app/
   - Create new project
   - Connect GitHub repository

2. **Configure Environment Variables in Railway Dashboard**
   ```
   CORS_ORIGINS=https://final-sandy-psi.vercel.app
   COHERE_API_KEY=<your-key>
   QDRANT_API_KEY=<your-key>
   QDRANT_URL=<your-url>
   CLAUDE_API_KEY=<your-key>
   DOCS_URL=https://final-sandy-psi.vercel.app/docs
   API_HOST=0.0.0.0
   API_PORT=8000
   LOG_LEVEL=INFO
   LOG_FORMAT=json
   ```

3. **Deploy**
   - Railway will automatically detect and deploy based on `Dockerfile`
   - Get deployment URL: `https://<project-name>.railway.app`

### Option B: Deploy to Render

1. **Create Render Service**
   - Sign up at https://render.com/
   - Create new Web Service
   - Connect GitHub repository
   - Select branch: `001-docs-rag-ingest`

2. **Configure Build & Runtime**
   - Build command: `pip install -e ./backend && pip install -e ./backend[dev]`
   - Start command: `bash backend/start.sh`
   - Root directory: `/backend`

3. **Set Environment Variables**
   - Same as Railway (see above)

4. **Deploy**
   - Click "Deploy"
   - Get deployment URL: `https://<service-name>.onrender.com`

### Option C: Deploy Frontend to Vercel

1. **Update Environment Variable**
   In Vercel dashboard, add:
   ```
   REACT_APP_CHAT_API_URL=https://<backend-url>
   ```

2. **Trigger Deployment**
   - Push to main branch or redeploy from Vercel dashboard
   - Verify chat widget loads on https://final-sandy-psi.vercel.app/docs

## Testing Checklist

### Backend Tests
```bash
# Run unit tests
cd backend
pytest tests/unit/ -v

# Run integration tests
pytest tests/integration/ -v

# Test API health check
curl http://localhost:8000/health

# Test chat endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?", "session_id": "test", "history": []}'
```

### Frontend Tests
- [ ] Chat bubble appears on all documentation pages
- [ ] Clicking bubble expands chat window smoothly
- [ ] Can type messages and submit
- [ ] Receives response with sources
- [ ] Confidence score displays correctly
- [ ] Error messages display properly
- [ ] Typing indicators appear while loading
- [ ] Chat history persists in session
- [ ] Clear chat button works
- [ ] Mobile layout (< 768px) is full-screen
- [ ] Dark theme displays correctly
- [ ] Light theme displays correctly
- [ ] Source links are clickable
- [ ] Source excerpts appear correctly

### Performance Tests
- [ ] First response: < 3 seconds
- [ ] Subsequent responses: < 2 seconds
- [ ] Widget load impact: < 100ms
- [ ] No console errors or warnings
- [ ] Memory usage stable (no leaks)

### E2E Tests (Production)
1. Visit https://final-sandy-psi.vercel.app/docs
2. Test 5 different questions
3. Verify responses match documentation
4. Test on mobile device
5. Test with different browsers (Chrome, Firefox, Safari)

## Architecture Decisions

### FastAPI vs Other Frameworks
- ✅ Async support for better concurrency
- ✅ Automatic OpenAPI documentation (/docs)
- ✅ Pydantic validation built-in
- ✅ Lightweight and fast
- ✅ Easy CORS configuration

### Session Management
- Stateless API design (no server-side state)
- Session ID stored in localStorage
- Conversation history sent with each request
- Allows easy scaling and deployment

### Styling Approach
- CSS variables for theme support (light/dark)
- Mobile-first responsive design
- Accessibility built-in (ARIA, focus states)
- No external CSS framework (pure CSS)

### Error Handling
- Graceful degradation (shows error messages)
- Retry logic in API client
- Rate limit handling (429 response)
- Service unavailable handling (503 response)

## Security Considerations

### CORS Configuration
- Only allows Vercel domain + localhost (development)
- Update `CORS_ORIGINS` for production deployment
- Prevent unauthorized API access from other domains

### API Keys
- All API keys loaded from environment variables
- Never hardcoded in source code
- Railway/Render dashboard for secure storage

### Input Validation
- Pydantic validates all request data
- Query length validation (min 1 character)
- Session ID format validation

### Rate Limiting
- Cohere API rate limiting handled with exponential backoff
- Future: Implement API-level rate limiting per IP

## Troubleshooting

### Chat Widget Not Appearing
1. Check browser console for JavaScript errors
2. Verify `src/components/ChatWidget/` files exist
3. Verify `chat-widget.css` is imported in `custom.css`
4. Clear browser cache and reload

### API Connection Fails
1. Check that backend is running: `curl http://localhost:8000/health`
2. Verify `REACT_APP_CHAT_API_URL` is set correctly
3. Check CORS_ORIGINS includes frontend domain
4. Check browser console for CORS errors

### Responses Take Too Long
1. Check Qdrant connectivity and query performance
2. Check Cohere API latency
3. Check Claude API response time
4. Monitor backend logs for bottlenecks

### Responses Are Inaccurate
1. Check documentation was indexed correctly
2. Verify Qdrant has documents: Check collection stats
3. Test retrieval manually:
   ```bash
   cd backend
   python chat.py
   # Ask test questions
   ```

## Future Enhancements

- [ ] Analytics dashboard (track popular questions)
- [ ] Feedback mechanism (thumbs up/down on responses)
- [ ] Conversation export (download chat history)
- [ ] Admin dashboard (monitor usage, adjust parameters)
- [ ] Multi-language support
- [ ] Voice input/output
- [ ] Streaming responses (progressively show answer)
- [ ] Follow-up question suggestions
- [ ] User authentication (optional)

## Monitoring & Maintenance

### Logs
- Backend: Structured JSON logging to stdout
- Frontend: Browser console logs and error reporting

### Metrics to Monitor
- API response time (p50, p95, p99)
- Error rate (5xx, 429, 503)
- Cohere API quota usage
- Qdrant collection size
- User engagement (questions asked, response quality)

### Health Checks
- Automated: `/health` endpoint checked every 30 seconds
- Manual: `curl http://<api-url>/health`

## Support & Issues

For issues or questions:
1. Check this guide's Troubleshooting section
2. Review backend logs: `docker logs <container-id>`
3. Review browser console (F12 → Console tab)
4. Create GitHub issue with:
   - Error message and stack trace
   - Steps to reproduce
   - Environment (OS, browser, backend URL)
   - Recent API responses (sanitized)

## Version History

### v1.0.0 (2026-01-26)
- Initial implementation
- FastAPI backend with `/api/chat` endpoint
- React chat widget with floating bubble
- Full responsive design (mobile, tablet, desktop)
- Light/dark theme support
- Source citations and confidence scoring
- Deployment configs for Railway and Render
