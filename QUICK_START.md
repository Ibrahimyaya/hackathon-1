# Quick Start: Web Chat Widget

## What Was Implemented

The complete web chat widget feature has been implemented and is ready for testing and deployment.

### Summary of Changes

| Category | Details |
|----------|---------|
| **Backend API** | FastAPI server with `/api/chat` endpoint |
| **Frontend Widget** | React chat bubble + window component |
| **Styling** | 700+ lines of responsive CSS |
| **Configuration** | Environment variables, deployment configs |
| **Documentation** | Setup guide, API spec, deployment instructions |

### Files Created (11)
```
backend/api/models.py (40 lines)
backend/api/server.py (160 lines)
backend/api/__init__.py (5 lines)
backend/Dockerfile (35 lines)
backend/start.sh (20 lines)
backend/railway.toml (30 lines)
backend/render.yaml (35 lines)
src/components/ChatWidget/index.tsx (150 lines)
src/components/ChatWidget/ChatBubble.tsx (50 lines)
src/components/ChatWidget/ChatWindow.tsx (110 lines)
src/components/ChatWidget/MessageList.tsx (95 lines)
src/css/components/chat-widget.css (700 lines)
CHAT_WIDGET_SETUP.md (400 lines)
QUICK_START.md (this file)
```

### Files Modified (7)
```
backend/pyproject.toml (added 3 dependencies)
backend/.env.example (added API config section)
src/css/custom.css (added import)
src/swizzled/Layout/index.tsx (added ChatWidget)
docusaurus.config.js (added customFields)
.env (added API variables)
history/prompts/001-docs-rag-ingest/7-implement-chat-widget.green.prompt.md (PHR)
```

## Test Locally (5 minutes)

### 1. Install & Start Backend
```bash
cd backend
pip install -e ".[dev]"
python -m uvicorn api.server:app --reload --port 8000
# Expected: Uvicorn running on http://0.0.0.0:8000
```

### 2. Start Frontend (new terminal)
```bash
npm install  # if not already installed
npm start
# Expected: Docusaurus on http://localhost:3000/docs
```

### 3. Test the Widget
- Open http://localhost:3000/docs
- Click the chat bubble (bottom-right corner)
- Type: "What is ROS 2?"
- Verify response appears with sources
- Test mobile view (F12 → toggle device)

### 4. Test API Directly
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "session_id": "test-session",
    "history": []
  }'
```

## Deploy to Production (30 minutes)

### Option A: Deploy Backend to Railway

1. **Create Project**: https://railway.app/new
2. **Connect GitHub**: Select repo and branch `001-docs-rag-ingest`
3. **Add Environment Variables** (in Railway dashboard):
   ```
   CORS_ORIGINS=https://final-sandy-psi.vercel.app
   COHERE_API_KEY=<your-key>
   QDRANT_API_KEY=<your-key>
   QDRANT_URL=<your-url>
   CLAUDE_API_KEY=<your-key>
   DOCS_URL=https://final-sandy-psi.vercel.app/docs
   ```
4. **Get URL**: `https://<project-name>.railway.app`

### Option B: Deploy Backend to Render

1. **Create Service**: https://render.com/new/web-service
2. **Connect GitHub**: Select repo and branch
3. **Configure**:
   - Root: `/backend`
   - Start: `bash start.sh`
4. **Add Environment Variables** (same as Railway)
5. **Get URL**: `https://<service-name>.onrender.com`

### Option C: Update Frontend on Vercel

1. **Vercel Dashboard**: Add environment variable:
   ```
   REACT_APP_CHAT_API_URL=https://<backend-url>
   ```
2. **Trigger Deploy**: Push to main or redeploy manually

## Verification Checklist

### Backend ✅
- [ ] API running: `curl http://localhost:8000/health` returns 200
- [ ] POST /api/chat returns JSON response with answer, sources, confidence
- [ ] CORS headers present for frontend domain
- [ ] Error handling works (invalid request → 400, server error → 500)

### Frontend ✅
- [ ] Chat bubble visible on all docs pages
- [ ] Click bubble → expands chat window
- [ ] Type question → receive response
- [ ] Response includes sources with links
- [ ] Confidence score displays
- [ ] Mobile layout (< 768px) is full-screen
- [ ] Dark theme works

### E2E (Production) ✅
- [ ] Visit https://final-sandy-psi.vercel.app/docs
- [ ] Chat bubble appears
- [ ] Send "What is ROS 2?" → receive response
- [ ] Click source link → navigates to docs
- [ ] Test on mobile device
- [ ] Check browser console (no errors)

## Key Features Implemented

### Floating Chat Bubble
- 60px diameter circle (bottom-right)
- Pulsing badge for unread messages
- Tooltip on hover
- Smooth expand/collapse animation

### Chat Window
- 350×500px on desktop (responsive)
- Full-screen on mobile (< 768px)
- Header with title and close button
- Message history with timestamps
- Typing indicators
- Error display
- Clear history button

### Messages
- User messages: right-aligned, blue background
- Assistant messages: left-aligned, gray background
- Emoji avatars (👤 for user, 🤖 for assistant)
- Word wrap and scrolling
- Sources expandable details
- Confidence score with color coding (green/yellow/red)

### Responsive Design
- Desktop (1024px+): 350×500px window
- Tablet (768px-1023px): 320×450px window
- Mobile (<768px): Full-screen overlay

### Accessibility
- ARIA labels on all buttons
- Keyboard navigation support
- Focus indicators
- High contrast support
- Reduced motion support
- Semantic HTML

## Architecture Highlights

### Backend
- **Framework**: FastAPI (async, lightweight)
- **Validation**: Pydantic models
- **CORS**: Configurable origins
- **Error Handling**: Graceful with HTTP status codes
- **Integration**: Existing RAGQueryEngine
- **Deployment**: Docker multi-stage build

### Frontend
- **Framework**: React with TypeScript
- **State**: useState, useEffect (no external state library needed)
- **Styling**: Pure CSS (no framework, full control)
- **API**: fetch() with error handling
- **Session**: localStorage for session ID
- **Responsive**: Mobile-first CSS approach

### Database & Services
- **Vector DB**: Qdrant (existing)
- **Embeddings**: Cohere API (existing)
- **LLM**: Claude API (existing)

## Performance Metrics

### Expected Performance
- First response: 2-3 seconds (includes embedding + retrieval + generation)
- Subsequent responses: 1-2 seconds
- Widget load: <100ms (lazy loaded with chat bubble)
- Bundle size: ~15KB gzipped (CSS + TypeScript)

### Optimizations
- Async API calls (non-blocking)
- CSS animations use GPU (transform, opacity)
- Lazy image loading (avatars are emoji, no images)
- Debounced input validation
- Efficient message re-rendering

## Security

### CORS
- Only Vercel domain + localhost allowed
- Configure in `CORS_ORIGINS` environment variable
- No cross-origin credentials sent

### API Keys
- All in environment variables
- Never logged or exposed
- Validated at startup
- Graceful error if missing

### Input Validation
- Query min 1 character
- Session ID format validated
- Pydantic enforces all type checks

## Troubleshooting

### Chat bubble not showing?
1. Check browser console (F12)
2. Verify `src/components/ChatWidget/` exists
3. Clear cache: Ctrl+Shift+Del → clear all

### API returns 503?
1. Check backend is running: `curl http://localhost:8000/health`
2. Verify all API keys are set
3. Check Qdrant/Cohere/Claude connectivity

### Slow responses?
1. Check network tab (F12) for latency
2. Verify Qdrant has indexed documents
3. Check API keys have quota remaining

### Styling looks broken?
1. Hard refresh: Ctrl+Shift+R
2. Verify `src/css/components/chat-widget.css` exists
3. Check custom.css imports chat-widget.css

## Next Steps

### Immediate (Day 1)
- [ ] Test locally with `npm start` + backend
- [ ] Verify all features work
- [ ] Run curl tests against API

### Short-term (Week 1)
- [ ] Deploy backend to Railway/Render
- [ ] Update Vercel environment variable
- [ ] Test on production site
- [ ] Monitor error logs

### Medium-term (Week 2-4)
- [ ] Collect user feedback
- [ ] Monitor performance metrics
- [ ] Analyze question patterns
- [ ] Consider enhancements

## Resources

- **Setup Guide**: See `CHAT_WIDGET_SETUP.md` for detailed instructions
- **API Documentation**: Automatic at http://localhost:8000/docs (FastAPI Swagger)
- **GitHub Issues**: Report bugs at project repo

## Support

For issues:
1. Check troubleshooting section above
2. Review browser console (F12 → Console)
3. Review backend logs: `docker logs <container>`
4. Check the `CHAT_WIDGET_SETUP.md` detailed guide

---

**Status**: ✅ Implementation Complete | Ready for Testing | Ready for Deployment
