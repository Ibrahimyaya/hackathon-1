# Deployment Checklist: Web Chat Widget

Use this checklist to guide deployment to production.

## Pre-Deployment Verification

- [ ] All code committed to `001-docs-rag-ingest` branch
- [ ] No API keys in code (all in .env)
- [ ] No hardcoded URLs (using environment variables)
- [ ] All tests pass locally
- [ ] Chat widget works on localhost
- [ ] No console errors in browser
- [ ] No Python syntax errors (`python -m py_compile`)

### Code Quality

- [ ] Python code: `black backend/api/`
- [ ] Python linting: `ruff check backend/api/`
- [ ] TypeScript: Review for type safety
- [ ] CSS: No hardcoded colors (using variables)
- [ ] Comments added for complex logic

### Testing

- [ ] Backend API tests: `pytest tests/`
- [ ] Manual API test: `curl http://localhost:8000/api/chat`
- [ ] Health check: `curl http://localhost:8000/health`
- [ ] Chat widget renders without errors
- [ ] Send 3+ test questions and verify responses
- [ ] Test on mobile (< 768px)
- [ ] Test on tablet (768px-1023px)
- [ ] Test on desktop (1024px+)
- [ ] Light theme: works correctly
- [ ] Dark theme: works correctly

## Backend Deployment (Railway)

### Step 1: Create Railway Project

- [ ] Go to https://railway.app/
- [ ] Click "Start a New Project"
- [ ] Select "Deploy from GitHub"
- [ ] Connect your GitHub account
- [ ] Select the repository

### Step 2: Configure Project

- [ ] Select branch: `001-docs-rag-ingest`
- [ ] Root directory: `/backend` (optional, Railway auto-detects)
- [ ] Environment: Production
- [ ] Keep CI/CD enabled for auto-deploys

### Step 3: Set Environment Variables

In Railway dashboard, add these variables:

| Variable | Value |
|----------|-------|
| `CORS_ORIGINS` | `https://final-sandy-psi.vercel.app` |
| `COHERE_API_KEY` | Get from Cohere dashboard |
| `QDRANT_API_KEY` | Get from Qdrant Cloud dashboard |
| `QDRANT_URL` | e.g., `https://cluster-name.qdrant.io:6333` |
| `CLAUDE_API_KEY` | Get from Anthropic console |
| `DOCS_URL` | `https://final-sandy-psi.vercel.app/docs` |
| `API_HOST` | `0.0.0.0` |
| `API_PORT` | `8000` |
| `LOG_LEVEL` | `INFO` |
| `LOG_FORMAT` | `json` |

- [ ] All 10 variables added
- [ ] No typos in keys
- [ ] Values are actual keys (not placeholders)
- [ ] Save variables

### Step 4: Deploy

- [ ] Click "Deploy" button
- [ ] Wait for build to complete (5-10 minutes)
- [ ] Check deployment logs for errors
- [ ] Verify no build failures
- [ ] Get deployment URL: `https://<project-name>.railway.app`

### Step 5: Verify Backend

- [ ] Health check: `curl https://<url>/health`
- [ ] Expected response: `{"status":"ok","version":"1.0.0"}`
- [ ] API docs: Visit `https://<url>/docs` (Swagger UI)
- [ ] Test chat endpoint with curl:
  ```bash
  curl -X POST https://<url>/api/chat \
    -H "Content-Type: application/json" \
    -d '{"query":"What is ROS 2?","session_id":"test","history":[]}'
  ```
- [ ] Response includes answer, sources, confidence
- [ ] No CORS errors in browser console

## Frontend Deployment (Vercel)

### Step 1: Update Environment Variable

- [ ] Go to Vercel project settings
- [ ] Go to "Environment Variables"
- [ ] Add new variable:
  ```
  REACT_APP_CHAT_API_URL=https://<backend-url>
  ```
  (Use the Railway URL from previous step)
- [ ] Save

### Step 2: Deploy

- [ ] Go to "Deployments" tab
- [ ] Click "Redeploy" on latest production deployment
- [ ] Wait for build (2-3 minutes)
- [ ] Verify deployment succeeds
- [ ] No build errors

### Step 3: Verify Frontend

- [ ] Visit https://final-sandy-psi.vercel.app/docs
- [ ] Chat bubble visible (bottom-right)
- [ ] Click bubble → expands chat window
- [ ] Type test question
- [ ] Receive response with sources
- [ ] No console errors (F12 → Console)
- [ ] CORS headers present in Network tab

## End-to-End Testing (Production)

### Functional Testing

- [ ] Test 1: "What is ROS 2?"
  - [ ] Response received
  - [ ] Sources displayed
  - [ ] Links are clickable
  - [ ] Confidence score shown

- [ ] Test 2: "How do I use nodes in ROS 2?"
  - [ ] Response received
  - [ ] Response is accurate
  - [ ] Sources relate to topics

- [ ] Test 3: "What is a sensor in URDF?"
  - [ ] Response received
  - [ ] Handles domain knowledge
  - [ ] Sources are relevant

### UI Testing

- [ ] Chat bubble visible on:
  - [ ] Home page
  - [ ] Chapter 1 pages
  - [ ] Chapter 2 pages
  - [ ] Chapter 3 pages
  - [ ] All other documentation pages

- [ ] Chat window responsive:
  - [ ] Desktop (1440px wide): 350×500px window
  - [ ] Tablet (768px wide): 320×450px window
  - [ ] Mobile (375px wide): Full-screen overlay
  - [ ] Mobile: Keyboard doesn't push content

- [ ] Interactions work:
  - [ ] Type message → keyboard works
  - [ ] Click send → message sent
  - [ ] Click close → window closes
  - [ ] Click bubble → opens again
  - [ ] Clear button → clears history

### Performance Testing

- [ ] Page load time: No significant increase
- [ ] Chat widget loads: < 100ms
- [ ] First response: < 3 seconds
- [ ] Subsequent responses: < 2 seconds
- [ ] No memory leaks: Performance tab stable
- [ ] No console errors over 5 messages

### Browser Testing

- [ ] Chrome (desktop)
- [ ] Firefox (desktop)
- [ ] Safari (desktop)
- [ ] Chrome (mobile)
- [ ] Safari (mobile)
- [ ] Edge (desktop)

### Accessibility Testing

- [ ] Tab navigation works
- [ ] Focus indicators visible
- [ ] Screen reader announces buttons
- [ ] High contrast mode works
- [ ] Reduced motion respected
- [ ] Color not only indicator (icons too)

### Error Testing

- [ ] Server down (stop backend):
  - [ ] Error message appears
  - [ ] Doesn't crash page
  - [ ] Can still browse docs

- [ ] Network error (throttle connection):
  - [ ] Loading indicator shows
  - [ ] Timeout after reasonable time
  - [ ] Error message appears

- [ ] Invalid input (empty message):
  - [ ] Send button disabled
  - [ ] No error thrown

## Monitoring

### Day 1 (After Deployment)

- [ ] Check backend logs for errors
- [ ] Monitor error rates (target: < 1%)
- [ ] Check response times (target: < 2s)
- [ ] Monitor Cohere quota usage
- [ ] Verify CORS is working (no blocked requests)
- [ ] Check user feedback

### Week 1

- [ ] Monitor daily error patterns
- [ ] Track popular questions
- [ ] Identify slow queries
- [ ] Check for any crashes
- [ ] Review user feedback

### Ongoing

- [ ] Set up alerts for errors
- [ ] Monitor API quota usage
- [ ] Track response time trends
- [ ] Collect usage metrics
- [ ] Plan improvements

## Rollback Plan

If critical issues occur:

### Immediate Rollback (< 5 min)

1. **Disable Widget** (fastest):
   ```
   # In src/swizzled/Layout/index.tsx
   // <ChatWidget />  // Comment out this line
   ```
   - Push to main branch
   - Vercel auto-deploys (2-3 min)
   - Widget disabled on production

2. **Revert Backend** (if needed):
   - Go to Railway dashboard
   - Click "Deployments"
   - Select previous version
   - Click "Redeploy"

### Full Rollback

1. Revert commits: `git revert <commit-hash>`
2. Push to main
3. Vercel auto-deploys
4. Railway auto-deploys (if connected)
5. Notify team of rollback

## Documentation

- [ ] Update README with chat widget info
- [ ] Document API changes (if any)
- [ ] Add troubleshooting guide to docs
- [ ] Create support channel for issues
- [ ] Document future enhancements

## Post-Deployment

- [ ] Create GitHub issue for feedback
- [ ] Monitor GitHub issues daily
- [ ] Respond to user feedback
- [ ] Plan next iteration
- [ ] Celebrate successful deployment! 🎉

## Sign-Off

| Role | Name | Date | Signature |
|------|------|------|-----------|
| Developer | | | |
| Reviewer | | | |
| QA | | | |
| Product | | | |

---

## Notes

- If deployment fails, check Railway/Render logs
- For CORS issues, verify `CORS_ORIGINS` environment variable
- For 503 errors, check API key validity
- For slow responses, check Qdrant collection size
- Estimated deployment time: 20-30 minutes (all steps)
