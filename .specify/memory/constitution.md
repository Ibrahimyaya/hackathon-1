# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Workflow (Non-Negotiable)
All work MUST be driven by a written specification. Before writing code, features, or documentation, a spec MUST exist and be approved. The specification MUST define acceptance criteria, constraints, and success metrics. Specifications MUST use Spec-Kit Plus templates and follow the constitutional standards outlined herein. No deviation from spec without documented amendment.

**Rationale:** Clarity-first development prevents scope creep, aligns stakeholders, and ensures tractable implementation. Every major decision is recorded in specs and can be revisited.

### II. Technical Accuracy from Official Sources (Non-Negotiable)
All technical content MUST be sourced from official documentation, peer-reviewed sources, or authoritative references. No hallucinated APIs, deprecated features, or unverified claims. Every statement of fact MUST be traceable to a primary source. Code examples MUST be tested and reproducible. Documentation MUST be version-pinned where applicable.

**Rationale:** This is a book on AI/specification-driven development. Credibility is essential. Readers rely on accuracy; inaccuracy erodes trust and leads to implementation failures.

### III. Clear, Developer-Focused Writing
Documentation and book content MUST be written for software engineers. Use precise terminology, include working examples, and avoid marketing language. Every explanation MUST be testable (readers can verify claims by running code). Assume intermediate developer competency; do not over-explain basic concepts.

**Rationale:** Developers read to learn and implement. Clarity enables adoption; vague writing wastes reader time.

### IV. Reproducible Setup and Deployment (Non-Negotiable)
Every procedure described in the book MUST be reproducible end-to-end on a clean system. Setup scripts MUST be provided and tested. Deployment MUST be automated and documented. All dependencies MUST be pinned to specific versions. No "it works on my machine" scenarios. Readers MUST be able to follow the book and achieve the same results.

**Rationale:** Reproducibility is a hallmark of rigorous engineering. It enables readers to learn by doing and validates the claims in the book.

### V. No Hallucinated Responses in RAG Chatbot (Non-Negotiable)
The embedded RAG chatbot MUST ONLY answer questions based on book content or user-selected context. If a question cannot be answered from the available knowledge base, the chatbot MUST explicitly state: "I don't have information about this in the current context. Please refer to [official source] or [book section]." Grounding in facts is non-negotiable. No generation of plausible-sounding but unverified information.

**Rationale:** A chatbot that hallucinates undermines the book's credibility and harms readers. RAG grounding ensures responses are traceable and verifiable.

### VI. GitHub-Based Source Control Mandatory
All source code, specifications, configurations, and documentation MUST be stored in GitHub. Commit history MUST be clean, semantic, and traceable. Every PR MUST reference the spec/task it implements. Merge strategy MUST enforce main branch protection. Release artifacts MUST be tagged and reproducible.

**Rationale:** GitHub provides transparency, auditability, and enables collaboration. Clean history makes debugging and feature archaeology straightforward.

### VII. Stack Fidelity (Non-Negotiable)
The technology stack is: Docusaurus (book), OpenAI Agents / ChatKit (RAG chatbot), FastAPI (backend), Neon Postgres (database), Qdrant Cloud (vector store). Alternative technologies MUST NOT be substituted without explicit architectural amendment (ADR). Code examples in the book MUST use this stack or document when using alternatives for pedagogical purposes.

**Rationale:** Stack consistency enables readers to follow along without friction. Demonstrating concepts with non-standard tools defeats the educational purpose.

## Development Workflow

### Specification-First Enforcement
Every feature starts with a spec at `specs/<feature-name>/spec.md`. The spec MUST be approved before implementation begins. Implementation tasks MUST reference the spec. Code review MUST verify spec compliance.

### Architectural Decision Records (ADRs)
Significant architectural decisions MUST be documented in `history/adr/` using the ADR template. Decisions include: technology choices, API design, data models, security policies, and deployment strategies. ADR creation requires ratification; no decisions are retroactively documented.

### Testing Requirements
- Unit tests MUST cover all libraries and core logic (target: ≥80% coverage).
- Integration tests MUST validate end-to-end workflows described in the book.
- Setup and deployment procedures MUST be tested on clean systems before documentation.
- All code examples in the book MUST pass their own tests.

### Code Review Checklist
All PRs MUST verify:
- [ ] Spec exists and is linked in PR description.
- [ ] Technical accuracy: claims are traceable to sources.
- [ ] Reproducibility: setup/deployment steps work on clean systems.
- [ ] Stack compliance: uses approved technologies.
- [ ] No hallucinated content in generated text or examples.
- [ ] Tests pass and coverage is maintained.
- [ ] Commit messages are semantic and reference specs/ADRs.

### Observability and Logging
Structured logging MUST be enabled in all services (FastAPI backend, chatbot). Logs MUST include: request ID, timestamp, severity, context. Errors MUST include stack traces and actionable remediation. Metrics (latency, throughput, errors) MUST be exposed for monitoring.

## Security and Compliance

### Secrets Management
NO secrets hardcoded in code or documentation. Use `.env` files for development. Use managed secrets (GitHub Actions Secrets, Neon, etc.) for deployment. Sensitive examples (API keys, connection strings) MUST be templated with clear instructions for readers.

### Data Handling
User data collected via the chatbot MUST respect privacy. Queries and interactions MUST be logged only for improvement, not surveillance. Data retention policies MUST be documented and enforced.

### Authentication and Authorization
RAG chatbot MAY operate as unauthenticated (public knowledge base) or authenticated (enterprise scenarios). If authentication is added, it MUST be documented in specs with threat models and security justifications.

## Documentation Standards

### Book Structure
- **Part 1: Foundations** — SDD principles, spec-driven workflows, tools.
- **Part 2: Technical Architecture** — System design, API contracts, data flows.
- **Part 3: Implementation** — Hands-on code, setup, deployment.
- **Part 4: Operations** — Monitoring, debugging, scaling.

### Code Example Standards
- Every code example MUST be a working, tested snippet.
- Examples MUST include the stack (Docusaurus, FastAPI, Qdrant, etc.).
- Examples MUST include setup and expected output.
- Deprecated or "do not use" patterns MUST be explicitly marked.

### API Documentation
All public APIs (FastAPI endpoints, chatbot endpoints) MUST be documented with:
- Input/output schemas.
- Error codes and meanings.
- Example requests and responses.
- Rate limits and constraints.

## Governance

### Constitution as Authoritative Source
This constitution supersedes all other project guidance documents. If any document (README, contributing guide, style guide) contradicts this constitution, the constitution prevails. Amendments to the constitution MUST be explicit, dated, and tracked in version history.

### Amendment Procedure
1. Proposer opens an issue detailing the amendment, with rationale.
2. Discussion and feedback collection (minimum 1 week).
3. Proposer updates the constitution and computes a new semantic version.
4. PR is created with the updated constitution and a clear change summary.
5. Review and approval by project leads.
6. Merge to main; tag with version.

### Compliance Verification
- Every PR MUST pass the code review checklist above.
- Spec and plan documents MUST align with current constitution.
- Templates (in `.specify/templates/`) MUST reflect current principles.
- Quarterly audits MUST verify no untracked deviations from constitution.

### Runtime Guidance
Day-to-day development guidance is captured in `CLAUDE.md` and command-specific files in `.claude/commands/`. These documents provide procedural details but MUST NOT contradict constitutional principles. If a conflict is detected, the constitution MUST be amended first, then guidance updated.

**Version**: 1.0.0 | **Ratified**: 2026-01-07 | **Last Amended**: 2026-01-07
