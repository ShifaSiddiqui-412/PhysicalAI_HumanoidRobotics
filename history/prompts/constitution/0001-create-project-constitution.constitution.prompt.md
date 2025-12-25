---
id: 1
title: Create project constitution
stage: constitution
date: 2025-12-23
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: shifaSiddiqui
command: /sp.constitution
labels: [constitution, project-setup, governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
/sp.constitution

Project: AI/Spec-Driven Technical Book with Embedded RAG Chatbot

Core principles:
- Spec-driven correctness (Spec-Kit Plus as source of truth)
- Technical accuracy via authoritative docs and runnable code
- Clarity for intermediate–advanced developers
- Reproducibility (end-to-end setup and deployment)
- Zero-hallucination RAG behavior

Key standards:
- All claims backed by official documentation or working code
- Preferred sources: official docs, maintained OSS, standards
- All code must be valid, runnable, and spec-compliant
- Claude Code outputs must strictly follow specs
- Consistent terminology and step-by-step explanations

Constraints:
- Format: Docusaurus (Markdown/MDX)
- Deployment: GitHub Pages
- RAG stack: OpenAI Agents/ChatKit, FastAPI, Neon Serverless Postgres, Qdrant Cloud (Free Tier)
- Retrieval limited to book content or user-selected text only
- No hardcoded secrets; env-based configuration only

Success criteria:
- Book builds and deploys successfully
- Specs validate under Spec-Kit Plus
- RAG chatbot answers only from retrieved context
- Selected-text–only Q&A works correctly
- Project reproducible by a third-party developer
```

## Response snapshot

Updated the constitution template with project-specific information for the AI/Spec-Driven Technical Book with Embedded RAG Chatbot project, including core principles, technology stack constraints, development workflow, and governance guidelines.

## Outcome

- ✅ Impact: Project constitution established with all core principles and constraints
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: none
- 🧠 Reflection: Clear project principles help ensure consistent development direction

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none