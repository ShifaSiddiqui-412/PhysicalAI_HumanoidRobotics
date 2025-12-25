# AI/Spec-Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-driven correctness
Spec-Kit Plus as source of truth for all development. All code and features must align with documented specifications before implementation. No feature development without corresponding spec validation.

### Technical accuracy via authoritative docs and runnable code
All claims and implementations must be backed by official documentation or working, testable code. No assumptions or theoretical implementations without verification.

### Clarity for intermediate–advanced developers
Documentation and code explanations must target intermediate to advanced developers. Complex concepts should be broken down systematically with practical examples.

### Reproducibility (end-to-end setup and deployment)
All processes must be reproducible by third-party developers. End-to-end setup instructions and deployment procedures must be complete and tested.

### Zero-hallucination RAG behavior
RAG chatbot must only respond based on retrieved context from book content or user-selected text. No generated content outside of provided context is allowed.

### Code Standards and Consistency
All code must be valid, runnable, and spec-compliant. Consistent terminology and step-by-step explanations throughout the project.

## Technology Stack Constraints
- Format: Docusaurus (Markdown/MDX)
- Deployment: GitHub Pages
- RAG stack: OpenAI Agents/ChatKit, FastAPI, Neon Serverless Postgres, Qdrant Cloud (Free Tier)
- Retrieval limited to book content or user-selected text only
- No hardcoded secrets; env-based configuration only

## Development Workflow
- All outputs must strictly follow specs defined in Spec-Kit Plus
- Claude Code outputs must follow specifications precisely
- All code must be validated for technical accuracy and reproducibility
- Preferred sources: official docs, maintained OSS, standards
- All code must be valid, runnable, and spec-compliant

## Governance

All development must adhere to the defined principles and constraints. Changes to core principles require explicit documentation and approval. All code and documentation must maintain technical accuracy and reproducibility standards. Compliance with the RAG limitation to book content or user-selected text is non-negotiable.

**Version**: 1.0.0 | **Ratified**: 2025-12-23 | **Last Amended**: 2025-12-23
