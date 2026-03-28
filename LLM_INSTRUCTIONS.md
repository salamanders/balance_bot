# LLM-Optimized Code Instructions (`llm_code.md`)

This repository is built and maintained by autonomous AI agents. To ensure deterministic execution, minimize token
costs, and prevent logic hallucinations, all Python code must be written and structured for machine consumption and
maintenance rather than human narrative readability.

When refactoring or generating code, adhere to the following transformations:

1. Architecture & Abstraction: Change from traditional deeply layered architectures enforcing DRY (Don't Repeat
   Yourself) to Vertical Slice Architecture and Locality of Behavior because deep modularity fragments the LLM's context
   window across multiple files. Colocating all dependencies for a single feature provides a dense, relevant context
   that reduces the "blast radius" of unintended AI edits.
2. Typing & Data Boundaries: Change from implicit "duck typing" and standard Pythonic brevity to exhaustive type
   hinting, strict schema definitions (e.g., Pydantic), and Design-by-Contract decorators (e.g., `icontract`) because
   computational ambiguity is the enemy of deterministic AI execution.
   Explicitly defining preconditions and postconditions prevents the model from probabilistically guessing data shapes
   and allows it to autonomously self-correct based on instant runtime assertion failures.
3. Documentation & Context: Change from narrative docstrings explaining historical choices and conversational inline
   comments to highly structured, machine-readable Context Headers (defining System Context, Business Rules, and
   Dependency Maps at the top of every file) because narrative explanations provide no computational value and waste
   tokens, whereas explicit structural markers act as mathematical
   anchors that allow intelligent RAG chunking systems to extract precise logic without token bloat. Inline comments
   should say why a function is architected the way it is to avoid "thrashing" where the next
   pass chooses a different structure.
4. Development Workflow & Verification: Change from Test-Driven Development (TDD) relying on subjective human "Looks
   Good To Me" reviews to Spec-Driven Development (SDD) utilizing Reinforcement Learning with Verifiable Rewards (RLVR)
   because LLMs frequently over-engineer simple TDD prompts or
   hallucinate logic. Forcing the AI to strictly satisfy immutable, execution-based mathematical tests ensures it
   cannot "hack" the review process with code that merely looks plausible.
5. Project Governance: Change from relying on tacit team culture and implicit architectural "vibes" to codifying
   absolute non-negotiable constraints within rules files like `AGENTS.md` because AI
   agents do not possess context unless it is explicitly injected, meaning multi-agent workflows will rapidly degrade
   the repository into spaghetti code without static, machine-readable
   governance enforcing the execution rules.