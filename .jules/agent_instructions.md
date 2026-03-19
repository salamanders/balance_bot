# Agent Guidelines: Code Quality and Type Safety

When working on this codebase, you must proactively prevent the following categories of errors that have previously plagued the project (e.g., incorrect types, mismatched function signatures, unused imports, and unbound variables). These errors often pass syntax checks but fail strict static analysis.

## 1. The Root Cause of Past Issues
In the past, agents introduced silent errors because they:
*   **Assumed Function Signatures:** Called functions with incorrect arguments (e.g., passing `Literal[Direction.BACKWARD]` when a `float` was expected) without verifying the function's definition first.
*   **Ignored Hardware Abstraction Interfaces:** Mismatched arguments when implementing or mocking hardware interfaces (e.g., `RobotHardware` vs. `SimHardware`).
*   **Left Unused Artifacts:** Left behind unused imports and local variables (e.g., in test files) after refactoring.
*   **Overlooked Shadowing:** Shadowed built-in names or outer scope variables.

## 2. Mandatory Pre-Flight Checklist
Before modifying code or submitting a change, you **MUST** follow these steps:

### A. Verify Function Signatures
Never guess a function's arguments or return type. Before calling any function or instantiating a class (especially Pydantic models like `HardwareConfig` or `LearningState`), use `read_file` or `grep` to read the actual definition in the source code.
*   Check for required positional vs. keyword arguments.
*   Verify the expected types (e.g., `float` vs. `Enum`, `int` vs. `float`).

### B. Handle Enums Correctly
When performing arithmetic operations or comparisons with custom Enum types (like `Direction` or `Axis`), explicitly access their numeric `.value` attribute to avoid `TypeError`s. Do not pass an Enum object where a primitive (like `float` or `int`) is expected.

### C. Clean Up After Yourself
When refactoring code:
*   Remove unused imports immediately.
*   Remove unused local variables. In benchmarking or test scripts where objects are instantiated solely for timing/side-effects, assign the result to `_` (e.g., `_ = ClassName(...)`) to satisfy linters.

### D. Run Linters and Tests
Before calling `submit`, you must:
1.  Run `uv run ruff check .` and fix all reported warnings (unused imports, unused variables, shadowing).
2.  Run the full test suite with `uv run pytest tests/` to ensure no regressions were introduced.

By strictly adhering to these rules, you will prevent the reintroduction of the structural and type-safety issues documented in `inspections.md`.
