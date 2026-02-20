## 2025-02-18 - Refactoring Hardware Drivers with Global State
**Learning:** When refactoring a module that relies heavily on global state (like `bus`, `DEBUG`, `RETRIES`) and is imported by other modules, testing becomes tricky. `unittest.mock.patch` might not work as expected if the module is not fully reloaded between tests. Using `importlib.reload` is essential to ensure a clean state when the module initializes resources (like `smbus.SMBus`) at the top level.
**Action:** When testing hardware drivers with global initialization, explicitly reload the module in `setUp` using `importlib.reload(module)` and ensure dependent modules are also handled if necessary.

## 2025-02-18 - Retry Logic Simplification
**Learning:** Hardware drivers often repeat error handling logic (try-except-retry) across many methods. This is a prime candidate for DRY. A simple decorator or helper function `_retry(func, name)` can reduce code size significantly (e.g., 50%) and ensure consistent error reporting.
**Action:** Look for repeated `try...except` blocks in hardware interface code and refactor them into a helper function early.

## 2025-02-18 - Pruning Vendor Code in src/
**Learning:** Vendor-supplied code (like `piconzero.py`) placed in `src/` can contain significant amounts of unused, redundant logic (e.g., helper methods for features not used by the application). Removing this dead code simplifies maintenance and testing, even if it deviates from the original vendor file.
**Action:** Always check "vendor" files in `src/` for unused code and aggressively prune them if they are not maintained as an external dependency.
