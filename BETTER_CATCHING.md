# Hypothesis: Root Cause of Missing Attribute Error in `icontract` Decorator

The error in question occurred during the execution of `repl.py` running a gyro test:
```
AttributeError: 'MeasureResult' object has no attribute 'status'
```
This error was triggered dynamically at runtime by the `@icontract.ensure(lambda result: result.status != 'unknown', ...)` decorator on the `drive_and_measure` function in `src/balance_bot/hardware/robot_hardware.py` (and duplicated in `sim_hardware.py`), because the return type `MeasureResult` does not have a `status` attribute.

## Why wasn't this caught earlier?

### 1. Weak Static Typing for `icontract`
The `@icontract.ensure` decorator accepts a lambda function. While the lambda implicitly takes the `result` argument (the return value of the decorated function, which is typed as `MeasureResult`), `mypy` does not automatically infer or enforce that `result` parameter within the lambda matches the return type of the decorated function.
Since `icontract` decorators lack generic type parameter bindings (like `Callable[[T], bool]` mapped to the function's return type `T`), static analysis tools treat the lambda argument `result` as `Any`. Therefore, `mypy` silently permitted accessing `.status` on it.

We confirmed this limitation with a quick test: a mocked generic wrapper accurately catches `error: "T" has no attribute "status"`, while `icontract` lets it slip past `mypy`.

### 2. Lack of Unit Test Coverage
We ran `grep` across the `tests/` directory and found zero unit tests that execute `drive_and_measure` directly or simulate the REPL's `test_gyro` command.
Since `icontract` is evaluated purely at runtime, the only way to catch an invalid contract (aside from better static type hints) is to actually execute the function. Because this function was untested, the bug went unnoticed until manual execution of the REPL.

### 3. Untyped Decorator Warning Suppression
`mypy` actually flags the functions decorated with `icontract` as "untyped":
```
error: Untyped decorator makes function "drive_and_measure" untyped [untyped-decorator]
```
However, the codebase's mypy configuration or local execution habits either ignore this warning or don't enforce strict typing on decorators, meaning we lose the strict typing for `drive_and_measure` simply by using `icontract`.

## Recommended Improvements

To prevent this entire category of errors (invalid lambda assertions inside decorators going unnoticed until runtime), we should implement the following changes:

1. **Enforce Test Coverage on Contracts:**
   Functions with complex runtime contracts (like `icontract.require`/`ensure`) must have unit tests covering their happy paths to trigger the contract validations during CI/test runs. Any function using an `icontract` decorator should be executed at least once in `pytest`.

2. **Adopt Stronger Static Type Enforcement for Contracts (or Alternatives):**
   - **Type Hinting the Lambda:** explicitly type hint the lambda argument in contracts (though lambda type hints are syntactically awkward in Python: `def check_status(result: MeasureResult) -> bool: ...` instead of lambdas).
   - If we define a separate function `check_status(result: MeasureResult) -> bool`, `mypy` will enforce that `.status` must exist on `MeasureResult`.
   - Consider replacing `icontract` with static typing or `pydantic`'s `@validate_call`, which integrates deeply with type hints and provides robust validation without losing static analysis benefits.

3. **Strict Mypy Rules on Decorators:**
   Ensure `mypy --strict` or `--disallow-untyped-decorators` is enabled and passing. This will force developers to provide properly typed stubs for external libraries or wrapper functions, surfacing type issues when decorators break the type chain.
