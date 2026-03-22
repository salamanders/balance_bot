1. **Replace `test_rate_limiter` in `tests/test_utils.py`**
   - The current `test_rate_limiter` relies on actual `time.sleep()` which is flaky especially in CI environments or under heavy load (e.g. `pytest -n auto` or running it multiple times). Sleep guarantees *minimum* sleep time, not *maximum*. So if CPU is busy it will sleep longer and fail the `elapsed < 0.12` condition.
   - We will replace `test_rate_limiter` with a mocked version similar to `test_rate_limiter_mocked` and `test_rate_limiter_lagging` we just built.
   - We will use `patch("time.perf_counter")` and `patch("time.sleep")` to deterministically verify that `RateLimiter.sleep()` sleeps correctly for the expected time.

2. **Remove `test_limiter_load_mocked.py`, `test_limiter_load.py`, `test_sleep.py`, `test_limiter.py`, `test_limiter_debug.py`, `test_limiter_pytest_load.py`**
   - Clean up scratchpad files used to verify the issue and test the new implementation.

3. **Complete pre commit steps**
   - Complete pre commit steps to ensure proper testing, verifications, reviews and reflections are done.

4. **Submit the change**
   - Submit the change with descriptive title and commit message.
