## 2025-05-02 - [Diagnositc Code Reuse]
**Learning:** Diagnostic tools often repeat the pattern of "record data -> analyze statistics -> check dominance/threshold". This logic is easily duplicated across different checks (wiring vs movement).
**Action:** Centralize statistical analysis and thresholding in `utils.py` early to avoid divergent reporting styles in CLI tools.

## 2025-05-02 - [Safe Motor Control Patterns]
**Learning:** Repeated "drive -> sleep -> stop" patterns in hardware tools are error-prone and verbose.
**Action:** Encapsulate motor drive and measurement loops in a `try...finally` context manager or helper method (like `drive_and_measure`) to guarantee motor safety and reduce boilerplate.
