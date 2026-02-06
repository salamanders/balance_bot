## 2025-05-02 - [Diagnositc Code Reuse]
**Learning:** Diagnostic tools often repeat the pattern of "record data -> analyze statistics -> check dominance/threshold". This logic is easily duplicated across different checks (wiring vs movement).
**Action:** Centralize statistical analysis and thresholding in `utils.py` early to avoid divergent reporting styles in CLI tools.
