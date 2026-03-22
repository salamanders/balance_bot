from unittest.mock import patch
from pathlib import Path
from balance_bot.configuration import LearningState

def test_learning_state_load_oserror():
    with patch.object(Path, 'exists', return_value=True), \
         patch.object(Path, 'read_text', side_effect=OSError("Disk read error")):
        state = LearningState.load()
        assert isinstance(state, LearningState)

def test_learning_state_save_oserror(caplog):
    state = LearningState()
    with patch.object(Path, 'write_text', side_effect=OSError("Disk full")):
        state.save()
        assert "Disk full" in caplog.text

def test_learning_state_load_empty():
    with patch.object(Path, 'exists', return_value=True), \
         patch.object(Path, 'read_text', return_value="   "):
        state = LearningState.load()
        assert isinstance(state, LearningState)
