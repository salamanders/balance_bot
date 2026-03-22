import pytest
from unittest.mock import patch
from pathlib import Path
from balance_bot.configuration import LearningState

def test_learning_state_load_oserror():
    with patch.object(Path, 'exists', return_value=True), \
         patch.object(Path, 'read_text', side_effect=OSError("Disk read error")):
        with pytest.raises(OSError):
            LearningState.load()

def test_learning_state_save_oserror():
    state = LearningState()
    with patch.object(Path, 'write_text', side_effect=OSError("Disk full")):
        with pytest.raises(OSError):
            state.save()

def test_learning_state_load_empty():
    with patch.object(Path, 'exists', return_value=True), \
         patch.object(Path, 'read_text', return_value="   "):
        _state = LearningState.load()
        assert _state == LearningState()
