from unittest.mock import MagicMock
import pytest
import os
from balance_bot.discovery.pipeline import SelfDiscoveryPipeline
from balance_bot.discovery.step import StepStatus, CalibrationStep
from balance_bot.configuration import HardwareConfig, LearningState

# Ensure mock fallback is allowed for pipeline (which creates hardware)
os.environ["ALLOW_MOCK_FALLBACK"] = "1"

class MockStep:
    def __init__(self, name="MockStep"):
        self._name = name
        self.run_called = False

    @property
    def name(self):
        return self._name

    def is_verified(self, state):
        return False

    def run(self, hw, config, state):
        self.run_called = True
        # Return a valid field update for LearningState
        return StepStatus.SUCCESS, {}, {'i2c_buses_verified': True}

def test_pipeline_execution():
    watchdog = MagicMock()
    pipeline = SelfDiscoveryPipeline(watchdog)

    mock_step_1 = MockStep("Step1")
    mock_step_2 = MockStep("Step2")
    pipeline.steps = [mock_step_1, mock_step_2]

    pipeline.run()

    assert mock_step_1.run_called
    assert mock_step_2.run_called

def test_pipeline_skips_verified():
    watchdog = MagicMock()
    pipeline = SelfDiscoveryPipeline(watchdog)

    mock_step = MockStep("Step1")
    # Patch is_verified to True
    mock_step.is_verified = lambda s: True

    pipeline.steps = [mock_step]
    pipeline.run()

    assert not mock_step.run_called
