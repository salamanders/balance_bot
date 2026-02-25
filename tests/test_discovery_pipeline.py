import pytest
from unittest.mock import MagicMock, patch
from balance_bot.discovery.pipeline import SelfDiscoveryPipeline
from balance_bot.discovery.step import CalibrationStep, StepStatus

def test_pipeline_flow():
    """Verify that pipeline runs steps sequentially and saves state."""

    # Mock dependencies
    mock_hw_cls = MagicMock()
    mock_config_cls = MagicMock()
    mock_state_cls = MagicMock()
    mock_watchdog = MagicMock()

    with patch("balance_bot.discovery.pipeline.RobotHardware", mock_hw_cls), \
         patch("balance_bot.discovery.pipeline.HardwareConfig", mock_config_cls), \
         patch("balance_bot.discovery.pipeline.LearningState", mock_state_cls):

         # Setup State
         mock_state = MagicMock()
         mock_state_cls.load.return_value = mock_state

         mock_config = MagicMock()
         mock_config_cls.load.return_value = mock_config

         # Mock model_copy to return a different mock so strict inequality passes
         new_mock_config = MagicMock()
         mock_config.model_copy.return_value = new_mock_config

         mock_hw = mock_hw_cls.return_value

         # Define Steps
         step1 = MagicMock(spec=CalibrationStep)
         step1.name = "Step1"
         step1.is_verified.return_value = False
         step1.run.return_value = (StepStatus.SUCCESS, {}, {}) # Success, no config change

         step2 = MagicMock(spec=CalibrationStep)
         step2.name = "Step2"
         step2.is_verified.return_value = True # Already verified

         step3 = MagicMock(spec=CalibrationStep)
         step3.name = "Step3"
         step3.is_verified.return_value = False
         # Step 3 changes config
         config_updates = {'field': 'val'}
         step3.run.return_value = (StepStatus.SUCCESS, config_updates, {})

         pipeline = SelfDiscoveryPipeline([step1, step2, step3], mock_watchdog)
         pipeline.run()

         # Verify Execution
         # Step 1 ran
         step1.run.assert_called_once()

         # Step 2 skipped
         step2.run.assert_not_called()

         # Step 3 ran and updated config
         step3.run.assert_called_once()
         mock_config.model_copy.assert_called_with(update=config_updates)
         new_mock_config.save.assert_called_once()
         mock_hw.apply_config.assert_called_with(new_mock_config)

         # Verify HAL stop
         mock_hw.stop.assert_called_once()
