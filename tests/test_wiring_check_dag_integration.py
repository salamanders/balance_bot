import pytest
from unittest.mock import MagicMock, patch
from balance_bot.wiring_check import WiringCheck

@patch("balance_bot.wiring_check.DAGExecutor")
@patch("balance_bot.wiring_check.HardwareConfig")
@patch("balance_bot.wiring_check.LearningState")
@patch("balance_bot.wiring_check.run_diagnostics")
def test_wiring_check_run_dag_construction(mock_diag, mock_ls, mock_hw_config, mock_dag_cls):
    """
    Verify that WiringCheck.run() constructs the DAG and calls execute.
    """
    # Setup
    wc = WiringCheck()
    mock_executor = mock_dag_cls.return_value
    mock_executor.run.return_value = True # Simulate success

    # Mock verify/summary to avoid HW calls
    wc.verify_final_configuration = MagicMock()
    wc._print_summary = MagicMock()
    wc.cleanup = MagicMock()

    # Run
    wc.run()

    # Verify run_diagnostics called
    mock_diag.assert_called_once()

    # Verify DAGExecutor instantiated with a graph
    assert mock_dag_cls.call_count == 1
    graph = mock_dag_cls.call_args[0][0]

    # Check keys exist
    expected_keys = [
        "i2c_buses", "hardware_init", "motor_candidates", "friction_threshold",
        "spatial_orientation", "motor_phasing", "motor_direction",
        "left_right_identity", "motor_trim", "mechanical_backlash",
        "kickup_dynamics"
    ]
    for k in expected_keys:
        assert k in graph, f"Missing key {k} in knowledge graph"

    # Verify execute called
    mock_executor.run.assert_called_once()
