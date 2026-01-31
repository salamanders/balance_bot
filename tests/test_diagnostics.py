import pytest
from unittest.mock import patch, MagicMock
from balance_bot.diagnostics import get_i2c_failure_report

@patch("balance_bot.diagnostics.Path.exists")
def test_bus_not_exist(mock_exists):
    mock_exists.return_value = False
    report = get_i2c_failure_report(1, 0x22, "TestDevice")
    assert "CRITICAL FAILURE: I2C Bus 1" in report
    assert "does not exist" in report

@patch("balance_bot.diagnostics.Path.exists")
@patch("balance_bot.diagnostics.os.access")
def test_permission_denied(mock_access, mock_exists):
    mock_exists.return_value = True
    mock_access.return_value = False
    report = get_i2c_failure_report(1, 0x22, "TestDevice")
    assert "CRITICAL FAILURE: Permission denied" in report

@patch("balance_bot.diagnostics.Path.exists")
@patch("balance_bot.diagnostics.os.access")
@patch("balance_bot.diagnostics.subprocess.run")
def test_device_detected_but_failed(mock_run, mock_access, mock_exists):
    mock_exists.return_value = True
    mock_access.return_value = True

    # Mock i2cdetect output showing 22
    mock_proc = MagicMock()
    mock_proc.stdout = "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n20: -- -- 22 -- -- -- -- -- -- -- -- -- -- -- -- --"
    mock_run.return_value = mock_proc

    report = get_i2c_failure_report(1, 0x22, "TestDevice")
    assert "CONFUSION: Device TestDevice (0x22) IS detected" in report

@patch("balance_bot.diagnostics.Path.exists")
@patch("balance_bot.diagnostics.os.access")
@patch("balance_bot.diagnostics.subprocess.run")
def test_device_not_detected(mock_run, mock_access, mock_exists):
    mock_exists.return_value = True
    mock_access.return_value = True

    # Mock i2cdetect output NOT showing 22
    mock_proc = MagicMock()
    mock_proc.stdout = "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --"
    mock_run.return_value = mock_proc

    report = get_i2c_failure_report(1, 0x22, "TestDevice")
    assert "HARDWARE FAILURE: Device TestDevice (0x22) is NOT responding" in report
