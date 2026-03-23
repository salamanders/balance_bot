from typing import Any
import unittest
from unittest.mock import MagicMock, patch, call
from balance_bot.utils import scan_i2c_candidates, find_threshold

class TestUtilsHelpers(unittest.TestCase):

    @patch("balance_bot.utils.smbus")
    def test_scan_i2c_candidates_found(self: Any, mock_smbus: Any) -> None:
        # Bus 1 has device
        mock_bus1 = MagicMock()
        mock_bus1.__enter__.return_value = mock_bus1
        mock_bus3 = MagicMock()
        mock_bus3.__enter__.return_value = mock_bus3

        def smbus_side_effect(bus_id: Any) -> Any:
            if bus_id == 1:
                return mock_bus1
            if bus_id == 3:
                return mock_bus3
            raise OSError("Bus Error")

        mock_smbus.SMBus.side_effect = smbus_side_effect

        def check_fn(bus: Any) -> Any:
            # Check function returns True only for bus1
            return bus == mock_bus1

        result = scan_i2c_candidates("TestDevice", check_fn)
        self.assertEqual(result, 1)

    @patch("balance_bot.utils.smbus")
    def test_scan_i2c_candidates_not_found(self: Any, mock_smbus: Any) -> None:
        mock_smbus.SMBus.side_effect = OSError("Bus Error")

        result = scan_i2c_candidates("TestDevice", lambda _b: True)
        self.assertIsNone(result)

    def test_find_threshold_success(self: Any) -> None:
        # Test steps: 10, 15, 20
        # Fail at 10, Success at 15

        # side_effect is called for each step?
        # find_threshold calls action_fn(val)

        # First call val=10 -> check_fn(False) -> fail
        # Second call val=15 -> check_fn(True) -> found

        def action_fn(val: Any) -> Any:
            return val == 15

        def check_fn(res: Any) -> Any:
            return res

        result = find_threshold("Test", 10, 5, 20, action_fn, check_fn)
        self.assertEqual(result, 15)

    def test_find_threshold_fail_action_retry(self: Any) -> None:
        # Fail at 10, Retry at 10, Success at 10
        # This simulates "retry_same" logic

        # Call 1: val=10 -> res="Fail1" -> check_fn=False -> fail_action=True (retry)
        # Call 2: val=10 -> res="Success" -> check_fn=True -> found

        action_fn = MagicMock(side_effect=["Fail1", "Success"])

        # fail_action returns True to retry same level
        fail_action = MagicMock(side_effect=[True])

        def check_fn(res: Any) -> Any:
            return res == "Success"

        result = find_threshold("Test", 10, 5, 20, action_fn, check_fn, fail_action=fail_action)
        self.assertEqual(result, 10)
        self.assertEqual(action_fn.call_count, 2)
        action_fn.assert_has_calls([call(10), call(10)])

    def test_find_threshold_limit_exceeded(self: Any) -> None:
        # Always fail
        action_fn = MagicMock(return_value=False)

        result = find_threshold("Test", 10, 5, 15, action_fn, lambda _x: False)
        self.assertIsNone(result)
