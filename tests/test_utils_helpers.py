import unittest
from unittest.mock import MagicMock, patch, call
from balance_bot.utils import scan_i2c_candidates, verify_with_retries, find_threshold

class TestUtilsHelpers(unittest.TestCase):

    @patch("balance_bot.utils.smbus")
    def test_scan_i2c_candidates_found(self, mock_smbus):
        # Bus 1 has device
        mock_bus1 = MagicMock()
        mock_bus3 = MagicMock()

        def smbus_side_effect(bus_id):
            if bus_id == 1:
                return mock_bus1
            if bus_id == 3:
                return mock_bus3
            raise OSError("Bus Error")

        mock_smbus.SMBus.side_effect = smbus_side_effect

        def check_fn(bus):
            # Check function returns True only for bus1
            return bus == mock_bus1

        result = scan_i2c_candidates("TestDevice", check_fn)
        self.assertEqual(result, 1)

    @patch("balance_bot.utils.smbus")
    def test_scan_i2c_candidates_not_found(self, mock_smbus):
        mock_smbus.SMBus.side_effect = OSError("Bus Error")

        result = scan_i2c_candidates("TestDevice", lambda _b: True)
        self.assertIsNone(result)

    def test_verify_with_retries_success(self):
        # We need to simulate retries.
        # test_fn accepts attempt number
        # Returns "Fail" first (attempt 0), then "Pass" (attempt 1)
        # However, verify_with_retries loops on attempts.
        # If check_fn returns False, it continues loop.

        test_fn = MagicMock(side_effect=["Fail", "Pass"])

        def check_fn(res):
            return res == "Pass"

        result = verify_with_retries("Test", test_fn, check_fn, max_attempts=3)
        self.assertTrue(result)
        self.assertEqual(test_fn.call_count, 2)
        # Note: verify_with_retries calls test_fn(i)
        test_fn.assert_has_calls([call(0), call(1)])

    def test_verify_with_retries_fail_retry(self):
        test_fn = MagicMock(return_value="Fail")

        result = verify_with_retries("Test", test_fn, lambda _x: False, max_attempts=2)
        self.assertFalse(result)
        self.assertEqual(test_fn.call_count, 2)
        test_fn.assert_has_calls([call(0), call(1)])

    def test_verify_with_retries_fail_fatal(self):
        # If check_fn returns "FAIL_FATAL", the loop breaks immediately and returns False
        test_fn = MagicMock(return_value="Fatal")

        def check_fn(_res):
            return "FAIL_FATAL"

        result = verify_with_retries("Test", test_fn, check_fn, max_attempts=3)
        self.assertFalse(result)
        test_fn.assert_called_once_with(0)

    def test_find_threshold_success(self):
        # Test steps: 10, 15, 20
        # Fail at 10, Success at 15

        # side_effect is called for each step?
        # find_threshold calls action_fn(val)

        # First call val=10 -> check_fn(False) -> fail
        # Second call val=15 -> check_fn(True) -> found

        def action_fn(val):
            return val == 15

        def check_fn(res):
            return res

        result = find_threshold("Test", 10, 5, 20, action_fn, check_fn)
        self.assertEqual(result, 15)

    def test_find_threshold_fail_action_retry(self):
        # Fail at 10, Retry at 10, Success at 10
        # This simulates "retry_same" logic

        # Call 1: val=10 -> res="Fail1" -> check_fn=False -> fail_action=True (retry)
        # Call 2: val=10 -> res="Success" -> check_fn=True -> found

        action_fn = MagicMock(side_effect=["Fail1", "Success"])

        # fail_action returns True to retry same level
        fail_action = MagicMock(side_effect=[True])

        def check_fn(res):
            return res == "Success"

        result = find_threshold("Test", 10, 5, 20, action_fn, check_fn, fail_action=fail_action)
        self.assertEqual(result, 10)
        self.assertEqual(action_fn.call_count, 2)
        action_fn.assert_has_calls([call(10), call(10)])

    def test_find_threshold_limit_exceeded(self):
        # Always fail
        action_fn = MagicMock(return_value=False)

        result = find_threshold("Test", 10, 5, 15, action_fn, lambda _x: False)
        self.assertIsNone(result)
