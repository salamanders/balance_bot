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
            if bus_id == 1: return mock_bus1
            if bus_id == 3: return mock_bus3
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

        result = scan_i2c_candidates("TestDevice", lambda b: True)
        self.assertIsNone(result)

    def test_verify_with_retries_success(self):
        test_fn = MagicMock(side_effect=["Fail", "Pass"])

        def check_fn(res):
            return res == "Pass"

        result = verify_with_retries("Test", test_fn, check_fn, max_attempts=3, fail_fatal=False)
        self.assertTrue(result)
        self.assertEqual(test_fn.call_count, 2)
        test_fn.assert_has_calls([call(0), call(1)])

    def test_verify_with_retries_fail_retry(self):
        test_fn = MagicMock(return_value="Fail")

        result = verify_with_retries("Test", test_fn, lambda x: False, max_attempts=2, fail_fatal=False)
        self.assertFalse(result)
        self.assertEqual(test_fn.call_count, 2)
        test_fn.assert_has_calls([call(0), call(1)])

    def test_verify_with_retries_fail_fatal(self):
        test_fn = MagicMock(return_value="Fatal")

        with self.assertRaises(SystemExit):
            verify_with_retries("Test", test_fn, lambda x: "FAIL_FATAL", max_attempts=3, fail_fatal=True)
        test_fn.assert_called_with(0)

    def test_find_threshold_success(self):
        # Fail at 10, Success at 15
        action_fn = MagicMock(side_effect=[False, True])

        result = find_threshold("Test", 10, 5, 20, action_fn, lambda x: x)
        self.assertEqual(result, 15)
        self.assertEqual(action_fn.call_count, 2)
        # Called with 10, then 15
        action_fn.assert_has_calls([call(10), call(15)])

    def test_find_threshold_fail_action_retry(self):
        # Fail at 10, Retry at 10, Success at 10
        # This simulates "retry_same" logic
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
        action_fn = MagicMock(return_value=False)

        with self.assertRaises(SystemExit):
            find_threshold("Test", 10, 5, 15, action_fn, lambda x: False)
