import os
import json
import unittest
from unittest.mock import patch, MagicMock
from balance_bot.jules_client import JulesClient

class TestJulesClient(unittest.TestCase):
    def setUp(self):
        # Prevent environment variable from interfering
        self.env_patcher = patch.dict(os.environ, {"JULES_API_KEY": "test_key_env"}, clear=True)
        self.env_patcher.start()

    def tearDown(self):
        self.env_patcher.stop()

    def test_init_with_argument(self):
        client = JulesClient(api_key="argument_key")
        # Check that the public attribute is gone (or renamed)
        self.assertFalse(hasattr(client, "api_key"), "Public 'api_key' attribute should not exist")
        # Check private attribute
        self.assertEqual(client._api_key, "argument_key")

    def test_init_with_env_var(self):
        client = JulesClient()
        self.assertEqual(client._api_key, "test_key_env")

    def test_repr_masking(self):
        client = JulesClient(api_key="secret_key_123")
        repr_str = repr(client)
        self.assertNotIn("secret_key_123", repr_str)
        self.assertIn("***", repr_str)

    @patch("urllib.request.urlopen")
    def test_make_request_success(self, mock_urlopen):
        client = JulesClient(api_key="test_key")

        # Mock response
        mock_response = MagicMock()
        mock_response.status = 200
        mock_response.read.return_value = json.dumps({"status": "ok"}).encode("utf-8")
        mock_urlopen.return_value.__enter__.return_value = mock_response

        response = client._make_request("GET", "test_endpoint")

        self.assertEqual(response, {"status": "ok"})

        # Verify headers contain the API key
        args, kwargs = mock_urlopen.call_args
        req = args[0]
        # urllib headers are usually capitalized
        self.assertEqual(req.headers["X-goog-api-key"], "test_key")

    @patch("urllib.request.urlopen")
    def test_make_request_missing_key(self, mock_urlopen):
        # Init without key
        with patch.dict(os.environ, {}, clear=True):
            client = JulesClient()
            with self.assertRaises(ValueError):
                client._make_request("GET", "test")

    @patch("balance_bot.jules_client.JulesClient.create_session")
    def test_report_crash_success(self, mock_create_session):
        client = JulesClient(api_key="test_key")
        mock_create_session.return_value = {"name": "session_123"}

        success, prompt = client.report_crash("Error", "Stack", "Logs", {}, {})

        self.assertTrue(success)
        self.assertIn("The application crashed", prompt)
        mock_create_session.assert_called_once()

    def test_report_crash_missing_key(self):
        # Ensure env var is cleared so no key is present
        with patch.dict(os.environ, {}, clear=True):
            client = JulesClient(api_key=None)
            success, prompt = client.report_crash("Error", "Stack", "Logs", {}, {})

            self.assertFalse(success)
            self.assertIn("The application crashed", prompt)

    @patch("balance_bot.jules_client.JulesClient.create_session")
    def test_report_crash_failure(self, mock_create_session):
        client = JulesClient(api_key="test_key")
        mock_create_session.side_effect = Exception("API Error")

        success, prompt = client.report_crash("Error", "Stack", "Logs", {}, {})

        self.assertFalse(success)
        self.assertIn("The application crashed", prompt)
