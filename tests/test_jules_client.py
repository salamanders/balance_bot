import os
import json
import unittest
from unittest.mock import patch, MagicMock
from balance_bot.jules_client import JulesClient, CrashReport

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
    def test_make_request_timeout(self, mock_urlopen):
        client = JulesClient(api_key="test_key")

        # Mock response
        mock_response = MagicMock()
        mock_response.status = 200
        mock_response.read.return_value = json.dumps({"status": "ok"}).encode("utf-8")
        mock_urlopen.return_value.__enter__.return_value = mock_response

        client._make_request("GET", "test_endpoint")

        # Verify timeout is passed to urlopen
        args, kwargs = mock_urlopen.call_args
        from balance_bot.jules_client import DEFAULT_TIMEOUT
        self.assertEqual(kwargs["timeout"], DEFAULT_TIMEOUT)

    @patch("urllib.request.urlopen")
    def test_make_request_missing_key(self, _mock_urlopen):
        # Init without key
        with patch.dict(os.environ, {}, clear=True):
            client = JulesClient()
            with self.assertRaises(ValueError):
                client._make_request("GET", "test")

    @patch.object(JulesClient, "_make_request")
    def test_get_sources(self, mock_make_request):
        client = JulesClient(api_key="test_key")
        mock_make_request.return_value = {"sources": ["source1", "source2"]}

        sources = client.get_sources()

        self.assertEqual(sources, ["source1", "source2"])
        mock_make_request.assert_called_once_with("GET", "sources")

    @patch.object(JulesClient, "_make_request")
    def test_create_session(self, mock_make_request):
        client = JulesClient(api_key="test_key")
        mock_make_request.return_value = {"name": "sessions/123"}

        response = client.create_session("Fix this bug")

        self.assertEqual(response, {"name": "sessions/123"})
        args, _ = mock_make_request.call_args
        self.assertEqual(args[0], "POST")
        self.assertEqual(args[1], "sessions")
        payload = args[2]
        self.assertEqual(payload["prompt"], "Fix this bug")
        self.assertEqual(payload["automationMode"], "AUTO_CREATE_PR")

    @patch.object(JulesClient, "create_session")
    def test_report_crash_success(self, mock_create_session):
        client = JulesClient(api_key="test_key")
        mock_create_session.return_value = {"name": "sessions/456"}

        report = CrashReport(
            error_msg="ZeroDivisionError",
            stack_trace="traceback...",
            logs="logs...",
            state={"pitch": 0.1},
            libs={"numpy": "1.2.3"},
            telemetry="telemetry..."
        )
        success, prompt = client.report_crash(report)

        self.assertTrue(success)
        self.assertIn("ZeroDivisionError", prompt)
        self.assertIn("traceback...", prompt)
        self.assertIn("telemetry...", prompt)
        mock_create_session.assert_called_once_with(prompt)

    def test_report_crash_no_api_key(self):
        # Ensure no API key in environment or argument
        with patch.dict(os.environ, {}, clear=True):
            client = JulesClient(api_key=None)
            report = CrashReport("err", "trace", "logs", {}, {})
            success, prompt = client.report_crash(report)
            self.assertFalse(success)
            self.assertIn("err", prompt)

    @patch.object(JulesClient, "create_session")
    def test_report_crash_api_error(self, mock_create_session):
        client = JulesClient(api_key="test_key")
        mock_create_session.side_effect = Exception("API Down")

        report = CrashReport("err", "trace", "logs", {}, {})
        success, prompt = client.report_crash(report)

        self.assertFalse(success)
        self.assertIn("err", prompt)

    def test_crash_report_string_representation(self):
        report = CrashReport(
            error_msg="TestError",
            stack_trace="trace",
            logs="logs",
            state={"a": 1},
            libs={"lib": "1.0"},
            telemetry="telemetry"
        )
        rep_str = str(report)
        self.assertIn("TestError", rep_str)
        self.assertIn("trace", rep_str)

    @patch("urllib.request.urlopen")
    def test_report_crash_http_error(self, mock_urlopen):
        client = JulesClient(api_key="test_key")

        from urllib.error import HTTPError
        import io

        fp = io.BytesIO(b'{"error": "Bad Request"}')
        mock_err = HTTPError(url="http://test", code=400, msg="Bad Request", hdrs={}, fp=fp)
        mock_urlopen.side_effect = mock_err

        report = CrashReport("err", "trace", "logs", {}, {})
        result = client.report_crash(report)

        success = result[0] if isinstance(result, tuple) else result
        self.assertFalse(success)

    @patch("urllib.request.urlopen")
    def test_report_crash_generic_exception(self, mock_urlopen):
        client = JulesClient(api_key="test_key")

        mock_urlopen.side_effect = Exception("Network down")

        report = CrashReport("err", "trace", "logs", {}, {})
        result = client.report_crash(report)

        success = result[0] if isinstance(result, tuple) else result
        self.assertFalse(success)

    @patch("urllib.request.urlopen")
    def test_report_crash_status_above_300(self, mock_urlopen):
        client = JulesClient(api_key="test_key")

        # Mock response with status >= 300
        mock_response = MagicMock()
        mock_response.status = 500
        mock_response.reason = "Internal Server Error"
        mock_urlopen.return_value.__enter__.return_value = mock_response

        report = CrashReport("err", "trace", "logs", {}, {})
        result = client.report_crash(report)

        success = result[0] if isinstance(result, tuple) else result
        self.assertFalse(success)
