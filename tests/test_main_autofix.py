import sys
from unittest.mock import MagicMock

# Mock hardware modules
sys.modules["smbus"] = MagicMock()
sys.modules["RPi"] = MagicMock()
sys.modules["RPi.GPIO"] = MagicMock()

import unittest
import builtins
from unittest.mock import patch, mock_open

# We mock everything before importing main to avoid side effects if any
with patch("sys.argv", ["main.py", "--auto-fix"]):
    from balance_bot import main

class TestMainAutoFix(unittest.TestCase):

    @patch("balance_bot.main.JulesClient")
    @patch("balance_bot.main.Agent")
    def test_autofix_fallback_file_creation(self, MockAgent, MockJulesClient):
        # Mock sys.argv for this test
        with patch.object(sys, 'argv', ["main.py", "--auto-fix"]):
            # Setup Agent to crash
            mock_agent_instance = MockAgent.return_value
            mock_agent_instance.init.side_effect = Exception("Simulated Crash")

            # Setup JulesClient to fail
            mock_client_instance = MockJulesClient.return_value
            mock_client_instance.report_crash.return_value = (False, "Detailed Crash Report Prompt")

            # Mock open
            m_open = mock_open()

            with patch("builtins.open", m_open):
                # Run main and expect it to raise the exception (as main always re-raises)
                with self.assertRaises(Exception) as cm:
                    main.main()

                self.assertIn("Simulated Crash", str(cm.exception))

                # Verify report_crash was called
                mock_client_instance.report_crash.assert_called_once()

                # Verify file was written
                m_open.assert_called()
                args, kwargs = m_open.call_args
                filename = args[0]
                self.assertTrue(filename.startswith("exception_"))
                self.assertTrue(filename.endswith(".md"))

                handle = m_open()
                handle.write.assert_called_with("Detailed Crash Report Prompt")

    @patch("balance_bot.main.JulesClient")
    @patch("balance_bot.main.Agent")
    def test_autofix_success_no_file(self, MockAgent, MockJulesClient):
        with patch.object(sys, 'argv', ["main.py", "--auto-fix"]):
            # Setup Agent to crash
            mock_agent_instance = MockAgent.return_value
            mock_agent_instance.init.side_effect = Exception("Simulated Crash")

            # Setup JulesClient to succeed
            mock_client_instance = MockJulesClient.return_value
            mock_client_instance.report_crash.return_value = (True, "Detailed Crash Report Prompt")

            # Mock open
            m_open = mock_open()

            with patch("builtins.open", m_open):
                with self.assertRaises(Exception) as cm:
                    main.main()

                self.assertIn("Simulated Crash", str(cm.exception))

                mock_client_instance.report_crash.assert_called_once()

                # Verify NO file was written
                m_open.assert_not_called()
