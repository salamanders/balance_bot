from unittest.mock import MagicMock, patch, call
from pathlib import Path

from balance_bot.behavior.leds import LedController
from balance_bot.configuration import LedConfig


def test_init_finds_path():
    with patch.object(Path, "exists", side_effect=lambda: True):
        controller = LedController()
        assert controller.led_path is not None
        assert controller.led_path.name == "brightness"

def test_init_finds_no_path():
    with patch.object(Path, "exists", return_value=False):
        controller = LedController()
        assert controller.led_path is None

def test_set_led_with_path():
    with patch.object(Path, "exists", return_value=True), patch.object(Path, "write_text") as mock_write_text:
        controller = LedController()

        controller.set_led(True)
        assert controller.is_on is True
        mock_write_text.assert_called_with("1")

        controller.set_led(False)
        assert controller.is_on is False
        mock_write_text.assert_called_with("0")

def test_set_led_without_path():
    with patch.object(Path, "exists", return_value=False), patch.object(Path, "write_text") as mock_write_text:
        controller = LedController()

        # Calling set_led should not raise any error even without led_path
        controller.set_led(True)
        assert controller.is_on is True
        mock_write_text.assert_not_called()

        controller.set_led(False)
        assert controller.is_on is False
        mock_write_text.assert_not_called()

def test_set_led_handles_exceptions():
    with patch.object(Path, "exists", return_value=True), patch.object(Path, "write_text", side_effect=PermissionError):
        controller = LedController()

        # Test PermissionError
        controller.set_led(True)  # Should not raise exception

    with patch.object(Path, "exists", return_value=True), patch.object(Path, "write_text", side_effect=OSError):
        controller = LedController()

        # Test OSError
        controller.set_led(True)  # Should not raise exception

def test_signal_setup():
    with patch.object(Path, "exists", return_value=False):
        config = LedConfig(setup_blink_interval=0.1)
        controller = LedController(config)

        controller.signal_setup()
        assert controller.mode == "SETUP"
        assert controller.blink_interval == 0.1

        # Second call should not do anything (since mode is already SETUP)
        controller.blink_interval = 0.5
        controller.signal_setup()
        assert controller.blink_interval == 0.5

def test_signal_ready():
    with patch.object(Path, "exists", return_value=False):
        controller = LedController()
        with patch.object(controller, "set_led") as mock_set_led:
            controller.signal_ready()
            assert controller.mode == "ON"
            mock_set_led.assert_called_once_with(True)

def test_signal_off():
    with patch.object(Path, "exists", return_value=False):
        controller = LedController()
        controller.mode = "ON"
        with patch.object(controller, "set_led") as mock_set_led:
            controller.signal_off()
            assert controller.mode == "OFF"
            mock_set_led.assert_called_once_with(False)

def test_update_blinking():
    with patch.object(Path, "exists", return_value=False):
        controller = LedController()
        with patch.object(controller, "set_led") as mock_set_led:
            controller.is_on = False
            controller.last_toggle = 100.0

            # Test SETUP mode
            controller.mode = "SETUP"
            controller.blink_interval = 0.1

            with patch("time.monotonic", return_value=100.05):
                controller.update()
                mock_set_led.assert_not_called()

            with patch("time.monotonic", return_value=100.15):
                controller.update()
                mock_set_led.assert_called_once_with(True)
                assert controller.last_toggle == 100.15

def test_update_solid_on():
    with patch.object(Path, "exists", return_value=False):
        controller = LedController()
        with patch.object(controller, "set_led") as mock_set_led:
            controller.mode = "ON"
            controller.is_on = False

            controller.update()
            mock_set_led.assert_called_once_with(True)

            mock_set_led.reset_mock()
            controller.is_on = True
            controller.update()
            mock_set_led.assert_not_called()

def test_update_solid_off():
    with patch.object(Path, "exists", return_value=False):
        controller = LedController()
        with patch.object(controller, "set_led") as mock_set_led:
            controller.mode = "OFF"
            controller.is_on = True

            controller.update()
            mock_set_led.assert_called_once_with(False)

            mock_set_led.reset_mock()
            controller.is_on = False
            controller.update()
            mock_set_led.assert_not_called()

def test_blink():
    with patch.object(Path, "exists", return_value=False):
        controller = LedController()
        with patch.object(controller, "set_led") as mock_set_led:
            with patch("time.sleep") as mock_sleep:
                controller._blink(2, 0.5, 0.2)

                assert mock_set_led.call_args_list == [
                    call(True),
                    call(False),
                    call(True),
                    call(False),
                ]

                assert mock_sleep.call_args_list == [
                    call(0.5),
                    call(0.2),
                    call(0.5),
                    call(0.2),
                ]

def test_countdown():
    with patch.object(Path, "exists", return_value=False):
        config = LedConfig(
            countdown_blink_count_3=3,
            countdown_blink_count_2=2,
            countdown_blink_count_1=1,
            countdown_blink_on_time=0.1,
            countdown_blink_off_time=0.1,
            countdown_pause_time=0.5
        )
        controller = LedController(config)
        with patch.object(controller, "_blink") as mock_blink:
            with patch("time.sleep") as mock_sleep:
                controller.countdown()

                assert mock_blink.call_args_list == [
                    call(3, 0.1, 0.1),
                    call(2, 0.1, 0.1),
                    call(1, 0.1, 0.1)
                ]

                assert mock_sleep.call_args_list == [
                    call(0.5),
                    call(0.5)
                ]
