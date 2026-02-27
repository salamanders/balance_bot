from unittest.mock import MagicMock
import pytest
from balance_bot.discovery.steps import MechanicalBacklashStep
from balance_bot.configuration import HardwareConfig, LearningState
import time

def test_backlash_step_logic():
    """Test that MechanicalBacklashStep calculates compensation correctly."""
    step = MechanicalBacklashStep()

    hw = MagicMock()
    config = HardwareConfig()
    state = LearningState()

    # Mock behavior:
    # 1. Forward drive (sleeps)
    # 2. Reverse drive (start timer)
    # 3. Read IMU -> wait -> Read IMU -> Spike

    # Sequence of return values for hw.read_imu_converted()
    r1 = MagicMock(pitch_rate=0.0)
    r2 = MagicMock(pitch_rate=1.0)
    r3 = MagicMock(pitch_rate=10.0) # Spike

    hw.read_imu_converted.side_effect = [r1, r2, r3, r3, r3]

    # Mock time.time to simulate precise delay
    # start_time = 100.0
    # first check: 100.05
    # second check: 100.10
    # third check: 100.15 -> Spike detected.
    # slop_time = 0.15
    # compensated = 0.15 - 0.02 = 0.13

    with pytest.MonkeyPatch.context() as m:
        # Step logic:
        # start = time.time() [100.0]
        # while time.time() [100.05] - start < 1.0:
        #   read imu
        #   sleep(0.005)
        #   ...

        # We need to feed enough times to cover the loop.
        # Call 1: Start time capture -> 100.0
        # Call 2: While check 1 -> 100.0
        # Call 3: While check 2 -> 100.05
        # Call 4: While check 3 -> 100.10
        # Call 5: While check 4 -> 100.15 (Break condition met by IMU spike)
        # Call 6: End time calc -> 100.15

        t_mock = MagicMock(side_effect=[
            100.0,  # start_time
            100.0,  # while check 1
            100.05, # while check 2
            100.10, # while check 3
            100.15, # while check 4
            100.15, # slop_time calc (re-read or variable capture? variable capture uses start_time)
            100.20
        ])
        m.setattr(time, 'time', t_mock)

        from balance_bot.discovery.step import StepStatus
        status, cfg_upd, state_upd = step.run(hw, config, state)

        assert status == StepStatus.SUCCESS
        assert state_upd['backlash_verified'] is True

        new_control = state_upd['control']

        # slop_time = current_time (100.15) - start_time (100.0) = 0.15
        # comp = 0.15 - 0.02 = 0.13
        # Assert with small tolerance
        assert abs(new_control.backlash_pulse_time - 0.13) < 0.01
