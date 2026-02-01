from balance_bot.tuner import BalancePointFinder
from balance_bot.config import TunerConfig

def test_balance_finder_no_adjustment():
    config = TunerConfig(
        balance_check_interval=5,
        balance_motor_threshold=5.0,
        balance_learning_rate=0.1
    )
    finder = BalancePointFinder(config)

    # 4 samples, not enough to trigger
    for _ in range(4):
        adj = finder.update(motor_output=10.0, pitch_rate=0.0)
        assert adj == 0.0

def test_balance_finder_positive_drift():
    # Robot is leaning forward, motor output is positive
    config = TunerConfig(
        balance_check_interval=5,
        balance_motor_threshold=5.0,
        balance_learning_rate=0.1
    )
    finder = BalancePointFinder(config)

    # 4 samples of 0 adjustment
    for _ in range(4):
        adj = finder.update(motor_output=10.0, pitch_rate=0.0)
        assert adj == 0.0

    # 5th sample triggers check
    adj = finder.update(motor_output=10.0, pitch_rate=0.0)
    # Average is 10.0 > 5.0 -> Decrease target angle (lean back)
    assert adj == -0.1

def test_balance_finder_negative_drift():
    # Robot is leaning backward, motor output is negative
    config = TunerConfig(
        balance_check_interval=5,
        balance_motor_threshold=5.0,
        balance_learning_rate=0.1
    )
    finder = BalancePointFinder(config)

    for _ in range(4):
        finder.update(motor_output=-10.0, pitch_rate=0.0)

    adj = finder.update(motor_output=-10.0, pitch_rate=0.0)
    # Average is -10.0 < -5.0 -> Increase target angle (lean forward)
    assert adj == 0.1

def test_balance_finder_reset_after_check():
    config = TunerConfig(
        balance_check_interval=5,
        balance_motor_threshold=5.0,
        balance_learning_rate=0.1
    )
    finder = BalancePointFinder(config)

    # Trigger one adjustment
    for _ in range(5):
        finder.update(motor_output=10.0, pitch_rate=0.0)

    # Next sample should be 0, buffer cleared
    adj = finder.update(motor_output=10.0, pitch_rate=0.0)
    assert adj == 0.0

def test_balance_finder_unstable_ignored():
    config = TunerConfig(
        balance_check_interval=5,
        balance_motor_threshold=5.0,
        balance_learning_rate=0.1,
        balance_pitch_rate_threshold=1.0
    )
    finder = BalancePointFinder(config)

    # Unstable input
    adj = finder.update(motor_output=10.0, pitch_rate=5.0)
    assert adj == 0.0

    # Check that it wasn't added to history
    assert len(finder.motor_history) == 0

    # Stable input
    adj = finder.update(motor_output=10.0, pitch_rate=0.5)
    assert len(finder.motor_history) == 1
