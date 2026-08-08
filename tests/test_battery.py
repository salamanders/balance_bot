from balance_bot.adaptation.battery import BatteryEstimator
from balance_bot.configuration import BatteryConfig


def test_battery_estimator_baseline() -> None:
    """Test that the estimator establishes a baseline correctly."""
    config = BatteryConfig(baseline_samples=10, min_pwm=10.0)
    estimator = BatteryEstimator(config=config)

    # 1. Establish Baseline
    # Simulate constant "Responsiveness" = 2.0 (Accel=100, PWM=50)
    _factor = 1.0
    for _ in range(10):
        _factor = estimator.update(pwm=50, angular_accel=100)
        assert _factor == 1.0

    assert estimator.baseline_responsiveness == 2.0


def test_battery_estimator_compensation() -> None:
    """Test that the estimator reduces factor when responsiveness drops."""
    config = BatteryConfig(baseline_samples=10, min_pwm=10.0)
    estimator = BatteryEstimator(config=config)

    # 1. Establish Baseline (Ratio=2.0)
    _factor = 1.0
    for _ in range(10):
        estimator.update(50, 100)

    assert estimator.baseline_responsiveness == 2.0

    # 2. Simulate Low Battery
    # Now responsiveness drops to 1.0 (Accel=50, PWM=50)
    # Ratio = 1.0 / 2.0 = 0.5
    # The factor should slowly drift towards 0.5

    factor = 1.0
    for _ in range(50):
        factor = estimator.update(50, 50)

    # Should be significantly less than 1.0
    assert factor < 0.9
    assert factor >= 0.5  # clamped


def test_battery_estimator_deadzone() -> None:
    """Test that small PWMs are ignored."""
    config = BatteryConfig(baseline_samples=10, min_pwm=20.0)
    estimator = BatteryEstimator(config=config)

    # 1. Try to update with small PWM
    factor = estimator.update(10, 100)

    # Should not count towards samples
    assert estimator.samples_collected == 0
    assert factor == 1.0


def test_battery_estimator_smoothing() -> None:
    """Test that the factor changes smoothly, not instantly."""
    config = BatteryConfig(baseline_samples=10, min_pwm=10.0)
    estimator = BatteryEstimator(config=config)

    # Baseline
    _factor = 1.0
    for _ in range(10):
        estimator.update(50, 100)

    # Instant drop in responsiveness
    # Accel=50, PWM=50 => Ratio=1.0 (vs Baseline 2.0) => Target 0.5

    # One step
    factor = estimator.update(50, 50)

    # Should have changed only slightly (0.01 * target + 0.99 * current)
    # 0.01 * 0.5 + 0.99 * 1.0 = 0.005 + 0.99 = 0.995
    assert factor < 1.0
    assert factor > 0.9  # Shouldn't jump to 0.5


def test_battery_estimator_zero_baseline() -> None:
    """Test that a baseline near zero avoids division by zero and ratio defaults to 1.0."""
    config = BatteryConfig(baseline_samples=1, min_pwm=10.0)
    estimator = BatteryEstimator(config=config)

    # Establish baseline near zero
    estimator.update(100.0, 0.0)

    # Next update should process running estimation
    # It should use the else branch because baseline_responsiveness is 0.0
    factor = estimator.update(100.0, 0.0)

    # The ratio defaults to 1.0, so the target is clamped to 1.0
    # factor = 0.01 * 1.0 + 0.99 * 1.0 = 1.0
    assert factor == 1.0


def test_battery_estimator_max_compensation() -> None:
    """Test that ratio is clamped to max_compensation when responsiveness is high."""
    config = BatteryConfig(baseline_samples=1, min_pwm=10.0, max_compensation=1.2)
    estimator = BatteryEstimator(config=config)

    # Establish low baseline
    estimator.update(100.0, 10.0)  # Baseline responsiveness = 0.1

    # Update with high responsiveness
    factor = 1.0
    for _ in range(500):
        factor = estimator.update(100.0, 1000.0)  # Raw responsiveness = 10.0

    # Current responsiveness goes towards 10.0, ratio goes towards 100
    # But the target is clamped to 1.2
    assert factor > 1.15
    assert factor <= 1.2
