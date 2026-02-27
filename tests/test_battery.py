from balance_bot.configuration import BatteryConfig
from balance_bot.adaptation.battery import BatteryEstimator

def test_battery_estimator_baseline():
    """Test that the estimator establishes a baseline correctly."""
    config = BatteryConfig(baseline_samples=10, min_pwm=10.0)
    estimator = BatteryEstimator(config=config)

    # 1. Establish Baseline
    # Simulate constant "Responsiveness" = 2.0 (Accel=100, PWM=50)
    for _ in range(10):
        factor = estimator.update(pwm=50, angular_accel=100)

    # During baseline, factor should stay 1.0 (or whatever default)
    # The actual implementation returns 1.0 during baseline collection
    assert factor == 1.0
    assert estimator.samples_collected == 10
    # Baseline should be ~2.0
    assert abs(estimator.baseline_responsiveness - 2.0) < 0.1

def test_battery_estimator_compensation():
    """Test that the estimator reduces factor when responsiveness drops."""
    config = BatteryConfig(baseline_samples=10, min_pwm=10.0)
    estimator = BatteryEstimator(config=config)

    # 1. Establish Baseline (Ratio=2.0)
    for _ in range(10):
        estimator.update(50, 100)

    # 2. Simulate Voltage Drop (Responsiveness = 1.0 -> 50% drop)
    # Factor smoothing is slow (0.01), so loop many times
    for _ in range(500):
        estimator.update(50, 50) # Accel 50 / PWM 50 = 1.0

    # Baseline=2.0, Current=1.0. Ratio should approach 0.5.
    # Factor should drop below 1.0
    assert estimator.compensation_factor < 0.9
    assert estimator.compensation_factor > 0.4

def test_battery_estimator_deadzone():
    """Test that small PWMs are ignored."""
    config = BatteryConfig(baseline_samples=10, min_pwm=20.0)
    estimator = BatteryEstimator(config=config)

    # 1. Try to update with small PWM
    factor = estimator.update(10, 100)

    # Should assume 1.0 and not increment samples
    assert factor == 1.0
    assert estimator.samples_collected == 0

def test_battery_estimator_smoothing():
    """Test that the factor changes smoothly, not instantly."""
    config = BatteryConfig(baseline_samples=10, min_pwm=10.0)
    estimator = BatteryEstimator(config=config)

    # Baseline
    for _ in range(10):
        estimator.update(50, 100)

    initial_factor = estimator.compensation_factor

    # Single bad sample
    estimator.update(50, 0) # 0 responsiveness

    # Should not crash the factor instantly due to smoothing
    assert abs(estimator.compensation_factor - initial_factor) < 0.05
