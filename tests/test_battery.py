from balance_bot.battery import BatteryEstimator

def test_battery_estimator_baseline():
    """Test that the estimator establishes a baseline correctly."""
    estimator = BatteryEstimator(baseline_samples=10, min_pwm=10.0)

    # 1. Establish Baseline
    # Simulate constant "Responsiveness" = 2.0 (Accel=100, PWM=50)
    for _ in range(10):
        factor = estimator.update(pwm=50, angular_accel=100, loop_delta_time=0.01)
        assert factor == 1.0

    assert estimator.baseline_responsiveness == 2.0

def test_battery_estimator_compensation():
    """Test that the estimator reduces factor when responsiveness drops."""
    estimator = BatteryEstimator(baseline_samples=10, min_pwm=10.0)

    # 1. Establish Baseline (Ratio=2.0)
    for _ in range(10):
        estimator.update(50, 100, 0.01)

    assert estimator.baseline_responsiveness == 2.0

    # 2. Simulate Low Battery
    # Now responsiveness drops to 1.0 (Accel=50, PWM=50)
    # Ratio = 1.0 / 2.0 = 0.5
    # The factor should slowly drift towards 0.5

    for _ in range(50):
        factor = estimator.update(50, 50, 0.01)

    # Should be significantly less than 1.0
    assert factor < 0.9
    assert factor >= 0.5 # clamped

def test_battery_estimator_deadzone():
    """Test that small PWMs are ignored."""
    estimator = BatteryEstimator(baseline_samples=10, min_pwm=20.0)

    # 1. Try to update with small PWM
    factor = estimator.update(10, 100, 0.01)

    # Should not count towards samples
    assert estimator.samples_collected == 0
    assert factor == 1.0

def test_battery_estimator_smoothing():
    """Test that the factor changes smoothly, not instantly."""
    estimator = BatteryEstimator(baseline_samples=10, min_pwm=10.0)

    # Baseline
    for _ in range(10):
        estimator.update(50, 100, 0.01)

    # Instant drop in responsiveness
    # Accel=50, PWM=50 => Ratio=1.0 (vs Baseline 2.0) => Target 0.5

    # One step
    factor = estimator.update(50, 50, 0.01)

    # Should have changed only slightly (0.01 * target + 0.99 * current)
    # 0.01 * 0.5 + 0.99 * 1.0 = 0.005 + 0.99 = 0.995
    assert factor < 1.0
    assert factor > 0.9 # Shouldn't jump to 0.5
