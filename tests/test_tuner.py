from balance_bot.tuner import ContinuousTuner

def test_oscillation_detection():
    # buffer_size=10 for faster testing
    tuner = ContinuousTuner(buffer_size=10)

    # Fill with oscillating data: 5, -5, 5, -5...
    # We need enough samples to fill buffer and trigger logic
    kp = 0
    kd = 0
    for i in range(15):
        val = 10.0 if i % 2 == 0 else -10.0
        kp, ki, kd = tuner.update(val)
        if kp != 0:
            break

    # Expect Kp reduction
    assert kp < 0, "Should reduce Kp on oscillation"
    assert kd > 0, "Should increase Kd on oscillation"

def test_stability_detection():
    tuner = ContinuousTuner(buffer_size=10)

    kp = 0
    # Fill with stable small data
    for _ in range(15):
        kp, ki, kd = tuner.update(0.1)
        if kp != 0:
            break

    # Expect Kp increase
    assert kp > 0, "Should increase Kp when stable"

def test_steady_error_detection():
    tuner = ContinuousTuner(buffer_size=10)

    ki = 0
    # Fill with large steady error
    for _ in range(15):
        kp, ki, kd = tuner.update(5.0)
        if ki != 0:
            break

    # Expect Ki increase
    assert ki > 0, "Should increase Ki on steady error"

def test_cooldown():
    tuner = ContinuousTuner(buffer_size=10)

    # Trigger tune
    last_kp = 0
    for i in range(15):
        val = 10.0 if i % 2 == 0 else -10.0
        kp, ki, kd = tuner.update(val)
        if kp != 0:
            last_kp = kp
            break

    assert last_kp != 0

    # Next call should be zero due to cooldown
    kp, ki, kd = tuner.update(10.0)
    assert kp == 0
    assert ki == 0
    assert kd == 0
