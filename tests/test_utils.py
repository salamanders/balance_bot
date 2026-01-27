import time
from balance_bot.utils import clamp, RateLimiter, ComplementaryFilter

def test_clamp():
    assert clamp(10, 0, 5) == 5.0
    assert clamp(-10, 0, 5) == 0.0
    assert clamp(3, 0, 5) == 3.0
    assert clamp(0, 0, 5) == 0.0
    assert clamp(5, 0, 5) == 5.0

def test_rate_limiter():
    freq = 50
    limiter = RateLimiter(freq)

    start = time.monotonic()
    # Burn first cycle reset
    limiter.reset()

    # Simulate a fast loop
    for _ in range(5):
        time.sleep(0.001) # Work takes 1ms
        limiter.sleep()   # Should sleep rest of 20ms

    end = time.monotonic()
    elapsed = end - start

    # 5 periods of 20ms = 0.1s
    assert elapsed >= 0.1
    # Should not be too slow either (allow 20% overhead)
    assert elapsed < 0.12

def test_complementary_filter():
    alpha = 0.98
    cf = ComplementaryFilter(alpha)

    assert cf.angle == 0.0

    # Update: rate=0, angle=0 => result=0
    res = cf.update(0.0, 0.0, 0.1)
    assert res == 0.0

    # One step update
    # new_angle = 10, rate = 10, dt = 0.1
    # gyro_part = 0 + 10 * 0.1 = 1.0
    # accel_part = 10.0
    # result = 0.98 * 1.0 + 0.02 * 10.0 = 0.98 + 0.2 = 1.18
    res = cf.update(10.0, 10.0, 0.1)
    assert abs(res - 1.18) < 1e-9

    # Internal state should update
    assert cf.angle == res
