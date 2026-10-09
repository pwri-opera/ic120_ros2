import pytest
from ic120_navigation.message_rate_limiter import StampRateLimiter


def test_rate_is_limited_in_source_time_and_resets_on_clock_restart():
    limiter = StampRateLimiter(20)
    assert limiter.allow(10)
    assert not limiter.allow(10.02)
    assert not limiter.allow(10)
    assert limiter.allow(10.05)
    assert limiter.allow(0)
    assert not limiter.allow(.01)
    limiter.reset()
    assert limiter.allow(.01)


def test_zero_keeps_original_publication_behavior():
    limiter = StampRateLimiter(0)
    assert limiter.allow(0)
    assert limiter.allow(0)
    assert not limiter.allow(float('nan'))


@pytest.mark.parametrize('value', [-1, float('nan'), float('inf')])
def test_invalid_rates_are_rejected(value):
    with pytest.raises(ValueError):
        StampRateLimiter(value)
