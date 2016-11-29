import time


def something(duration=0.000001):
    """
    Function that needs some serious benchmarking.
    """
    time.sleep(duration)

    return 123


def test_my_stuff(benchmark):
    # benchmark something
    result = benchmark.pedantic(something, iterations=10, rounds=10)
    print result

    # Extra code, to verify that the run completed correctly.
    # Sometimes you may want to check the result
    assert result == 123