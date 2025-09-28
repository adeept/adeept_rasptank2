import time

def wait_for(predicate, timeout_seconds=10, interval=0.1):
    """
    Poll `predicate()` until it returns True or `timeout` seconds elapse.
    Raises TimeoutError on timeout.
    """
    start = time.monotonic()
    while True:
        if predicate():
            return True
        if time.monotonic() - start > timeout_seconds:
            print(f"condition not met within {timeout_seconds} seconds")
            raise TimeoutError(f"condition not met within {timeout_seconds} seconds")
        time.sleep(interval)