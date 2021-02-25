import time


def get_time(start_time):
    time.sleep(5)
    current_time = time.time() - start_time
    return current_time

start = time.perf_counter()
print(start)
print(get_time(time.time()))
end = time.perf_counter()
print(start - end)
