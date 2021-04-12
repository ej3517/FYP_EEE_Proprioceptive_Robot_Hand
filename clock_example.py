import time


def get_time(start_time):
    time.sleep(2)
    current_time = time.time() - start_time
    return current_time

start = time.time()
print(start)

print(get_time(time.time()))
end = time.perf_counter()
print(start - end)
