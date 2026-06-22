import random

vals = [0] * 9

for _ in range(100000):
    mid = 5

    for _ in range(10):
        if random.randint(0, 1) == 0:
            if mid > 0:
                mid -= 1
        else:
            if mid < 8:
                mid += 1

    vals[mid] += 1

print(vals)
