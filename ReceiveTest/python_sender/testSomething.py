import random

vals = [0] * 9

for i in range(100):
    mid = 5
    height = 10
    while height > 0:
        rand = random.randint(0,1)
        if rand == 0 and mid > 0:
            mid -= 1
        elif rand == 1 and mid < 8:
            mid += 1
        height -= 1
    vals[mid] += 1

print(vals)
