from Main import find_approx_ratio
from math import pi, ceil
import numpy as np
import matplotlib.pyplot as plt

n_max = 12
ratios = []
time_spent = None
times_n = []
for i in range(1, n_max+1):
    r, ts = find_approx_ratio(i, 1 + 2/pi, draw=i==n_max)
    ratios.append(r)
    print(f"Computed ratio for n={i} - r={r}")
    times_n.append(ts["total"])
    if not time_spent:
        time_spent = ts
    else:
        for x in ts.keys():
            time_spent[x] += ts[x]

print(f"Ratios: {ratios}")
print(f"Times: {times_n}")
print(f"=====Time per task=====")
total_time = time_spent["total"]
for x in time_spent.keys():
    print(f"{x} - {ceil(time_spent[x])//60}m{ceil(time_spent[x])%60}s - {time_spent[x]/total_time*100}%")

xpoints = np.array(range(1,n_max+1))
ypoints = np.array(ratios)

plt.plot(xpoints, ypoints)
plt.show()