## Computing a lowerbound on the topological stability ratio of balanced BEM and EM where $n$ is even in 2 dimensions
This repository contains the code used to compute a lowerbound on the topological stability ratio of kinetic BEM and kinetic EM in two dimensions. We give a brief explanation of how the code for BEM corresponds to the pseudocode found in the thesis. The code for EM contains the same functions but has be slightly altered to work for EM instead of BEM.

The function `find_approx_ratio` (main.py, line 180) performs the binary search on the approximation ratio. In each search iteration it calls the function `compute_flip_path` which corresponds to the algorithm given in pseudocode. The function `compute_flip_path` does the following:
- It first generates all matchings (BEM/main.py, line 116), here we already filter out matchings which we know will never have an approximation ratio smaller than or equal to `approx_ratio` ($r$). This corresponds to line 1 of the pseudocode.
- Next, it computes the time intervals for each matching (BEM/main.py, line 122-129), which corresponds to lines 2-10 of the pseudocode.
- Next, it computes the edges of the flip graph (BEM/main.py, line 135), which corresponds to lines 11-16 of the pseudocode.
- Finally, the algorithm checks whether there exists a path from $M_1$ to $M_2$ using Dijkstra's (BEM/main.py, line 141-156), which corresponds to lines 17-18 of the pseudocode.

The table below shows the lowerbounds computed by the algorithm for $n = 2, 3, \dots, 11$ for BEM and for $n = 4, 6, \dots, 18$ for EM truncated to four decimal places.

| #points in input| lowerbound (BEM) | lowerbound (EM) |
|-----|------------|------------|
| 4 | 1 | 1 |
| 6 | 1.2373 | 1.2373 |
| 8 | 1.2967 | 1.2967 |
| 10 | 1.4075 | 1.4075 |
| 12 | 1.4232 | 1.4232 |
| 14 | 1.4744 | 1.4744 |
| 16 | 1.4808 | 1.4808 |
| 18 | 1.5101 | 1.5101 |
| 20 | 1.5132 | - |
| 22 | 1.5322 | - |
