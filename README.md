This repository contains the code used to compute a lowerbound on the topological stability ratio of kinetic Euclidean matching in two dimensions. We give a short explanation of how the code corresponds to the pseudocode found in the thesis.

The function `find_approx_ratio` (main.py, line 180) performs the binary search on the approximation ratio. In each search iteration it calls the function `compute_flip_path` which corresponds to the algorithm given in pseudocode. The function `compute_flip_path` does the following:
- It first generates all matchings (main.py, line 116), here we already filter out matchings which we know will never have an approximation ratio smaller than or equal to `approx_ratio` ($r$). This corresponds to line 1 of the pseudocode.
- Next, it computes the time intervals for each matching (main.py, line 122-129), which corresponds to lines 2-10 of the pseudocode.
- Next, it the edges of the flip graph (main.py, line 145), which corresponds to lines 11-16 of the pseudocode.
- Finally, the algorithm checks whether there exists a path from $M_1$ to $M_2$ using Dijkstra's (main.py, line 141-156), which corresponds to lines 17-18 of the pseudocode.

