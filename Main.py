from Graph import VertexType, Vertex, Edge, Matching
from math import cos, sin, pi
import numpy as np
import time


def compute_flip_path(n_points, agents, tasks, approx_ratio, verbose=False):

    # Compute the maximum cost of an optimal matching, which occurs at t=0.5
    max_opt_val = 0
    for i in range(n_points):
        max_opt_val += Edge(agents[i], tasks[i]).length(0.5)
    B = approx_ratio*max_opt_val

    # min_lengths[i][j] represents the min length of the edge between agents[i] and tasks[j]
    # only used to filter out "bad" matchings in generate_matchings
    min_lengths = []
    for i in range(len(agents)):
        min_lengths_agent = []
        for j in range(len(tasks)):
            min_lengths_agent.append(Edge(agents[i], tasks[j]).min_length(0,1))
        min_lengths.append(min_lengths_agent)

    # Compute matchings (nodes of the flip graph)
    def generate_matchings(agents, tasks, i, selected_tasks, w, w_max, matchings):
        """Recursively generates matchings, already filters out matchings whose cost is too large"""
        if w > w_max + 0.001:
            # sum of min lenghts already results in approximation factor worse than goal, don't include
            return 
        if i == len(agents):
            matchings.append(Matching(agents, tasks, selected_tasks))
            return
        for j in range(len(tasks)):
            if j not in selected_tasks:
                generate_matchings(agents, tasks, i+1, selected_tasks + [j], w + min_lengths[i][j], w_max, matchings)
        return

    # Compute edges of the flip graph
    def compute_edges(matchings, intervals):
        # Variables used for tracking number of edges to measure effectiveness of filtering
        n_flip_edges = 0    # Number of candidate edges in flip graph (intervals might not overlap)
        n_edges = 0         # Number of actual edges in generated graph (intervals do overlap)

        # Adjacency dictionary for flip graph
        neighbours = dict()
        for i in range(len(matchings)):
            neighbours[i] = []

        for i in range(len(matchings)):
            for j in range(i + 1, len(matchings)):
                # Check if it is possible to transition using a flip
                n_changes = 0
                k = 0
                while n_changes <= 2 and k < n_points:
                    if matchings[i].assignment[k] != matchings[j].assignment[k]:
                        n_changes += 1
                    k += 1
                if n_changes == 2:
                    # Can transition using flip, add edge
                    n_flip_edges += 1

                    # Check if the time intervals of the matchings intersect
                    i_s, i_e = intervals[i]
                    j_s, j_e = intervals[j]
                    if not (i_e < j_s or j_e < i_s):
                        # They intersect, add edge
                        neighbours[i].append(j)
                        neighbours[j].append(i)
                        n_edges += 1
    
        return neighbours, n_edges, n_flip_edges

    # Search for a path through the flip graph from source to goal along which t is non-decreasing.
    def dijkstra(matchings, intervals, neighbours, source, goal):
        min_t = [np.inf]*len(matchings)
        min_t[source] = 0
        visited = [False]*len(matchings)
        prev = [None]*len(matchings)
        prev[source] = source

        for _ in range(len(matchings)):
            # find matching which can be reached the earliest
            smallest = np.inf
            for m in range(len(matchings)):
                if not visited[m] and min_t[m] <= smallest:
                    smallest = min_t[m]
                    earliest_m = m
            
            # If either reached goal, or no more new reachable matchings, return
            if earliest_m == goal or smallest > 1:
                return min_t, prev

            for m in neighbours[earliest_m]:
                if not visited[m]:
                    em_s, em_e = intervals[earliest_m]
                    m_s, m_e = intervals[m]
                    min_shared_t = max(em_s, m_s)
                    # Check if we dont arrive after interval
                    if min_t[earliest_m] > m_e:
                        min_shared_t = np.inf
                    else:
                        min_shared_t = max(min_shared_t, min_t[earliest_m])
                    
                    if min_shared_t < min_t[m]:
                        min_t[m] = min_shared_t
                        prev[m] = earliest_m
            
            visited[earliest_m] = True

    time_spent = dict()
    t_start = time.time()

    t0 = t_start
    if verbose: print("Generating matchings...")
    matchings = []
    generate_matchings(agents, tasks, 0, [], 0, B, matchings)
    time_spent["generating matchings"] = time.time() - t0
    if verbose: print(f"Finished in {time.time() - t0}s")

    t0 = time.time()
    if verbose: print("Calculating intervals...")
    intervals = [m.get_intervals(approx_ratio) for m in matchings]
    flip_graph_vertices = []
    for i in range(len(intervals)):
        # For each interval of a matching add a copy
        for _ in intervals[i]:
            flip_graph_vertices.append(matchings[i])
    # Flatten intervals
    intervals = [i for m_intervals in intervals for i in m_intervals]
    time_spent["calculating intervals"] = time.time() - t0
    if verbose: print(f"Finished in {time.time() - t0}s")

    t0 = time.time()
    if verbose: print("Computing edges...")
    neighbours, n_edges, n_flip_edges = compute_edges(flip_graph_vertices, intervals)
    time_spent["computing edges"] = time.time() - t0
    if verbose: print(f"Finished in {time.time() - t0}s")

    t0 = time.time()
    if verbose: print("Identifying source and goal...")
    for i in range(len(flip_graph_vertices)):
        if flip_graph_vertices[i].assignment == sorted(flip_graph_vertices[i].assignment):
            source = i
            break
    for i in range(len(flip_graph_vertices)):
        sorted_m = sorted(flip_graph_vertices[i].assignment)
        if flip_graph_vertices[i].assignment == sorted_m[-1:] + sorted_m[:-1]:
            goal = i
            break
    time_spent["finding source and goal"] = time.time() - t0
    if verbose: print(f"Finished in {time.time() - t0}s")

    t0 = time.time()
    if verbose: print(f"Running Dijkstra's...")
    min_t, prev = dijkstra(flip_graph_vertices, intervals, neighbours, source, goal)
    time_spent["dijkstra's"] = time.time() - t0
    if verbose: print(f"Finished in {time.time() - t0}s")

    time_spent["total"] = time.time() - t_start

    t_done = min_t[goal]
    flip_path = []
    if t_done <= 1:
        i = goal
        flip_path.insert(0, (t_done, matchings[goal]))
        while prev[i] != i:
            i = prev[i]
            flip_path.insert(0, (min_t[i], matchings[i])) 


    if verbose: 
        print("#============Summary============#")
        print(f"#matchings: {len(matchings)}")
        print(f"#flip-edges: {n_flip_edges}")
        print(f"#edges: {n_edges}")
        print(f"t-done: {t_done}")
    
    return t_done, flip_path, time_spent

def find_approx_ratio(n_points, approx_ratio_ub, draw=False, verbose=False, acc=0.0001):
    phi = 0
    points = []
    for _ in range(n_points):
        points.append((cos(phi), sin(phi)))
        phi += 2*pi/n_points

    agents = []
    tasks = []
    for i in range(len(points)):
        agents.append(Vertex(points[i], points[i], VertexType.AGENT))
        tasks.append(Vertex(points[i], points[(i+1)%len(points)], VertexType.TASK))

    ub = approx_ratio_ub
    lb = 1
    search_iterations = 0
    time_spent = None
    while ub - lb > acc:
        approx_ratio = lb + (ub - lb)/2
        t, fp, ts = compute_flip_path(n_points, agents, tasks, approx_ratio)
        if t > 1:
            lb = approx_ratio
        else:
            ub = approx_ratio
            flip_path = fp
        search_iterations += 1

        if not time_spent:
            time_spent = ts
        else:
            for x in ts.keys():
                time_spent[x] += ts[x]
        if verbose: print(f"Iteration {search_iterations}   -   ub-lb: {ub-lb}   -   lb: {lb}")

    if verbose:
        print(f"#========Summary========#")
        print(f"#Search iterations: {search_iterations}")
        print(f"Approx Ratio found: {lb}")

    if draw:
        # Draw
        from PIL import Image, ImageDraw   
        w, h = 256, 256
        scaling_vector = np.array([128, 128])
        point_size = np.array([3,3])
        i = 0
        for t, m in flip_path:
            img = Image.new("RGB", (w, h))
            img1 = ImageDraw.Draw(img)
            for j in range(len(m.assignment)):
                u = scaling_vector + scaling_vector*m.agents[j].location(t)
                v = scaling_vector + scaling_vector*m.tasks[m.assignment[j]].location(t)
                img1.line([tuple(u), tuple(v)], fill ="white", width = 3)
                img1.ellipse([tuple(u - point_size), tuple(u + point_size)], fill="blue", width=2)
                img1.ellipse([tuple(v - point_size), tuple(v + point_size)], fill="red", width=3) 
            img.save(f"images/matching_{i}_t{t}.png")
            i += 1
    
    return lb, time_spent


