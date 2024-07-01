from enum import Enum
import numpy as np
from numpy import linalg as LA


class Vertex:
    """Represents a vertex which moves along a linear trajectory"""

    def __init__(self, p_start, p_end):
        self.p_start = np.array(p_start)
        self.p_end = np.array(p_end)

    def location(self, t):
        return self.p_start + (self.p_end-self.p_start)*t

class Edge:
    """Represents an edge between two vertices"""

    def __init__(self, u, v):
        self.u = u
        self.v = v

    def length(self, t):
        u = self.u.location(t)
        v = self.v.location(t)
        return LA.norm(u-v, 2)

    def min_length(self, t_start, t_end):
        """Compute the minimum length an edge will have for t in [t_start,t_end]"""
        u_start = self.u.p_start
        u_end = self.u.p_end
        v_start = self.v.p_start
        v_end = self.v.p_end
        b = u_start - v_start
        a = u_end - u_start - (v_end - v_start)
        min_t = -np.dot(a,b) / LA.norm(a, 2)**2
        if t_start <= min_t and min_t <= t_end:
            return self.length(min_t)
        else:
            return min(self.length(t_start), self.length(t_end))

class Matching:
    """"Represents a matching"""

    def __init__(self, agents, tasks, assignment):
        self.agents = agents
        self.tasks = tasks
        self.assignment = assignment

    def get_intervals(self, max_approx_ratio):
        """
        Computes time intervals at which the matching has an approximation ratio <= max_approx_ratio
        by computing a lower bound on the approximation ratio for fixed time intervals.
        """

        # Construct list of edges of this matching
        edges = []
        for i in range(len(self.assignment)):
            edges.append(Edge(self.agents[i], self.tasks[self.assignment[i]]))

        # Construct list of edges of optimal matching (for t in [0, 0.5])
        edges_opt= []
        for i in range(len(self.agents)):
            edges_opt.append(Edge(self.agents[i], self.tasks[i]))
        # Maximum value of opt (occurs at t=0.5)
        opt_max = sum([edge.length(0.5) for edge in edges_opt])

        # Compute intervals for which cost of matching is below max_approx_ratio
        steps = 100
        below_ratio = False
        interval_start = 0
        intervals = []
        step = 0
        while step < steps:
            t = step/steps
            # Compute a lowerbound on the cost of the matching
            min_cost = sum([edge.min_length(t, t+1/steps) for edge in edges])
            # Compute an upperbound on the cost of optimal matching
            if t+1/steps <= 0.5:
                max_opt = 2*(t+1/steps)*opt_max
            elif t >= 0.5:
                max_opt = 2*(1-t)*opt_max
            else:
                max_opt = opt_max
            
            # Compute minimum achievable approximation ratio of matching in this time step
            # Due to rough bounds, this can be smaller than 1
            approx_ratio = min_cost / max_opt
            
            if approx_ratio <= max_approx_ratio and not below_ratio:
                # Dropped below the ratio, start of new interval
                interval_start = max(0, t - 1/steps)
                below_ratio = True
            if approx_ratio > max_approx_ratio and below_ratio:
                # Increased above the ratio, end of interval
                intervals.append((interval_start, min(1, t + 2/steps)))
                below_ratio = False
            step += 1
 
        if below_ratio:
            intervals.append((interval_start, 1))

        return intervals