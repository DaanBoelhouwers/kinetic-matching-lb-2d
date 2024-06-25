from enum import Enum
import numpy as np
from numpy import linalg as LA


class VertexType(Enum):
    AGENT = 1
    TASK = 2

class Vertex:
    """Represents a vertex which moves along a linear trajectory"""

    def __init__(self, p_start, p_end, vertex_type):
        self.p_start = np.array(p_start)
        self.p_end = np.array(p_end)
        self.vertex_type = vertex_type

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
        by computing approximation at fixed time steps.
        """

        # Construct list of edges of this matching
        edges = []
        for i in range(len(self.assignment)):
            edges.append(Edge(self.agents[i], self.tasks[self.assignment[i]]))

        # Construct list of edges of optimal matching (for t in [0, 0.5])
        edges_opt= []
        for i in range(len(self.agents)):
            edges_opt.append(Edge(self.agents[i], self.tasks[i]))
        # Maximum value of opt
        opt_max = sum([edge.length(0.5) for edge in edges_opt])

        # Note that at t = 0 and t = 1 all matchings (except for the optimal matchings) have an approx ratio worse than max_approx_ratio
        # Therefore, below_ratio starts at False
        steps = 100
        below_ratio = False
        interval_start = 0
        intervals = []
        step = 0
        while step <= steps:
            t = step/steps
            cost = sum([edge.length(t) for edge in edges])
            # Due to cost of opt changing linearly, we can compute it by interpolating between min and max values.
            cost_opt = min(2*t*opt_max, 2*(1-t)*opt_max)
            if cost_opt < 0.0001:
                approx_ratio = np.inf
            else:
                approx_ratio = cost / cost_opt
            
            if approx_ratio <= max_approx_ratio and not below_ratio:
                # Dropped below the ratio, start of new interval
                interval_start = t - 1/steps
                below_ratio = True
            if approx_ratio > max_approx_ratio and below_ratio:
                # Increased above the ratio, end of interval
                intervals.append((interval_start, t))
                below_ratio = False

            # Attempt at speeding up by increasing step size, has little effect on run time in practice
            step_size = 1
            if approx_ratio == np.inf:
                pass
            elif below_ratio:
                while 2*step_size < (max_approx_ratio - approx_ratio) / (4 * opt_max):
                    step_size *= 2
            else: # above ratio
                while 2*step_size < (approx_ratio - max_approx_ratio) / (4 * opt_max):
                    step_size *= 2
            step += step_size

        return intervals