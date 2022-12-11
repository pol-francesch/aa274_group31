import numpy as np
import matplotlib.pyplot as plt

from utils.grids import DetOccupancyGrid2D, StochOccupancyGrid2D

def line_line_intersection(l1, l2):
    """Checks whether or not two 2D line segments `l1` and `l2` intersect.

    Args:
        l1: A line segment in 2D, i.e., an array-like of two points `((x_start, y_start), (x_end, y_end))`.
        l2: A line segment in 2D, i.e., an array-like of two points `((x_start, y_start), (x_end, y_end))`.

    Returns:
        `True` iff `l1` and `l2` intersect.
    """

    def ccw(A, B, C):
        return np.cross(B - A, C - A) > 0

    A, B = np.array(l1)
    C, D = np.array(l2)
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

def plot_line_segments(segments, **kwargs):
    plt.plot([x for tup in [(p1[0], p2[0], None) for (p1, p2) in segments] for x in tup],
             [y for tup in [(p1[1], p2[1], None) for (p1, p2) in segments] for y in tup], **kwargs)

class RRTStar():
    """ 
        Represents a motion planning problem to be solved using the RRT* algorithm
        Source: https://arxiv.org/pdf/1105.1186.pdf
    """

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, obstacles, free_motion_step=None):
        self.statespace_lo = np.array(statespace_lo)    # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = np.array(statespace_hi)    # state space upper bound (e.g., [5, 5])
        self.x_init = np.array(x_init)                  # initial state
        self.x_goal = np.array(x_goal)                  # goal state
        self.path = None                                # the final path as a list of states

        # Obstacles need to work on both RRT and A* maps
        self.obstacles = obstacles
        self.free_motion_step = free_motion_step

    def occupancy_grid_to_obs(self, occupancy):
        """
        Constructs the set of obstacles that RRT expects from
        the set of obstacles that A* gives it.

        Inputs:
            occupancy: A DetOccupancyGrid2D object or similar
        Output:
            obstacles
        """
        # Occupancy grid is defined as corners of obstacles
        # We need lines
        # Assume rectangular obstacles
        maze = []
        for obstacle in occupancy.obstacles:
            line1 = ((obstacle[0][0], obstacle[0][1]), ((obstacle[0][0], obstacle[1][1])))
            line2 = ((obstacle[0][0], obstacle[0][1]), ((obstacle[1][0], obstacle[0][1])))
            line3 = ((obstacle[1][0], obstacle[1][1]), ((obstacle[0][0], obstacle[1][1])))
            line4 = ((obstacle[1][0], obstacle[1][1]), ((obstacle[1][0], obstacle[0][1])))
            maze += [line1, line2, line3, line4]
        
        # Add in the edges (assuming (0,0) is at the bottom left corner)
        edge1 = ((0,0), (0,occupancy.height))
        edge2 = ((0,0), (occupancy.width, 0))
        edge3 = ((occupancy.width, occupancy.height), (0,occupancy.height))
        edge4 = ((occupancy.width, occupancy.height), (occupancy.width,0))
        maze += [edge1, edge2, edge3, edge4]

        return np.array(maze)

    def solve(self, eps=1.0, max_iters=1000, goal_bias=0.05, search_radius=2.0, plot=False):
        """
        Constructs an RRT rooted at self.x_init with the aim of producing a
        dynamically-feasible and obstacle-free trajectory from self.x_init
        to self.x_goal.

        Inputs:
            eps: maximum steering distance
            max_iters: maximum number of RRT iterations (early termination
                is possible when a feasible solution is found)
            goal_bias: probability during each iteration of setting
                x_rand = self.x_goal (instead of uniformly randly sampling
                from the state space)
            search radius: radius that spans area in which we check for
                nearby states
        Output:
            Whether we succed
        """
        # Prep work
        success = False
        state_dim = len(self.x_init)

        V = np.zeros((max_iters+1, state_dim))          # states that have been added to RRT
        V[0,:] = self.x_init                            # RRT is rooted at self.x_init
        P = -np.ones(max_iters+1, dtype=int)            # parents of each state
        C = np.zeros(max_iters+1, dtype=float)          # cost of each state

        n = 1                                           # the current size of the RRT (states accessible as V[range(n),:])

        # Draw out all the nodes
        while n < max_iters:
            x_rand = self.sample_free(goal_bias, state_dim)
            x_nearest = self.find_nearest(V[0:n,:], x_rand)
            x_new, added_cost = self.steer_towards(x_nearest, x_rand, eps)

            if self.is_free_motion(self.obstacles, x_nearest, x_new):
                i_near = self.find_nearby_states(V[0:n,:], x_new, self.obstacles, eps, search_radius)
                V[n, :] = x_new

                if i_near:
                    # Connect along minimum-cost path
                    parent, cost = self.choose_parent(V[0:n,:], C[0:n], x_new, i_near)
                    P[n] = parent; C[n] = cost
                
                    # Rewire the tree
                    n = n + 1
                    P[0:n], C[0:n] = self.rewire(V[0:n,:], P[0:n], C[0:n], x_new, i_near)
                else:
                    parent_idx = np.where((V[0:n, :] == x_nearest).all(axis=1))[0][0]
                    P[n] = parent_idx; C[n] = C[parent_idx] + added_cost

                    n = n + 1
        
        # Here we do not stop when we've reached the goal since we want to
        # re-wire the tree as much as possible. So our last node is not 
        # our goal node. So we find the node that is closest to the goal.
        goal_parent = self.find_goal_parent(V, C, self.obstacles, eps)
        if goal_parent:
            success = True
            # Re-trace the path
            self.path = self.generate_path(V, P, goal_parent)
        
        # Plot the path
        if plot:
            plt.figure()
            self.plot_problem()
            self.plot_tree(V, P, color="blue", linewidth=.5, label="RRT tree", alpha=0.5)
            if success:
                self.plot_path(color="green", linewidth=2, label="Solution path")
            plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)
            plt.scatter(V[:n,0], V[:n,1])
            plt.savefig("test5.png")
        
        return success

    def sample_free(self, goal_bias, state_dim):
        '''
        Constructs a new node. The goal node is used based on the probability
        stated by the goal_bias.

        Inputs:
            goal_bias: probability during each iteration of setting
                x_rand = self.x_goal (instead of uniformly randly sampling
                from the state space)
        Output:
            x_rand (np array(state_dim,)): new state
        '''
        if np.random.uniform(0.0, 1.0, None) < goal_bias:
            x_rand = self.x_goal
        else:
            x_rand = np.zeros((state_dim,))
            for i in range(0, state_dim):
                x_rand[i] = np.random.uniform(self.statespace_lo[i], self.statespace_hi[i], None)
        
        return x_rand
    
    def find_nearest(self, V, x):
        """
        Given a list of states V and a query state x, returns the index (row)
        of V such that the steering distance (subject to robot dynamics) from
        V[i] to x is minimized

        Inputs:
            V: list/np.array of states ("samples")
            x - query state
        Output:
            Integer index of nearest point in V to x
        """
        return V[np.argmin(np.linalg.norm(V - x, axis=-1)),:]

    def steer_towards(self, x1, x2, eps):
        """
        Steers from x1 towards x2 along the shortest path (subject to robot
        dynamics). Returns x2 if the length of this shortest path is less than
        eps, otherwise returns the point at distance eps along the path from
        x1 to x2.

        Inputs:
            x1: start state
            x2: target state
            eps: maximum steering distance
        Output:
            State (numpy vector) resulting from bounded steering
            Added cost to get to new state
        """
        if np.linalg.norm(x2 - x1) <= eps:
            x = x2
        else:
            x = x1 + eps * (x2 - x1) / np.linalg.norm(x2 - x1)
        
        added_cost = np.linalg.norm(x1 - x, axis=-1)

        return x, added_cost
    
    def is_free_motion(self, obstacles, x1, x2):
        """
        Subject to the robot dynamics, returns whether a point robot moving
        along the shortest path from x1 to x2 would collide with any obstacles
        (implemented as a "black box")

        Inputs:
            obstacles: list/np.array of line segments ("walls")
            x1: start state of motion
            x2: end state of motion
        Output:
            Boolean True/False
        """
        if isinstance(obstacles, DetOccupancyGrid2D) or isinstance(obstacles, StochOccupancyGrid2D):
            # Check along the line if there is an obstacle
            # First create the points along the line
            line = np.array([np.linspace(x1[0],x2[0],self.free_motion_step),
                             np.linspace(x1[1],x2[1],self.free_motion_step)]).T

            # Use built-in is_free function
            for point in line:
                if not obstacles.is_free(point):
                    return False
            
            return True
        else:
            motion = np.array([x1, x2])
            for line in obstacles:
                if line_line_intersection(motion, line):
                    return False
            return True

    def find_nearby_states(self, V, x, obstacles, eps, search_radius):
        '''
        Given a list of states V and a query state x, returns the indeces (row)
        of V such that the steering distance (subject to robot dynamics) from
        V[i] to x is less than a parameter

        Inputs:
            V: list/np.array of states ("samples")
            x: query state
            obstacles: list/np.array of line segments ("walls")
            eps: maximum steering distance
            search radius: radius that spans area in which we check for
                nearby states
        Output:
            List of indeces of nearby states in V to x
        '''
        # TODO: Parameter calculation here may be off
        n = V.shape[0]
        r = min(search_radius * np.sqrt(np.log(n)/n), eps)
        # r = search_radius * np.sqrt(np.log(n)/n)
        
        distances = np.linalg.norm(V - x, axis=-1)

        # Indeces of distances that meet the maximum distance requirement and generate obstacle-free paths
        nearby_states_idxs = [i for i in range(distances.shape[0]) if distances[i] <= r and 
                                                self.is_free_motion(obstacles, V[i,:], x)]

        return nearby_states_idxs
    
    def choose_parent(self, V, C, x, i_near):
        '''
        Given a list of states V, a query state x and the indeces of the 
        states near x, returns the index of the parent of x and the cost to 
        get to x

        Inputs:
            V: list/np.array of states ("samples")
            C: list/np.array of costs to get to states
            x: query state
            i_near: list of indeces of nearby states in V to x
        Output:
            index of parent of x
            cost to get to x
        '''
        if len(i_near) == 0:
            print("This should never happen")
            return -1, np.inf
        
        distances = np.linalg.norm(V[i_near,:] - x, axis=-1)
        costs     = C[i_near] + distances

        # Parent is state with smallest cost
        idx = np.argmin(costs)

        parent = i_near[idx]
        cost   = costs[idx]

        return parent, cost

    def rewire(self, V, P, C, x, i_near):
        '''
        Rewire the RRT to ensure the vertices are 
        reached with a minimum-cost path

        Inputs:
            V: list/np.array of states ("samples")
            P: list/np.array of parents of states
            C: list/np.array of costs to get to states
            x: query state
            i_near: list of indeces of nearby states in V to x
        Output:
            Rewired P and C
        '''
        n = V.shape[0]-1        # index of x
        for i in i_near:
            nearby_state = V[i]

            # What if we go from the new state to the neighboring states
            old_cost = C[i]
            new_cost = C[n] + np.linalg.norm(x-nearby_state)

            if old_cost > new_cost:
                C[i] = new_cost
                P[i] = n
        
        return P, C
    
    def find_goal_parent(self, V, C, obstacles, eps):
        # Finds valid node closest to goal and treats it as the goal parent
        distances = np.linalg.norm(V - self.x_goal, axis=-1)

        nearby_states_idxs = [i for i in range(distances.shape[0]) if distances[i] <= eps and 
                                        self.is_free_motion(obstacles, V[i,:], self.x_goal)]

        if nearby_states_idxs:
            costs     = distances[nearby_states_idxs] + C[nearby_states_idxs]
            return nearby_states_idxs[np.argmin(costs)]
        
        return None
        
    def generate_path(self, V, P, goal_parent):
        # generate the path
        path = [self.x_goal]

        n = goal_parent
        while n != -1:
            path.append(V[n,:])
            n = P[n]
        
        path.reverse()

        return np.array(path)

    def plot_problem(self):
        if isinstance(self.obstacles, DetOccupancyGrid2D):
            self.obstacles = self.occupancy_grid_to_obs(self.obstacles)
        plot_line_segments(self.obstacles, color="red", linewidth=2, label="obstacles")
        plt.scatter([self.x_init[0], self.x_goal[0]], [self.x_init[1], self.x_goal[1]], color="green", s=30, zorder=10)
        plt.annotate(r"$x_{init}$", self.x_init[:2] + [.2, 0], fontsize=16)
        plt.annotate(r"$x_{goal}$", self.x_goal[:2] + [.2, 0], fontsize=16)
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.03), fancybox=True, ncol=3)
        plt.axis('scaled')

    def plot_tree(self, V, P, **kwargs):
        plot_line_segments([(V[P[i],:], V[i,:]) for i in range(V.shape[0]) if P[i] >= 0], **kwargs)

    def plot_path(self, **kwargs):
        path = np.array(self.path)
        plt.plot(path[:,0], path[:,1], **kwargs)