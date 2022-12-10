import numpy as np

temp = np.array([[200, 262],
                [200, 263],
                [200, 264],
                [200, 265],
                [200, 266],
                [200, 267],
                [200, 268],
                [200, 269],
                [200, 270],
                [200, 271],
                [200, 272],
                [200, 273],
                [201, 273],
                [202, 273],
                [203, 273],
                [204, 273],
                [205, 273],
                [206, 273],
                [207, 273],
                [208, 273],
                [209, 273],
                [210, 273],
                [211, 252],
                [211, 253],
                [211, 254],
                [211, 255],
                [211, 256],
                [211, 257],
                [211, 258],
                [211, 259],
                [211, 260],
                [211, 261],
                [211, 262],
                [211, 273],
                [212, 252],
                [212, 273],
                [213, 252],
                [213, 273]])


lines = []
n = 0

while n < temp.shape[0]-2:
    # Try to make one line
    start = temp[n]
    line = [start]
    for i in range(n+1,temp.shape[0]):
        obs = temp[i]

        if start[0] == obs[0] and obs[1] == temp[i-1][1] + 1:
            line.append(obs)
        elif i == temp.shape[0]-1:
            n = i
            break
        else:
            n = i
            break

    if len(line) > 1:
        lines.append([line[0], line[-1]])

# y-lines
print(lines)

    def probabilistic_grid_to_obs(self, occupancy, resolution):
        """
        Constructs the set of obstacles that RRT expects from
        the set of obstacles that A* gives it.

        Inputs:
            occupancy: A StochOccupancyGrid2D object or similar
        Output:
            obstacles
        """
        obstacles_point_map = np.argwhere(occupancy.probs > occupancy.thresh)
        lines = []
        # print(obstacles_point_map)
        # x-lines
        n = 0
        while n < obstacles_point_map.shape[0]-2:
            # Try to make one line
            start = obstacles_point_map[n]
            line = [start]

            for i in range(n+1,obstacles_point_map.shape[0]):
                obs = obstacles_point_map[i]
                # print(start)
                # print(obs)
                if start[0] == obs[0] and obs[1] <= obstacles_point_map[i-1][1] + 15:
                    # print('line')
                    # print(line)
                    line.append(obs)
                elif i == obstacles_point_map.shape[0]-1:
                    n = i
                    break
                else:
                    n = i
                    break
            
            if len(line) > 1:
                # print('line')
                # print(line)
                # print()
                lines.append([line[0], line[-1]])
            break
        # print('lines')
        # print(lines)
        # exit()
        # y-lines
        n = 0
        while n < obstacles_point_map.shape[0]-2:
            # Try to make one line
            start = obstacles_point_map[n]
            line = [start]
            for i in range(n+1,obstacles_point_map.shape[0]):
                obs = obstacles_point_map[i]

                if start[1] == obs[1] and obs[0] <= obstacles_point_map[i-1][0] + 15:
                    line.append(obs)
                elif i == obstacles_point_map.shape[0]-1:
                    n = i
                    break
                else:
                    n = i
                    break

            if len(line) > 1:
                lines.append([line[0], line[-1]])

        # print(lines)
        return np.array(lines)*resolution
    

        def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, obstacles, resolution=None):
        self.statespace_lo = np.array(statespace_lo)    # state space lower bound (e.g., [-5, -5])
        self.statespace_hi = np.array(statespace_hi)    # state space upper bound (e.g., [5, 5])
        self.x_init = np.array(x_init)                  # initial state
        self.x_goal = np.array(x_goal)                  # goal state
        self.path = None                                # the final path as a list of states

        # We want compatability with both A* and RRT simulation
        # So we just need to check the type of obstacle:
        if isinstance(obstacles, DetOccupancyGrid2D):
            self.obstacles = self.occupancy_grid_to_obs(obstacles)
        elif isinstance(obstacles, StochOccupancyGrid2D):
            self.obstacles = self.probabilistic_grid_to_obs(obstacles, resolution)
        else:
            self.obstacles = obstacles