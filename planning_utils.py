from enum import Enum
from queue import PriorityQueue
import numpy as np


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


def a_star(grid, h, start, goal):

    n, m = grid.shape[0] - 1, grid.shape[1] - 1

    # open set
    queue = PriorityQueue()
    queue.put((0, start))

    # backtracking cache
    came_from = {start: None}

    # closed set
    visited = {start}

    # g-score
    cost = {start: 0}

    # 8 directions to move
    dx = [0, 0, 1, 1, 1, -1, -1, -1]
    dy = [1, -1, 0, 1, -1, 0, 1, -1]
    
    while not queue.empty():
        _, current_node = queue.get()
            
        # backtracking
        if current_node == goal:        
            print('Found a path.')
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = came_from[current_node]
            return path[::-1], cost[goal]

        current_cost = cost[current_node]
        visited.add(current_node)

        # exploring
        for direction in range(8):
            x, y = current_node[0] + dx[direction], current_node[1] + dy[direction]
            next_node = (x,y)

            # ignore if node in closed set, or out of bounds, or not safe
            if next_node in visited or x < 0 or y < 0 or x >= n or y >= m or grid[x, y] == 1:
                continue

            # cost if following the new path
            tentative_cost = current_cost + euclidean(current_node, next_node)

            # update if visiting a new node, or find a shorter path to an old node
            if next_node not in cost or cost[next_node] > tentative_cost:
                cost[next_node] = tentative_cost
                came_from[next_node] = current_node
                queue.put((tentative_cost + h(next_node, goal), next_node))
             
    print('**********************')
    print('Failed to find a path!')
    print('**********************') 



def euclidean(position, goal_position):
    return sum(pow(a-b,2) for a,b in zip(position, goal_position))

