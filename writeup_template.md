## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 2. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
##### 2.1 `planning_utils.py`

This script helps define several helper functions (creating grid, A\* search for shortest path) that will be used in `motion_planning.py`, specially the function `MotionPlanning.plan_path`

Function | Functionality
-------- | ---------------
`create_grid`  | creating a grid that represent the 2D map of the environment, with each cell being 0 if it is safe to move to the cell or 1 if there is an obstacle in or next to the cell 
`valid_action` | returning the set of valid actions to take in each of the cell in the grid, i.e. the set of moves that move the drone into a safe cell rather than an unsafe cell 
`a_star` | using A\* search to find the shortest safe path from a starting point to a goal 
`heuristic` | return the heuristic function that is used in A\* search to estimate the distance towards the goal (here we use Euclidean distance) 

##### 2.2 `motion_planning.py` 

This script defines all the necessary callbacks and transitions for piloting the drones through the waypoints.

Functions | Functionality
--------- | -------------
`MotionPlanning.__init__` | Initializing the waypoints, the drone status and registering the callbacks 
`MotionPlanning.local_position_callback` | Defining the callback when receiving a new position message:  either transitioning to waypoints if just finishing taking off, or continuing the waypoints if the drone is following the waypoints and not yet reaching the final waypoint, or transitioning to landing otherwise 
`MotionPlanning.velocity_callback` | Defining the callback when receiving a new velocity message: if the drone is landing and about to hit the ground, transitioning to disarming 
`MotionPlanning.state_callback` | Defining the callback when receiving a new state message: transitioning to the appropriate states 
`MotionPlanning.arming_transition` | Defining the transition to arming state: start the drone and let the program take control of the pilot 
`MotionPlanning.takeoff_transition` | Defining the transition to takeoff state: set the target altitude to the altitude of the first point on the waypoint 
`MotionPlanning.waypoint_transition` | Defining the transition to and during waypoint state: set the target to the next waypoint 
`MotionPlanning.landing_transition` | Defining the transition to landing state 
`MotionPlanning.disarming_transition` | Defining the transition to disarming state 
`MotionPlanning.manual_transition` | Defining the transition to manual state 
`MotionPlanning.send_waypoints` | helper function used by `plan_path` to send the waypoints to the simulator for visualization of the path 
`MotionPlanning.plan_path` | preparing the waypoints: 1. read the data from `collider.csv`, then set up the grid, then use A\* search to find the best path 
 |  

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

Open and read the first line of the file `colliders.csv`. Split the line at the comma to separate the latitude and the longitude fields. Use `set_home_position` to set the global home position with the given coordinates.

```python
        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv', 'r') as inputfile:
            fields = inputfile.read().strip().split(",")
            lat0 = float(fields[0].split()[1])
            lon0 = float(fields[1].split()[1])

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position
Obtain the current global position from `_longitude` and `_latitude` attributes of the drone, and pass them through the function `global_to_local` to get the current local position.

```python
        # TODO: retrieve current global position
        global_position = (self._longitude, self._latitude, self._altitude)
 
        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(global_position, self.global_home)
```

#### 3. Set grid start position from local position

Convert the local position to the grid start position by offsetting the local position with the grid offsets and round them down to the nearest integers.

```python
        # TODO: convert start position to current position rather than map center
        grid_start = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))
```

#### 4. Set grid goal position from geodetic coords
First convert the global goal geodetic coordinates to the local goal position using the `global_to_local` function, and then offsetting the resulted local goal position with the grid offsets to obtain the grid goal position.

```python
        # TODO: adapt to set goal as latitude / longitude position and convert
        global_goal = (-122.39400598, 37.79696888, 0)
        local_goal = global_to_local(global_goal, self.global_home)
        grid_goal = (int(local_goal[0]-north_offset), int(local_goal[1] - east_offset))
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
I use 16 moves on the grids (8 king's moves and 8 knight moves) to approximate the possible state transitions of the drone.

```python
    # 16 directions to move (king moves + knight moves)
    dx = [0, 0, 1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 2, 2, -2, -2]
    dy = [1, -1, 0, 1, -1, 0, 1, -1, 2, -2, 2, -2, 1, -1, 1, -1]
```

I cleaned up the A\* search code a bit and use the Euclidean distance as both the true cost and the heuristic cost of the path

```python
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

    # 16 directions to move (king moves + knight moves)
    dx = [0, 0, 1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 2, 2, -2, -2]
    dy = [1, -1, 0, 1, -1, 0, 1, -1, 2, -2, 2, -2, 1, -1, 1, -1]
    
    while not queue.empty():
        _, current_node = queue.get()
            
        # backtracking if goal is reached
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
        for _dx, _dy in zip(dx, dy):
            x, y = next_node = current_node[0] + _dx, current_node[1] + _dy

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
```

#### 6. Cull waypoints 

I use the Bresenham to identify redundant waypoints. In each step, I try to connect the current point to the further possible way point without going through any unsafe cell on the grid and eliminate all the way points in between.

```python
        # TODO: prune path to minimize number of waypoints
        current_node = path[0]
        prune_path = [current_node]

        for i in range(1, len(path)-1):
            if any(x<0 or x>=n or y<0 or y>=m or grid[x,y]==1 for x,y in bresenham(*current_node, *path[i+1])):
                current_node = path[i]
                prune_path.append(current_node)

        prune_path.append(path[-1])
```



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


