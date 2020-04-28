# 3DMotionPlanning :bee:

Path planning algorithm for a quadcopter in a 3D environment.
![Image of drone flying](https://github.com/lorenzoajt/3DMotionPlanning/blob/master/misc/flying.png)

## Starter code explanation :shipit:
The first task in this project is to explain what's different about motion_planning.py from the backyard_flyer_solution.py script, and how the functions provided in planning_utils.py work.

1. The first thing to notice is that there is an extra state called PLANNING. Planning state begins after the drone is armed as stated in state_callback() function.
2. Theres no calculate box function in motion Planning
3. It has a send waypoints function to send them to the simulator

**plan_path() function**

1. Sets a fixed target altitude and safety distance

2. Load the obstacles. 

3. create_grid() function: Returns a 2d grid representation of the obstacles given the drone altitude and safety distance. It also returns the minimum north  and east coordinates.
```python
print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

#Output:
#North offset = -316, east offset = -445

```

4. Set a grid start and a grid goal. 
````python
grid_start = (-north_offset, -east_offset)
grid_goal = (-north_offset + 10, -east_offset + 10)
print('Local Start and Goal: ', grid_start, grid_goal)

#Output
#Local Start and Goal:  (316, 445) (326, 455)
````

6. a_star() function: find path from start to goal
````python
path, _ = a_star(grid, heuristic, grid_start, grid_goal)
print(path)

#Output
#Found a path.
[(316, 445), (316, 446), (317, 446), (317, 447), (318, 447), (318, 448), (319, 448), (319, 449), (320, 449), (320, 450), (321, 450), (321, 451), (322, 451), (322, 452), (323, 452), (323, 453), (324, 453), (324, 454), (325, 454), (325, 455), (326, 455)]
````


7. Set the Waypoints: It has to be with respect to the origin, that's why we add the offsets so we can command the drone with these coordinates.
```python
waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
print(waypoints)

#Output
#[[0, 0, 5, 0], [0, 1, 5, 0], [1, 1, 5, 0], [1, 2, 5, 0], [2, 2, 5, 0], [2, 3, 5, 0], [3, 3, 5, 0], [3, 4, 5, 0], [4, 4, 5, 0], [4, 5, 5, 0], [5, 5, 5, 0], [5, 6, 5, 0], [6, 6, 5, 0], [6, 7, 5, 0], [7, 7, 5, 0], [7, 8, 5, 0], [8, 8, 5, 0], [8, 9, 5, 0], [9, 9, 5, 0], [9, 10, 5, 0], [10, 10, 5, 0]]
````

![Image of first path](https://github.com/lorenzoajt/3DMotionPlanning/blob/master/misc/first.png)

## Implementing the Path Planning Algorithm
1. Here you should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. The method used was: 
```python
with open('colliders.csv', 'r') as file:
    header = file.readline()
    lat, lon = header.split(",")
    lat0 = float(lat.strip().split(' ')[1])
    lon0 = float(lon.strip().split(' ')[1])
self.set_home_position(lon0, lat0, 0)
home_pos = (lon0, lat0, 0)

```

2. Determine your local position relative to global home 
```python
geo_curr_coordinates = self.global_position
# TODO: convert to current local position using global_to_local()
local_position_ned = global_to_local(geo_curr_coordinates, home_pos)

````
3. In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position.

```python
start = (local_position_ned[0]-north_offset, local_position_ned[1]-east_offset)
grid_start = (int(start[0]), int(start[1]))
```

4. Set goal position as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)

:rotating_light: **Here you can set geo_grid_goal to any latitude/longitude coordinate** :rotating_light:	

```python
geo_grid_goal = (-122.397545, 37.793745,  0.0)
goal_ned = global_to_local(geo_grid_goal, home_pos)
goal = (goal_ned[0]-north_offset, goal_ned[1]-east_offset)
grid_goal = (int(goal[0]), int(goal[1]))
```

5. Modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. In your writeup, explain the code you used to accomplish this step. The changes were in the Action class and valid_actions() function. 

Basically I just added DIAG_NW, DIAG_NE, DIAG_SW and DIAG_SE including their deltas and the cost which was for all square root of 2.
```python
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    DIAG_NW = (-1, -1, np.sqrt(2))
    DIAG_NE = (-1, 1, np.sqrt(2))
    DIAG_SW = (1, -1, np.sqrt(2))
    DIAG_SE = (1, 1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])
	
````
Then I added them to a list of valid actions and removed them if one of them collided with an object or got out of the map. 

```python
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    if x - 1 < 0 or y - 1 < 0 or grid[x-1, y-1]  == 1:
        valid_actions.remove(Action.DIAG_NW)
    if x -1 < 0 or y + 1 > m or grid[x-1, y+1] == 1:
        valid_actions.remove(Action.DIAG_NE)
    if x + 1 > n or y - 1 < 0 or grid[x+1, y-1]  == 1:
        valid_actions.remove(Action.DIAG_SW)
    if x + 1 > n or y +1 > m or grid[x+1, y+1] == 1:
        valid_actions.remove(Action.DIAG_SE)

    return valid_actions
```
**Path 1**
![Path 1 ](https://github.com/lorenzoajt/3DMotionPlanning/blob/master/misc/path1.png)
**Path 2**
![Path 2 ](https://github.com/lorenzoajt/3DMotionPlanning/blob/master/misc/path2.png)

6. Cull waypoints from the path you determine using search.

The method used was collinearity check.
First we need to calculate the determinant of 3 points and check if the result is 0 for collinearity.
```python
def collinearity_check(p1, p2, p3): 
    collinear = False
    # TODO: Calculate the determinant of the matrix using integer arithmetic 
    det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
    # TODO: Set collinear to True if the determinant is equal to zero
    
    if det == 0:
        collinear = True

    return collinear
```
Then all the points in the path are passed trough the prune_path() function which uses previous collinearity_check to eliminate all innecesary waypoints.

```python
def prune_path(path):
    pruned_path = [p for p in path]
    
    i = 0
    while i < len(pruned_path) - 2:
        p1 = pruned_path[i]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]
        
        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path
```

**Pruned path 1**
![Pruned path 1 ](https://github.com/lorenzoajt/3DMotionPlanning/blob/master/misc/pruned_path1.png)
**Pruned path 2**
![Path 2 ](https://github.com/lorenzoajt/3DMotionPlanning/blob/master/misc/pruned2.png)








