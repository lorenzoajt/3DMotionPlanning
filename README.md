# 3DMotionPlanning
Path planning algorithm for a quadcopter in a 3D environment.

## Starter code explanation
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
## Implementing the Path Planning Algorithm








