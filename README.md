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
1. Here you should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. The method used was: 
```python
with open('colliders.csv', 'r') as file:
    header = file.readline()
    lat, lon = header.split(",")
    lat0 = float(lat.strip().split(' ')[1])
    lon0 = float(lon.strip().split(' ')[1])
print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
#Output:
#North offset = -316, east offset = -445

```

2. Determine your local position relative to global home 
```python
    #set home position to (lon0, lat0, 0)
		global_home = self.set_home_position(lon0, lat0, 0)

		#retrieve current global position
		global_position = self.global_position

		#convert to current local position using global_to_local()
		local_position_ned = global_to_local(global_position, global_home)
````
3. In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position.

```python
grid_start = (local_position_ned[0] - north_offset, local_position_ned[1] - east_offset)
```

4. Set goal position as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)

```python
grid_goal_geodetic = (lat, lon, 0)
grid_goal_ned = global_to_local(grid_goal_geodetic, global_home)
```
I found more interesting setting a new random point.

```python
north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))


east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

grid_goal = (random.randrange(-north_min, north_max), random.randrange(-east_min, east_max))
````
5. Modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. In your writeup, explain the code you used to accomplish this step.









