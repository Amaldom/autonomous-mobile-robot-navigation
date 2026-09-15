A* Path Planning

Overview

This folder contains the Python implementation of A* path planning developed as part of the Autonomous Mobile Robot Navigation & Path Planning project.

The planner operates on an occupancy grid derived from a map and generates a collision-free path between a defined start position and goal position. The resulting path was evaluated in a custom Webots 3D simulation environment.

The implementation focuses on practical mobile-robot path planning by combining:

* Occupancy-grid mapping
* Obstacle inflation
* A* search
* Euclidean heuristic
* 8-directional movement
* Diagonal collision checking
* Wall-proximity cost
* Grid ↔ world coordinate conversion
* Waypoint reduction
* Path visualisation

⸻

Planning Pipeline

Map Image
    ↓
Grayscale Conversion
    ↓
Binary Map
    ↓
Resize to Occupancy Grid
    ↓
Obstacle Inflation
    ↓
Start / Goal Conversion
    ↓
A* Path Planning
    ↓
Wall-Proximity Cost
    ↓
Path Reconstruction
    ↓
Waypoint Reduction
    ↓
Webots Evaluation

⸻

Occupancy Grid

The input map is converted into a binary occupancy grid where:

* 1 represents free space
* 0 represents obstacles

The map is represented using a 120 × 60 grid.

The implementation also inflates obstacles before planning. This provides additional clearance around obstacles and helps reduce the risk of generating paths too close to walls.

⸻

A* Algorithm

The planner uses the standard A* search strategy based on:

[
f(n) = g(n) + h(n)
]

where:

* g(n) is the accumulated cost from the start node
* h(n) is the heuristic estimate to the goal
* f(n) is the total estimated cost

A Euclidean distance heuristic is used:

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

The planner considers 8 possible movement directions, allowing horizontal, vertical and diagonal movement.

Diagonal movement is additionally checked to prevent the robot from passing diagonally through obstacle corners.

⸻

Wall-Proximity Penalty

In addition to the normal movement cost and heuristic, the implementation applies a wall-proximity penalty.

The planner evaluates nearby cells and increases the cost of nodes located close to obstacles. This encourages the generated path to maintain greater clearance from walls where possible.

Movement Cost
      +
Heuristic Cost
      +
Wall-Proximity Penalty
      ↓
   A* Priority

This makes the planning strategy more suitable for a physical mobile robot than simply selecting the shortest geometric route.

⸻

Coordinate Conversion

The implementation provides conversion functions between Webots world coordinates and occupancy-grid coordinates.

World → Grid

world_to_grid(x, y)

Grid → World

grid_to_world(row, col)

The map dimensions used by the planner are:

* Width: 4.71 m
* Height: 9.56 m

This allows planned grid paths to be related to positions within the Webots environment.

⸻

Obstacle Inflation

Before running A*, obstacles are expanded using morphological dilation.

inflate_obstacles(grid, inflation_radius=1)

Obstacle inflation provides an additional safety margin around occupied cells and helps prevent the planner from generating paths that pass too close to obstacles.

⸻

Path Reconstruction and Waypoint Reduction

After A* reaches the goal, the path is reconstructed by following the stored parent nodes back from the goal to the start.

The implementation then reduces the number of path points using:

reduce_by_step(path, step=2)

This produces a smaller set of waypoints that can be more practical for robot navigation and visualisation.

⸻

Visualisation

The implementation includes several visualisation functions for analysing the planner:

* A* path over the occupancy grid
* Path-only visualisation
* Wall-penalty heatmap
* Occupancy-grid visualisation

These visualisations were used to inspect the generated path and the effect of the wall-proximity penalty.

⸻

Webots Evaluation

The A* planner was evaluated using a custom Webots 3D simulation environment.

The planned route was generated from the mapped environment and used to demonstrate navigation from the selected start position to the goal while avoiding obstacles.

The demonstrated simulation produced a path that reached the goal without collision.

A* path planning was evaluated as a separate Webots-based contribution to the overall project.

⸻

File

File	Description
MRcode.py	Python implementation of occupancy-grid processing, obstacle inflation, A* search, wall-proximity cost and path visualisation

⸻

Technologies

* Python
* NumPy
* OpenCV
* SciPy
* Matplotlib
* Webots
* A* Path Planning
* Occupancy Grid Mapping

⸻

Key Learning Outcomes

Through this component, I developed practical experience with:

* Implementing A* from scratch in Python
* Representing environments using occupancy grids
* Converting between world and grid coordinates
* Applying obstacle inflation for safer planning
* Designing additional path-cost functions
* Handling diagonal movement and collision constraints
* Visualising and analysing generated paths
* Evaluating a path-planning algorithm in Webots

⸻

Limitations & Future Improvements

Potential improvements include:

* Dynamic obstacle handling
* Real-time replanning
* More advanced path smoothing
* Improved trajectory generation from A* waypoints
* Automatic map acquisition from the simulated robot
* Integration with a complete autonomous navigation stack
* More systematic comparison with alternative planners such as Dijkstra or RRT

⸻

Project Context

This A* implementation forms one component of the broader:

Autonomous Mobile Robot Navigation & Path Planning

project, which also includes Pioneer 3-DX navigation, odometry-based waypoint navigation, colour-based target tracking, sonar obstacle avoidance, Mapper3 mapping and Webots simulation.

See the main repository README for the complete project overview.
