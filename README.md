# Multi Agent Path Finding
This project focuses on solving the path planning problem in multi-robot systems, specifically for warehouse automation. The goal is to ensure that multiple robots can navigate a warehouse environment efficiently without collisions, while avoiding static obstacles like shelves. The simulation mimics real-world conditions and is built using Python and Matplotlib for visualization.

## Features:
- **A* Algorithm**: Implemented for single-agent pathfinding.
- **Multi-Agent Pathfinding (MAPF)**:
  - **Priority-Based Search**: Fast, but not always collision-free.
  - **Conflict-Based Search (CBS)**: Complete and optimal, though computationally intensive.
- **Warehouse Simulation**: A 2D grid-based warehouse environment where robots navigate from start to goal positions while avoiding obstacles.


**File Execution**
1: Execute the Independent MAPF solver using the below command line:
```
	python main.py--grid grids/test_map_3.txt --algorithm Independent
```

2: Execute the Priority Based MAPF solver using the below command line:
```
	python main.py--grid grids/test_map_3.txt --algorithm Prioritized
```
3: Execute the Conflict Based MAPF solver using the below command line:
```
	python main.py--grid grids/test_map_3.txt --algorithm CBS
```
# Result
**Conflict Base Search Algorithm**:
![CBS animation](media\CBS_output.gif)

**Priority Base Search Algorithm:**
![priority](media\priority_output.gif)