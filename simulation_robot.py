import heapq
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the grid dimensions
grid_size = 10

# Create a 10x10 grid with obstacles
grid = np.zeros((grid_size, grid_size), dtype=int)

# Define new start (S) and goal (G) positions
start = (8, 1)  # Near bottom-left corner
goal = (1, 8)   # Near top-right corner

# Place fixed obstacles (O) in the grid
obstacles = [
    (3, 1), (3, 2), (3, 3), (4, 5), (5, 5), (6, 5), 
    (6, 6), (7, 7), (2, 4), (2, 6), (4, 8), (5, 8)
]
for obs in obstacles:
    grid[obs] = 1  # Mark obstacles as 1 in the grid

# Heuristic function (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* algorithm
def astar(grid, start, goal):
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    explored = set()  # To keep track of explored nodes
    
    while open_list:
        # Get the node with the lowest f(n) value
        _, current = heapq.heappop(open_list)
        
        # If we reach the goal, reconstruct the path
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1], explored  # Return reversed path from start to goal, along with explored nodes
        
        explored.add(current)
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Possible movements: up, down, left, right
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Check if the neighbor is within grid bounds and not an obstacle
            if 0 <= neighbor[0] < grid_size and 0 <= neighbor[1] < grid_size and grid[neighbor] == 0:
                tentative_g_score = g_score[current] + 1  # Cost to reach neighbor
                
                # If the neighbor is not in g_score or we found a cheaper path
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
    
    return None, explored  # No path found

# Run A* and get the path and explored nodes
path, explored = astar(grid, start, goal)

# 3D Visualization function
def visualize_3d_grid(grid, path, explored, start, goal):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Set axis limits
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    ax.set_zlim(0, 3)  # 3 layers: 0 for grid, 1 for explored, 2 for path

    # Draw obstacles
    for x in range(grid_size):
        for y in range(grid_size):
            if grid[x, y] == 1:
                ax.bar3d(y, grid_size - x - 1, 0, 1, 1, 1, color='black', alpha=0.8)  # Obstacles

    # Draw explored nodes
    for (x, y) in explored:
        if (x, y) != start and (x, y) != goal:
            ax.bar3d(y, grid_size - x - 1, 1, 1, 1, 1, color='lightblue', alpha=0.5)  # Explored nodes

    # Draw the path
    if path:
        for (x, y) in path:
            if (x, y) != start and (x, y) != goal:
                ax.bar3d(y, grid_size - x - 1, 2, 1, 1, 1, color='yellow', alpha=0.7)  # Path cells

    # Draw start and goal
    ax.bar3d(start[1], grid_size - start[0] - 1, 2, 1, 1, 1, color='green', alpha=1.0)  # Start point
    ax.bar3d(goal[1], grid_size - goal[0] - 1, 2, 1, 1, 1, color='red', alpha=1.0)    # Goal point

    # Labels and title
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Layers")
    ax.set_title("3D Simulation of A* Pathfinding")

    plt.show()

# Visualize the 3D grid with path, explored nodes, and obstacles
visualize_3d_grid(grid, path, explored, start, goal)
