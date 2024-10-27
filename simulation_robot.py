import heapq
import numpy as np
import matplotlib.pyplot as plt

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

# Visualization function
def visualize_grid(grid, path, explored, start, goal):
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # Draw grid with obstacles, start, goal, path, and explored nodes
    for x in range(grid_size):
        for y in range(grid_size):
            if grid[x, y] == 1:
                ax.add_patch(plt.Rectangle((y, grid_size - x - 1), 1, 1, color='black'))  # Obstacles
            elif (x, y) == start:
                ax.add_patch(plt.Rectangle((y, grid_size - x - 1), 1, 1, color='green'))  # Start point
            elif (x, y) == goal:
                ax.add_patch(plt.Rectangle((y, grid_size - x - 1), 1, 1, color='red'))    # Goal point
            elif (x, y) in explored and (x, y) not in path:
                ax.add_patch(plt.Rectangle((y, grid_size - x - 1), 1, 1, color='lightblue'))  # Explored nodes

    # Draw path
    if path:
        for (x, y) in path:
            if (x, y) != start and (x, y) != goal:
                ax.add_patch(plt.Rectangle((y, grid_size - x - 1), 1, 1, color='yellow'))  # Path cells
    
    # Grid and axis settings
    ax.set_xticks(np.arange(0, grid_size, 1))
    ax.set_yticks(np.arange(0, grid_size, 1))
    ax.grid(color='gray')
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    plt.gca().invert_yaxis()
    plt.show()

# Visualize the grid with path, explored nodes, and obstacles
visualize_grid(grid, path, explored, start, goal)
