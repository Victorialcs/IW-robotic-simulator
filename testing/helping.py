# import pybullet as p
# import pybullet_data
# import time

# # Connect to PyBullet
# p.connect(p.GUI)
# p.resetSimulation()
# p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Default PyBullet assets path

# # Add the path to your BB8 URDF
# p.setAdditionalSearchPath("path_to_your_urdf_directory")  # Update this path

# # Set gravity
# p.setGravity(0, 0, -9.8)
# p.setRealTimeSimulation(0)

# # Load plane
# p.loadURDF("plane.urdf")

# # Load the BB8 URDF
# bb8 = p.loadURDF("bb8.urdf", [0, 0, 0.5])


# # Focus camera on BB8
# focus_position, _ = p.getBasePositionAndOrientation(bb8)
# p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=focus_position)

# # Keyboard control variables
# forward_velocity = 5
# turn_velocity = 1

# debug_text_id = None

# # Main loop
# while True:
#     keys = p.getKeyboardEvents()
    
#     # Movement logic
#     linear_velocity = [0, 0, 0]
#     angular_velocity = [0, 0, 0]
    
#     if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
#         linear_velocity[1] = forward_velocity  # Forward
#     if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
#         linear_velocity[1] = -forward_velocity  # Backward
#     if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
#         angular_velocity[2] = turn_velocity  # Turn left
#     if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
#         angular_velocity[2] = -turn_velocity  # Turn right
    
#     # Apply velocity to the BB8 body
#     p.resetBaseVelocity(bb8, linearVelocity=linear_velocity, angularVelocity=angular_velocity)
    
#     # Focus camera on the robot
#     focus_position, _ = p.getBasePositionAndOrientation(bb8)
#     p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=focus_position)
    
#     pos, _ = p.getBasePositionAndOrientation(bb8)
#     debug_text_id = p.addUserDebugText(f"BB8 Position: {pos}", [-1, 0, 2], textColorRGB=[1, 0, 0], textSize=1.5)

#     # Step simulation
#     p.stepSimulation()
#     time.sleep(0.01)

import numpy as np
from collections import deque


# Map dimensions
width, height = 10, 10

# Obstacles represented as a set of (x, y) coordinates
obstacles = {(0,9), (1,9), (0,8), (1,8), (1,3), (1,4), (1,5), (1,6), 
             (2,3), (2,4), (2,5), (2,6), (3,3), (3,4), (3,5), (3,6),
             (2,1), (3,1), (4,1), (5,1), (6,1), (6,2), (6,3), (7,3),
             (4,9), (5,9), (6,9), (7,9), (4,8), (5,8), (6,8), (7,8),
             (5,7), (6,7), (7,7), (5,6), (6,6), (5,5), (6,5), (9,5),
             (9,6), (9,7)}


# Start and goal coordinates
start = (0, 0)
goal = (9, 9)

# DFS function to search for a path
def bfs(start, goal):
    queue = deque([start])  # Queue to keep track of nodes to explore
    visited = {start: None}  # Dictionary to track visited nodes and their parents
    
    while queue:
        current = queue.popleft()

        if current == goal:
            # Goal reached; reconstruct the path
            path = []
            while current is not None:
                path.append(current)
                current = visited[current]
            path.reverse()  # Reverse the path to get it from start to goal
            return path

        # Define valid neighbors (up, down, left, right)
        neighbors = [(current[0] + dx, current[1] + dy) for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]]

        for neighbor in neighbors:
            # Check if the neighbor is within the grid bounds and not an obstacle
            if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height and neighbor not in visited and neighbor not in obstacles:
                queue.append(neighbor)  # Add neighbor to the queue
                visited[neighbor] = current  # Track the parent of this neighbor

    return None  # Return None if no path is found

# Run BFS to find the path
bfspath = bfs(start, goal)
obstacles.add((1,2))
bfspath = bfs(start, goal)
if bfspath:
    print("Path found:", bfspath)
else:
    print("No path found.")

def dfs(current, goal, visited, path):
    if current == goal:
        return True

    visited.add(current)
    
    # Define valid neighbors (up, down, left, right)
    neighbors = [(current[0] + dx, current[1] + dy) for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]]
    
    for neighbor in neighbors:
        # Check if the neighbor is within the grid bounds and not an obstacle
        if 0 <= neighbor[0] < width and 0 <= neighbor[1] < height and neighbor not in visited and neighbor not in obstacles:
            path.append(neighbor)  # Add the neighbor to the path
            if dfs(neighbor, goal, visited, path):  # Recursive DFS call
                return True
            path.pop()  # Backtrack if this path doesn't lead to the goal
    
    return False

def find_path_dfs(start, goal):
    visited = set()
    path = [start]
    
    if dfs(start, goal, visited, path):
        return path
    else:
        return None

# Run DFS to find the path
dfspath = find_path_dfs(start, goal)

if dfspath:
    print("Path found")
    print (dfspath)
else:
    print("No path found.")