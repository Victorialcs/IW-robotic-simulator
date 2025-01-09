import pybullet as p
import time
import pybullet_data
import numpy as np
from collections import deque


# Connect to the PyBullet GUI
p.connect(p.GUI, options="--width=1920 --height=1080")


# Set up the simulation
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Path to PyBullet's data
p.setGravity(0, 0, -9.8)

camera_distance = 7     # Distance from the scene
camera_yaw = 0          # Yaw angle to look parallel along the grid (X-axis)
camera_pitch = -70      # Pitch angle to tilt downwards slightly
camera_target_position = [0, -2, 0]  # The point the camera looks at (center of the grid)
p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)

# Load a plane to have a ground
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

# Load the simple robot from the URDF file at an elevated position
dfsrobot = p.loadURDF("simple_robot.urdf", [-4.5, -4.5, 0.3])
bfsrobot = p.loadURDF("bfsRobot.urdf", [100, 100, 0]) # load this robot off screen to avoid collision
thirdRobot = p.loadURDF("thirdRobot.urdf", [-100, 100, 0])
fourthRobot = p.loadURDF("fourthRobot.urdf", [-100, -100, 0])

obheight = 0.5
ob1 = p.loadURDF("obstacle1.urdf", [-4, 4, obheight/2+0.05])
ob2 = p.loadURDF("obstacle2.urdf", [-2.5, 0, obheight/2+0.05])
ob3 = p.loadURDF("obstacle3.urdf", [-1, -3.5, obheight/2+0.05])
ob4 = p.loadURDF("obstacle4.urdf", [0.5, 4, obheight/2+0.05])
ob5 = p.loadURDF("obstacle5.urdf", [4.5, 1.5, obheight/2+0.05])




# Set the initial robot position
initial_position = [-4.5, -4.5, 0.3]  # Initial position of the robot


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

# DFS FUNCTION TO FIND PATH
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
    path_3d = [(x, y, 0.3) for (x, y) in dfspath]
else:
    print("No path found.")

dfs_target_positions = path_3d - np.array([4.5, 4.5, 0])
third_target_positions = path_3d - np.array([4.5, 4.5, 0])

# BFS FUNCTION TO FIND PATH
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

if bfspath:
    print("Path found.")
    path_3d = [(x, y, 0.3) for (x, y) in bfspath]
else:
    print("No path found.")

# Convert path coordinates to PyBullet's coordinate system
bfs_target_positions = np.array(path_3d) - np.array([4.5, 4.5, 0])
fourth_target_positions = np.array(path_3d) - np.array([4.5, 4.5, 0])

# MOVING THRU TARGET POSITION LIST
movement_steps = 5  # Number of steps to complete the shift for each target
current_target_index = [0, 0, 0, 0]  # Start with the first target

# Function to calculate movement increment
def calculate_movement_increment(initial_position, target_position, steps):
    return [(target_position[i] - initial_position[i]) / steps for i in range(3)]

# Calculate movement increment for the first target
robot_positions = [initial_position.copy(), initial_position.copy(), initial_position.copy(), initial_position.copy()]
movement_increment = [calculate_movement_increment(robot_positions[0], dfs_target_positions[0], movement_steps), None, None, None]
robot_moving = [True, False, False, False]

# Initialize step counter
step_counter = [0, 0, 0, 0]
robotnum = 0
pop = False #obstaclepop

text_id = p.addUserDebugText(
                text="DFS Path",      # Text to display
                textPosition=[0, 0, 4],             # Position of the text in the world (x, y, z)
                textColorRGB=[1.0, 0.5, 0.0],         # RGB color of the text (red in this case)
                textSize=3,                         # Size of the text
                lifeTime=0                          # Duration (0 for persistent)
                )

text_start = p.addUserDebugText(
                text="Start",      # Text to display
                textPosition=[-2.5, -4.5, 3.1],             # Position of the text in the world (x, y, z)
                textColorRGB=[0, 0, 0],         # RGB color of the text (red in this case)
                textSize=1.3,                         # Size of the text
                lifeTime=0                          # Duration (0 for persistent)
                )

text_goal = p.addUserDebugText(
                text="Goal",      # Text to display
                textPosition=[2.1, 0, 3.2],             # Position of the text in the world (x, y, z)
                textColorRGB=[0, 0, 0],         # RGB color of the text (red in this case)
                textSize=1.3,                         # Size of the text
                lifeTime=0                          # Duration (0 for persistent)
                )


# def detect_new_obstacle(robot_pos, direction):
#     # Define a small offset for obstacle detection
#     ray_end = [robot_pos[0] + direction[0], robot_pos[1] + direction[1], robot_pos[2]]
#     results = p.rayTest(robot_pos, ray_end)
#     for result in results:
#         hit_object_id, hit_position, _ = result[0], result[3], result[2]
#         if hit_object_id >= 0:  # If an obstacle is detected
#             obstacle_coords = (round(hit_position[0] + 4.5), round(hit_position[1] + 4.5))
#             obstacles.add(obstacle_coords)
#             print(f"Detected new obstacle at {obstacle_coords}")
#             return True
#     return False

# if detect_new_obstacle(robot_positions[i], movement_increment[i]):
#     print("Obstacle detected, recalculating path.")
#     current_start = (round(robot_positions[i][0] + 4.5), round(robot_positions[i][1] + 4.5))
#     if i == 0:  # Recalculate path for DFS
#         dfspath = find_path_dfs(current_start, goal)
#         dfs_target_positions = np.array([(x - 4.5, y - 4.5, 0.3) for (x, y) in dfspath])
#     elif i == 1:  # Recalculate path for BFS
#         bfspath = bfs(current_start, goal)
#         bfs_target_positions = np.array([(x - 4.5, y - 4.5, 0.3) for (x, y) in bfspath])
#     current_target_index[i] = 0
#     step_counter[i] = 0
#     continue

def collide(rob):
    position, _ = p.getBasePositionAndOrientation(rob)
    obs = [1.5, -0.5, 0.3]
    dist = np.linalg.norm(np.array(position) - obs)
    if dist < 1 :
        print ("current position:", position)
        return True
    else:
        return False 

    
# Main simulation loop
while True:
    p.stepSimulation()
    time.sleep(1./240.)
    if pop and (time.time() - start_time) >= 5:
        obpop = p.loadURDF("obstaclepop.urdf", [1.5, -0.5, obheight/2+0.05])
        pop = False


    for i in range(4):  # Iterate over all four robots
        
        if robot_moving[i]:
            step_counter[i] += 1

            # Obstacle detection and pathfinding logic
            if i == 2 and collide(thirdRobot):
                obstacles.add((6,4))
                dfspath = find_path_dfs((5,4), goal)
                if dfspath:
                    print("Path found")
                    path_3d = [(x, y, 0.3) for (x, y) in dfspath]
                else:
                    print("No path found.")
                third_target_positions = path_3d - np.array([4.5, 4.5, 0])
                current_target_index[i] = 0
            
            if i == 3 and collide(fourthRobot):
                bfspath = bfs((5,4), goal)
                if bfspath:
                    print("Path found")
                    path_3d = [(x, y, 0.3) for (x, y) in bfspath]
                else:
                    print("No path found.")
                fourth_target_positions = path_3d - np.array([4.5, 4.5, 0])
                current_target_index[i] = 0

            # Move the robot towards the current target position
            if step_counter[i] <= movement_steps:
                robot_positions[i] = [robot_positions[i][j] + movement_increment[i][j] for j in range(3)]
                robot = dfsrobot if i == 0 else bfsrobot if i == 1 else thirdRobot if i == 2 else fourthRobot
                p.resetBasePositionAndOrientation(robot, robot_positions[i], [0, 0, 0, 1])
            else:
                # Update the next target or stop the robot if it has reached the end
                current_target_index[i] += 1

                if i == 0 and current_target_index[i] < len(dfs_target_positions):
                    step_counter[i] = 0
                    movement_increment[i] = calculate_movement_increment(robot_positions[i], dfs_target_positions[current_target_index[i]], movement_steps)
                elif i == 1 and current_target_index[i] < len(bfs_target_positions):
                    if np.allclose(robot_positions[i], (4.5, 4.5, 0.3), atol=1e-2):
                        print("GOAL")
                        break
                    step_counter[i] = 0
                    movement_increment[i] = calculate_movement_increment(robot_positions[i], bfs_target_positions[current_target_index[i]], movement_steps)
                elif i == 2 and current_target_index[i] < len(third_target_positions):
                    step_counter[i] = 0
                    movement_increment[i] = calculate_movement_increment(robot_positions[i], third_target_positions[current_target_index[i]], movement_steps)
                elif i == 3 and current_target_index[i] < len(fourth_target_positions):
                    step_counter[i] = 0
                    movement_increment[i] = calculate_movement_increment(robot_positions[i], fourth_target_positions[current_target_index[i]], movement_steps)
                else:
                    robot_moving[i] = False
                    robotnum += 1
                    # Add debug messages for each robot
                    if i == 0:
                        p.removeBody(dfsrobot)
                        p.removeUserDebugItem(text_id)
                        text_id = p.addUserDebugText(
                            text="BFS Path",      # Text to display
                            textPosition=[0, 0, 4],             # Position of the text in the world (x, y, z)
                            textColorRGB=[0, 1, 0],             # RGB color of the text (red in this case)
                            textSize=3,                         # Size of the text
                            lifeTime=0                          # Duration (0 for persistent)
                            )
                    elif i == 1:
                        p.removeBody(bfsrobot)
                        p.removeUserDebugItem(text_id)
                        text_id = p.addUserDebugText(
                            text="DFS Path with Sensors",      # Text to display
                            textPosition=[-1, 0, 4],             # Position of the text in the world (x, y, z)
                            textColorRGB=[0, 0, 1],             # RGB color of the text (red in this case)
                            textSize=3,                         # Size of the text
                            lifeTime=0                          # Duration (0 for persistent)
                            )                    
                    elif i == 2:
                        p.removeBody(thirdRobot)
                        p.removeUserDebugItem(text_id)
                        p.removeBody(obpop)
                        text_id = p.addUserDebugText(
                            text="BFS Path with Sensors",      # Text to display
                            textPosition=[-1, 0, 4],             # Position of the text in the world (x, y, z)
                            textColorRGB=[1, 0, 0],             # RGB color of the text (red in this case)
                            textSize=3,                         # Size of the text
                            lifeTime=0                          # Duration (0 for persistent)
                            )                           
                    elif i == 3:
                        p.removeBody(fourthRobot)
                        p.removeUserDebugItem(text_id)


    # Robot sequencing logic
    if not any(robot_moving):
        if robotnum == 1:
            robot_moving[1] = True
            current_target_index[1] = 0
            movement_increment[1] = calculate_movement_increment(robot_positions[1], bfs_target_positions[0], movement_steps)
        elif robotnum == 2:
            robot_moving[2] = True
            current_target_index[2] = 0
            movement_increment[2] = calculate_movement_increment(robot_positions[2], third_target_positions[0], movement_steps)
            start_time = time.time()
            pop = True
        elif robotnum == 3:
            robot_moving[3] = True
            current_target_index[3] = 0
            movement_increment[3] = calculate_movement_increment(robot_positions[3], fourth_target_positions[0], movement_steps)
            start_time = time.time()-2
            pop = True
    