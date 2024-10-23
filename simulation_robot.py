import matplotlib.pyplot as plt
import numpy as np
import time
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Initialisation of robot parameters
robot_speed = 5  # m/s
robot_position = [0, 0, 0]  # Initial position of the robot (x, y, z)
robot_size = [2, 1, 1]  # Width, length and height of the robot (2m x 1m x 1m)

# Definition of sensor angles (8 sensors around the robot in a horizontal plane and 2 in the vertical)
sensor_angles_xy = np.linspace(0, 2*np.pi, 8, endpoint=False)
sensor_angles_z = [-np.pi/4, np.pi/4]  # Sensors tilted upwards and downwards

# Define obstacles in the scene (x, y, z) - you can modify this list
num_obstacles = 10
obstacles = [(np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(-1, 2)) for _ in range(num_obstacles)]

# Simulation of proximity sensor readings
def read_sensors():
    distances = []
    
    # Sensors in the horizontal plane (x-y)
    for angle in sensor_angles_xy:
        x_sensor = robot_position[0] + np.cos(angle) * 2
        y_sensor = robot_position[1] + np.sin(angle) * 2
        z_sensor = robot_position[2]
        distance = min([np.linalg.norm(np.array([obstacle[0], obstacle[1], obstacle[2]]) - np.array([x_sensor, y_sensor, z_sensor])) for obstacle in obstacles])
        distances.append(distance)
    
    # Sensors tilted upwards and downwards (x-y-z)
    for angle_z in sensor_angles_z:
        x_sensor = robot_position[0]
        y_sensor = robot_position[1]
        z_sensor = robot_position[2] + np.sin(angle_z) * 2
        distance = min([np.linalg.norm(np.array([obstacle[0], obstacle[1], obstacle[2]]) - np.array([x_sensor, y_sensor, z_sensor])) for obstacle in obstacles])
        distances.append(distance)
    
    return distances

# 3D robot movement logic with gradual speed reduction
def move_robot():
    global robot_position
    sensors = read_sensors()
    print("Proximity sensors: ", sensors)
    
    front_distance = min(sensors[0:4])  # Closest obstacle in front
    z_distance = min(sensors[8:10])  # Closest obstacle above/below

    # Reduce speed based on proximity
    if front_distance < 3:
        print("Reducing speed based on proximity to front obstacle.")
        robot_position[0] += robot_speed * (front_distance / 3) * 0.1  # Scales speed based on distance
    elif z_distance < 1:
        print("Reducing speed based on proximity to obstacle above or below.")
        robot_position[2] += robot_speed * (z_distance / 1) * 0.1  # Scales speed based on distance
    else:
        print("No obstacles detected, the robot is moving at full speed.")
        robot_position[0] += robot_speed * 0.1  # Moves at full speed

# 3D visual simulation with matplotlib
def visual_simulation():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Set axis limits to zoom in
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(-2, 5)
    
    # Set the viewing angle
    ax.view_init(elev=20, azim=60)

    # Draw the robot (parallelepiped) in green with black edges and no transparency
    robot_points = np.array([[robot_position[0] - robot_size[0]/2, robot_position[1] - robot_size[1]/2, robot_position[2] - robot_size[2]/2],
                             [robot_position[0] + robot_size[0]/2, robot_position[1] - robot_size[1]/2, robot_position[2] - robot_size[2]/2],
                             [robot_position[0] + robot_size[0]/2, robot_position[1] + robot_size[1]/2, robot_position[2] - robot_size[2]/2],
                             [robot_position[0] - robot_size[0]/2, robot_position[1] + robot_size[1]/2, robot_position[2] - robot_size[2]/2],
                             [robot_position[0] - robot_size[0]/2, robot_position[1] - robot_size[1]/2, robot_position[2] + robot_size[2]/2],
                             [robot_position[0] + robot_size[0]/2, robot_position[1] - robot_size[1]/2, robot_position[2] + robot_size[2]/2],
                             [robot_position[0] + robot_size[0]/2, robot_position[1] + robot_size[1]/2, robot_position[2] + robot_size[2]/2],
                             [robot_position[0] - robot_size[0]/2, robot_position[1] + robot_size[1]/2, robot_position[2] + robot_size[2]/2]])
    
    verts = [[robot_points[j] for j in [0, 1, 2, 3]],
             [robot_points[j] for j in [4, 5, 6, 7]], 
             [robot_points[j] for j in [0, 1, 5, 4]], 
             [robot_points[j] for j in [2, 3, 7, 6]], 
             [robot_points[j] for j in [1, 2, 6, 5]], 
             [robot_points[j] for j in [4, 7, 3, 0]]]
    
    ax.add_collection3d(Poly3DCollection(verts, facecolors='green', linewidths=1, edgecolors='black', alpha=1.0))  # Robot color changed to green

    # Draw the sensors (x-y and z planes) in yellow
    for angle in sensor_angles_xy:
        x_sensor = robot_position[0] + np.cos(angle) * 2
        y_sensor = robot_position[1] + np.sin(angle) * 2
        z_sensor = robot_position[2]
        ax.plot([robot_position[0], x_sensor], [robot_position[1], y_sensor], [robot_position[2], z_sensor], color='yellow')  # Changed color to yellow

    for angle_z in sensor_angles_z:
        x_sensor = robot_position[0]
        y_sensor = robot_position[1]
        z_sensor = robot_position[2] + np.sin(angle_z) * 2
        ax.plot([robot_position[0], x_sensor], [robot_position[1], y_sensor], [robot_position[2], z_sensor], color='purple')  # Changed color to purple

    # Draw the obstacles (blue spheres)
    for obstacle in obstacles:
        ax.scatter(obstacle[0], obstacle[1], obstacle[2], color='blue', s=300)  # Changed obstacle color to blue and size to 300
    
    plt.pause(0.1)
    plt.draw()

# Main simulation loop
def simulation_loop():
    for _ in range(20):  # Simulate 20 time steps
        move_robot()
        plt.clf()  # Clear the previous figure
        visual_simulation()  # Draw the new scene
        time.sleep(0.5)  # Delay for visual simulation

# Start the simulation
simulation_loop()
