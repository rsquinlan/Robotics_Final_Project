"""grocery controller."""

"""
IMPORTANT: This must be run on Webots version 2022b not 2023a.
Version 2023a breaks color blob detection. 
Need to pip install ikpy and add Range Finder to Tiago robot tree under 'Camera' before running.

ALSO VERY IMPORTANT: If robot is autonomous mode, it may get stuck on certain waypoints 
and start turning back and forth. If this happens, press N on your keyboard to skip 
to the next target. This will correct the robot's movement. Don't press for too long or the 
robot will try to go through a wall.

CONTROLS FOR ARM: 
Up Arrow: moves arm forward (away from ) 
'D': move arm down
'U': move arm up
'F': grab object

"""
###################
#
# SETUP
#
###################

#Import libraries
from controller import Robot
from controller import RangeFinder
import math
import random
import numpy as np
from ikpy.chain import Chain
from matplotlib import pyplot as plt

#Initialization
print("=== Initializing Grocery Shopper...")

#Consts
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12
LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 4 # Meters, was 5.5
LIDAR_ANGLE_RANGE = math.radians(240)

forwardback = 0
leftright = 0
updown = 0

old_a = -4000 # Initialization of old bearing value

# Create the Robot instance.
robot = Robot()

# Initalize shelf value
shelf = "shelf the object is on"

# Get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#Initialize value for currentTarget
currentTarget = 1

# The Tiago robot has multiple motors, each identified by their names below

part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
              "gripper_left_finger_joint","gripper_right_finger_joint")

# 

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0, .07, 0, 0, 0, 0, 0.0, 0, 'inf', 'inf',0.0,0.0)

# Initialize face directions
face_direction = ["north", "south", "south", "south", "south", "south", "north", "north", "north", "north", "north"]

# Initialize devices
robot_parts={}
for i, part_name in enumerate(part_names):
    robot_parts[part_name]=robot.getDevice(part_name)
    robot_parts[part_name].setPosition(float(target_pos[i]))
    robot_parts[part_name].setVelocity(robot_parts[part_name].getMaxVelocity() / 2.0)

# Enable gripper encoders (position sensors)
left_gripper_enc=robot.getDevice("gripper_left_finger_joint_sensor")
right_gripper_enc=robot.getDevice("gripper_right_finger_joint_sensor")
left_gripper_enc.enable(timestep)
right_gripper_enc.enable(timestep)

# Enable Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# Enable Range Finder
kinect = robot.getDevice("range-finder")
kinect.enable(timestep)
kinect.projection = "spherical"

# Enable GPS and compass localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Enable Keyboard
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# Enable LiDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Enable display
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS) # Initialize offsets
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

# Mapping Setup
map = None
map = np.empty((360,360))
newMap = np.empty((360,360))

# Mode Setting
#mode = 'manual' 
mode = 'planner'
#mode = "autonomous"
#mode = "autonomous mapping"
#mode = "initial grab"

driveMode = "driving"

###################
#
# URDF LINK SETUP
#
###################

# Import ikpy library 
from ikpy.link import OriginLink, URDFLink 

#with open("tiago_urdf.urdf", "w") as file:  
    #file.write(robot.getUrdf())

# Initialize gripper status
gripper_status = "closed"

# Initialize wheel velocities
robot_parts["wheel_left_joint"].setVelocity(0)
robot_parts["wheel_right_joint"].setVelocity(0)

# Initialize base elements list
base_elements=["base_link", "base_link_Torso_joint", "Torso", "torso_lift_joint", "torso_lift_link", "torso_lift_link_TIAGo front arm_joint", "TIAGo front arm"]

# Initialize chain from urdf file
my_chain = Chain.from_urdf_file("tiago_urdf.urdf", last_link_vector=[0.004, 0,-0.1741], base_elements=base_elements)

# Initialize parts 
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
            "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
            "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

for link_id in range(len(my_chain.links)):
    # This is the actual link object
    link = my_chain.links[link_id]
    # Disabled "torso_lift_joint" manually as it can cause the TIAGO to become unstable.
    if link.name not in part_names or  link.name =="torso_lift_joint":
        my_chain.active_links_mask[link_id] = False
  
i = 0      
motors = []
for link in my_chain.links:
    if link.name in part_names and link.name != "torso_lift_joint":
        motor = robot.getDevice(link.name)
        # Accounted for any motors that require a different maximum velocity
        if link.name == "torso_lift_joint":
            motor.setVelocity(0.07)
        else:
            motor.setVelocity(0.5)
        
        # Get position, enable, and append motors
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timestep)
        motors.append(motor)

"""
[a, b, c] = [ , , height] 
"""
target_above = [0.3, 0, 0.77]
target_or = [0, 0, 0]

mapping_array = [(-5, 0), (-5.39, -5.62), (12.7, -5.38), (13, -2.1), (-5.9, -2.1), (-5.67, 2.06), (12.6, 1.8), (12.1, 5.72), (-9.75, 5.66), (-8.71, -5.71), (-11.5, -6.1), (-11.4, 5.02)]

recognized_objects = camera.getRecognitionObjects()

robot_parts["gripper_left_finger_joint"].setPosition(0.045)
robot_parts["gripper_right_finger_joint"].setPosition(0.045)

###################
#
# PATH PLANNING
#
###################

if mode == 'planner':
    class Node:
        """
        Node for RRT Algorithm. This is what you'll make your graph with!
        """
        def __init__(self, pt, parent=None):
            self.point = pt # n-Dimensional point
            self.parent = parent # Parent node
            self.path_from_parent = [] # List of points along the way from the parent node (for edge's collision checking)
    
    def is_valid_vertex(map, point):
        '''
        Function that takes a map and an n-dimensional point and checks if it is not inside the obstacle
        :param map: The world map to sample points from
        :param point: n-Dimensional point
        :return: Boolean whose value depends on whether the point is valid or not
        '''
        if(map[point[0]][point[1]] == 0): # Free space would be marked as 0
            return True
        return False
        
    def get_closest_point(node_list, point):
        '''
        Function that finds a node in node_list with closest node.point to q_point
        :param node_list: List of Node objects
        :param point: n-dimensional array representing a point
        :return Node in node_list with closest node.point to query q_point
        '''
        # Initialize values
        closest_distance = 0
        closest_point_index = 0

        for i in range (len(node_list)): # traverse list of nodes
            q_point = (node_list[i]).point
            # distance between node and q accounting for n-dimensional array
            dist = math.sqrt(math.pow(point[0] - q_point[0], 2) + math.pow(point[1] - q_point[1], 2))
            
            # Finding new minimum distance
            if(i == 0):
                closest_point_index = i
                closest_distance = dist
            else:
                if(dist < closest_distance):
                    closest_point_index = i
                    closest_distance = dist
        
        return node_list[closest_point_index]
        
    def path_is_valid(map, path_from_parent):
        '''
        Function that checks if a path (or edge that is made up of waypoints) is collision free or not
        :param map: The world map to sample points from
        :param path_from_parent: A 1D array containing the n-dimensional points along an edge
        :return: Boolean based on whether the path is collision free or not
        '''
        for i in range(len(path_from_parent)): # traverse path
            if(map[round((path_from_parent[i])[0])][round((path_from_parent[i])[1])] == 1): # obstacle marked as 1
                return False
        return True 
    
    def get_random_valid_vertex(map):
        '''
        Function that samples a random 2-dimensional point which is valid (i.e. collision free and within the bounds)
        :param map: The world map to sample points from
        :return: n-Dimensional point
        '''
        # Intialize vertex value
        vertex = None
        
        while vertex is None:
            # Grab random x and y coordinates
            possible_x = random.randrange(0, 359)
            possible_y = random.randrange(0, 359)
            # Check if random point is obstacle free
            if(map[possible_x][possible_y] == 0):
                vertex = (possible_x, possible_y)
            
        return vertex
        
    def steer(from_point, to_point, delta_q):
        '''
        Function steers path towards goal with delta_q lengthed distances
        :param from_point: n-Dimensional array (point) where the path to "to_point" is originating from (e.g., [1.,2.])
        :param to_point: n-Dimensional array (point) indicating destination (e.g., [0., 0.])
        :param delta_q: Max path-length to cover, possibly resulting in changes to "to_point" (e.g., 0.2)
        :return path: Array of points leading from "from_point" to "to_point" (inclusive of endpoints)  (e.g., [ [1.,2.], [1., 1.], [0., 0.] ])
        '''
        # Initialize values
        f_point = np.array(from_point)
        t_point = np.array(to_point)
        dist = math.sqrt(math.pow(t_point[0] - f_point[0], 2) + math.pow(t_point[1] - f_point[1], 2))
    
        # If "to_point" can be used as-is, add to path 
        if(dist <= delta_q):
            path = np.linspace(f_point, t_point, num = 10)
        else:
            # Convert this point to another point on the same line, but with distance delta_q from f_point
            change_in_coordinates = (t_point - f_point) / dist
            change_in_coorinates_p = change_in_coordinates * delta_q
            new_coordinates = f_point + change_in_coorinates_p
            path = np.linspace(f_point, new_coordinates, num = 10)
    
        return path
    
    def retrieve_path(node_list):
        '''
        Function that retrieves path from the node_list
        :param node_list: List of Node objects
        :return list_of_points: Array of points leading from beginning to end of path
        '''
        # Intialize list and point
        list_of_points = []
        point = node_list[len(node_list)-1]
        
        while(point != None):
            # Insert point into list
            list_of_points.insert(0, point.point)
            # Update point to the parent
            point = point.parent
            
        return list_of_points
    
    # Implement RRT
    def path_planner(map, start, end, delta_q):
        '''
        Function using RRT to plan path from start to end point
        :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
        :param start: A tuple of indices representing the start cell in the map
        :param end: A tuple of indices representing the end cell in the map
        :param delta_q: Max path-length to cover
        :return: A list of tuples as a path from the given start to the given end in the given maze
        '''
        # Initializations
        node_list = []
        firstNode = Node(start, parent = None)
        node_list.append(firstNode)
        
        # Within 10000 iterations
        for iteration in range(10000):
            # Initialize point status to false
            addedPoint = False
            
            while(addedPoint == False):
                # Grab a new random point
                new_point = get_random_valid_vertex(map)
    
                #Every once in a while, if we are trying to reach a goal, force the algorithm to move directly towards the goal
                if (random.random() < 0.05):
                    new_point = end

                # Grab closetest point to random point
                closest_point = get_closest_point(node_list, new_point)
                
                # Steer towards random point by path length of delta_q
                possible_path = steer(closest_point.point, new_point, delta_q)
                
                # New path is valid
                if(path_is_valid(map, possible_path)):
                    # Grab new point
                    actual_point = possible_path[9] 
                    # Wrap point as a Node and set the new node's parent to the nearest node
                    node = Node(actual_point, parent = closest_point) 
                    # Initialize parent path with q_near's path from parent
                    node.path_from_parent = possible_path
                    # Append nodes list with new node
                    node_list.append(node)
                    # Update point status
                    addedPoint = True
    
                    #Check if new point is close enough to goal
                    if(True):
                        # Grab new start and end points
                        new_point_array = np.array(actual_point)
                        goal_point_array = np.array(end)
                        # Calculate new distance
                        dist_from_goal = math.sqrt(math.pow(new_point_array[0] - goal_point_array[0], 2) + math.pow(new_point_array[1] - goal_point_array[1], 2))
                        # New distance is close enough
                        if(dist_from_goal < 1e-5):
                            return node_list

        return node_list
            
# Load map (map.npy) from disk and visualize it
    map = np.load("map.npy")
    print("Map loaded")
    #plt.imshow(map)
    #plt.show()

    # Compute an approximation of the “configuration space”
    for x in range(360):
        for y in range(360):
            for i in range(-6, 6):
                for z in range(-6, 6):
                    if(map[x][y] == 1):
                        if(0 <= (x + i) <= 359 and 0 <= (y + z) <= 359):
                            newMap[x+i][y+z] = 1
                        newMap[x][y] = 1
    np.save("cspace.npy",newMap)
    print("Cspace map file saved")  

    # Initialize node_list and append coordinates of yellow blocks
    node_list = []
    node_list.append(path_planner(newMap, (120, 180), (164, 118), 8))
    node_list.append(path_planner(newMap, (164, 118), (208, 150), 8))
    node_list.append(path_planner(newMap, (208, 150), (145, 199), 8))
    node_list.append(path_planner(newMap, (145, 199), (146, 199), 8))
    node_list.append(path_planner(newMap, (146, 199), (170, 199), 8))
    node_list.append(path_planner(newMap, (170, 199), (191, 199), 8))
    node_list.append(path_planner(newMap, (191, 199), (223, 210), 8))
    node_list.append(path_planner(newMap, (223, 210), (208, 210), 8))
    node_list.append(path_planner(newMap, (208, 210), (140, 210), 8))
    node_list.append(path_planner(newMap, (140, 210), (222, 252), 8))
    node_list.append(path_planner(newMap, (222, 252), (255, 252), 8))

    # Initialize array
    correct_order_list = []
    
    # Append path to new array in the correct order
    for i in range(len(node_list)):
        correct_order_list.append(retrieve_path(node_list[i]))
    
    def smooth(path, weight_data = 0.5, weight_smooth = 0.1, tolerance = 0.00001):
        '''
        Function that prunes unnecessary waypoint in RRT path. 
        :param path: A 1D array containing n-dimensional points along a path
        :param weight_data: coefficient assigned to the data reflecting the computational importance
        :param weight_smooth: coefficient assigned to the smoothing reflecting the amount of smoothing applied
        :param tolerance: bound of smoothing effect
        :return: RRT path with smoothing 
        '''
        # Intialize array for new path
        newpath = [[0 for col in range(len(path[0]))] for row in range(len(path))]

        # Traverse old path & copy over to new path
        for i in range(len(path)):
            for j in range(len(path[0])):
                newpath[i][j] = path[i][j]

        # Intialize change value
        change = 1

        # Execute smoothing until tolerance value is reached
        while change > tolerance:
            # Re-initalize change value
            change = 0 

            # Traverse each point in path
            for i in range(1,len(path)-1):
                for j in range(len(path[0])):
                    # Intialize original point
                    ori = newpath[i][j]

                    # Smooth path with weighted values
                    newpath[i][j] = newpath[i][j] + weight_data*(path[i][j]-newpath[i][j])
                    newpath[i][j] = newpath[i][j] + weight_smooth*(newpath[i+1][j]+newpath[i-1][j]-2*newpath[i][j])
                    
                    # Update change value
                    change += abs(ori - newpath[i][j])
        
        return newpath 
    
    # Intialize array for smoothed path and append the smoothed points
    smooth_path = []
    for i in range(len(correct_order_list)):
        smooth_path.append(smooth(correct_order_list[i]))
    
    #Draw original path and smoothed path (yellow is smoothed, red is original)
    display.setColor(0xff0000)
    for i in range(len(correct_order_list)):
        for x in range(len(correct_order_list[i]) - 2):
            display.drawLine(int(correct_order_list[i][x][0]), int((correct_order_list[i][x][1])), int(correct_order_list[i][x+1][0]), int((correct_order_list[i][x+1][1])))
    
    display.setColor(0xffc50d)
    for i in range(len(smooth_path)):
        if(i == 1):
            display.setColor(0x65C3BC)
        for x in range(len(smooth_path[i]) - 1):
            display.drawLine(int(smooth_path[i][x][0]), int((smooth_path[i][x][1])), int(smooth_path[i][x+1][0]), int((smooth_path[i][x+1][1])))
    
    #Displays all obstacle pixels after plotting path
    for x in range(360):
        for y in range(360):
            if(newMap[x][y] > 0):
                display.setColor(0xFFFFFF)
                display.drawPixel(x, y)
    
    # Intialize arrays for individual paths and the final whole path
    individual_path = []
    final_path = []
    
    # Append each invididual path to the final path
    for x in range(len(smooth_path)):
        # Re-intialize individual path array
        individual_path = []

        # Adjust x and y coordinates and append to paths
        for i in range(len(smooth_path[x])):
            x_coor = (180 - (smooth_path[x][i])[0]) / -12
            y_coor = (180 - (smooth_path[x][i])[1]) / -12
            individual_path.append((x_coor, y_coor))
        final_path.append(individual_path)
    
    # Save path file
    np.save("path.npy", final_path)
    print("Path file saved")

###################
#
# MAPPING FROM FILES
#
###################

# Waiting a certain amount of timesteps for mapping
wait = 0

if mode == 'autonomous':
    # Load path from disk and visualize it
    waypoints = np.load("path.npy", allow_pickle=True)
    map = np.load("map.npy")
    
    # Draw Map
    for x in range(360):
        for y in range(360):
            if(map[x][y] > 0):
                display.setColor(0xFFFFFF)
                display.drawPixel(x, y)
    
    # Initialize target and path values
    currentTarget = 1
    currentPath = 0

    # Draw path
    color = 0xf5ef42
    display.setColor(color)
    for i in range(len(waypoints)):
        color = color + 0x000f0f
        if(color > 0xffffff):
            color = 0xffffff
        if(color < 0x000000):
            color = 0x000000
        display.setColor(color)
        for x in range(len(waypoints[i]) - 1):
            display.drawLine(int(180 + (waypoints[i][x][0] * 12)), int(180 + (waypoints[i][x][1] * 12)), int(180 + (waypoints[i][x+1][0] * 12)), int(180 + (waypoints[i][x+1][1] * 12)))

###################
#
# MAIN LOOP
#
###################

# Intialize iterators
iterator = 0
iterator2 = 0

# Set arm position depending on mode
if(mode != "autonomous mapping"):
    robot.getDevice("arm_1_joint").setPosition(0.07)
    robot.getDevice("arm_2_joint").setPosition(1.02)
    robot.getDevice("arm_3_joint").setPosition(-2)
    robot.getDevice("arm_4_joint").setPosition(2.29)
    robot.getDevice("arm_5_joint").setPosition(-1.0)
    robot.getDevice("arm_6_joint").setPosition(1.39)
    robot.getDevice("arm_7_joint").setPosition(-2.07)
else:
    robot.getDevice("arm_1_joint").setPosition(0.07)
    robot.getDevice("arm_2_joint").setPosition(0.6)
    robot.getDevice("arm_3_joint").setPosition(1.5)
    robot.getDevice("arm_4_joint").setPosition(2.29)
    robot.getDevice("arm_5_joint").setPosition(0)
    robot.getDevice("arm_6_joint").setPosition(0)
    robot.getDevice("arm_7_joint").setPosition(0)

while robot.step(timestep) != -1 and mode != 'planner':
###################
#
# LOCALIZATION
#
###################

    # Ground truth pose
    pose_y = -gps.getValues()[1]
    pose_x = -gps.getValues()[0]
    n = compass.getValues()
    # Adjust compass coordinates 
    rad = ((math.atan2(n[0], n[1])))
    pose_theta = rad

###################
#
# MAPPING WITH LIDAR
#
###################
    if (mode == "autonomous mapping"):
        lidar_sensor_readings = lidar.getRangeImage()
        lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
        if wait > 157: #if 157 timesteps have happened (~5s), do something
            for i, rho in enumerate(lidar_sensor_readings):
                alpha = lidar_offsets[i]
                if rho > LIDAR_SENSOR_MAX_RANGE:
                    continue
                # Adjust Webots coordinate system to match the robot-centric axes
                rx = -math.cos(alpha)*rho + 0.202
                ry = math.sin(alpha)*rho - 0.004

                # Convert detection from robot coordinates into world coordinates
                wx =  (math.cos(pose_theta)*rx - math.sin(pose_theta)*ry + pose_x)
                wy =  (math.sin(pose_theta)*rx + math.cos(pose_theta)*ry) + pose_y
    
                # Mapping with LIDAR sensors
                if rho < LIDAR_SENSOR_MAX_RANGE:
                # Visualize map gray values. 
                    if((180-int(wy*12) < 360) and (180-int(wx*12) < 360)):
                        if(map[180-int(wx*12)][180-int(wy*12)] < 1):
                            map[180-int(wx*12)][180-int(wy*12)] += 5e-3
                        else:
                            map[180-int(wx*12)][180-int(wy*12)] = 1
                            display.setColor(int(0xFFFFFF))
                            display.drawPixel(180-int(wx*12),180-int(wy*12))
                    
            # Draw the robot's current pose on the 360x360 display using red pixels
            display.setColor(int(0xFF0000))
            display.drawPixel(180-int(pose_x*12),180-int(pose_y*12))
        else:
            wait = wait + 1

    ###################
    #
    # MANUAL CONTROLLER
    #
    ###################

    # Robot Control
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            for x in range(0,360):
                for y in range(0,360):
                    if map[x][y] == 1: map[x][y] = True
                    else: map[x][y] = False
            map = np.multiply(map,1)
            np.save("map.npy",map)
            print("Map file saved")
        elif key == ord('L'):
            map = np.load("map.npy")
            print("Map loaded")
            plt.imshow(map)
            plt.show()
            newMap = np.load("cspace.npy")
            plt.imshow(newMap)
            plt.show()
        else: # slow down
            vL *= 0.75
            vR *= 0.75

    ###################
    #
    # AUTO ARM CONTROLLER
    #
    ###################
    
    elif(mode == "put into basket"):
        # Update Iterator
        iterator2 = iterator2 + 1

        if(iterator2 == 200):
            print("Swinging out!")
            
            if(shelf == "top"):
                robot.getDevice("arm_1_joint").setPosition(0.07)
                robot.getDevice("arm_2_joint").setPosition(0.75)
                robot.getDevice("arm_4_joint").setPosition(-0.32)
            elif(shelf == "middle"):
                robot.getDevice("arm_1_joint").setPosition(0.15)
                robot.getDevice("arm_2_joint").setPosition(0.1)
                robot.getDevice("arm_4_joint").setPosition(-0.32)
    
        if(iterator2 == 350):
            print("Going to basket!")
            robot.getDevice("arm_1_joint").setPosition(0.07)
            robot.getDevice("arm_2_joint").setPosition(1.02)
            robot.getDevice("arm_3_joint").setPosition(-2)
            robot.getDevice("arm_4_joint").setPosition(2.29)
            robot.getDevice("arm_5_joint").setPosition(-1.0)
            robot.getDevice("arm_6_joint").setPosition(1.39)
            robot.getDevice("arm_7_joint").setPosition(-2.07)
        
        if(iterator2 == 700):
            print("Releasing into basket")
            robot_parts["gripper_left_finger_joint"].setPosition(0.045)
            robot_parts["gripper_right_finger_joint"].setPosition(0.045)
            iterator = 0
            updown = 0
            leftright = 0
            forwardback = 0
        if(iterator2 == 750):
             mode = "autonomous"

    ###################
    #
    # MANUAL ARM CONTROLLER
    #
    ###################
         
    elif(mode == "user control"):
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass

        if(key == ord('L')):
            print("adjusting wrist left")
            save_wrist_joint = save_wrist_joint + 0.05
            if(save_wrist_joint < -1.39):
                save_wrist_joint = -1.39
            if(save_wrist_joint > 1.39):
                save_wrist_joint = 1.39
            robot.getDevice("arm_6_joint").setPosition(save_wrist_joint)
        elif(key == ord('R')):
            print("adjusting wrist right")
            save_wrist_joint = save_wrist_joint - 0.05
            if(save_wrist_joint < -1.39):
                save_wrist_joint = -1.39
            if(save_wrist_joint > 1.39):
                save_wrist_joint = 1.39
            robot.getDevice("arm_6_joint").setPosition(save_wrist_joint) 
        elif(key == ord('W')):
            print("driving forward")
            robot_parts["wheel_left_joint"].setVelocity(2.5)
            robot_parts["wheel_right_joint"].setVelocity(3)  
        elif(key == ord('S')):
            print("driving backward")
            robot_parts["wheel_left_joint"].setVelocity(-3)
            robot_parts["wheel_right_joint"].setVelocity(-2.5) 
        elif(key == ord('G')):
            print("Grabbing!")
            robot_parts["gripper_left_finger_joint"].setPosition(0.0)
            robot_parts["gripper_right_finger_joint"].setPosition(0.0)
        elif(key == ord('B')):
            print("Making guess")
            iterator = 0
            iterate_reset = 0
            mode = "reset arm position"
        elif(key == ord('O')):
            print("Opening claws")
            robot_parts["gripper_left_finger_joint"].setPosition(0.045)
            robot_parts["gripper_right_finger_joint"].setPosition(0.045)
        elif(key == ord('F')):
            print("putting into basket")
            mode = "put into basket"
            iterator2 = 0
        elif(key == ord('D')):
            updown = updown - 0.07
            print("user moving arm down")
            offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
            try:
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position, target_orientation = [0,0,1], orientation_mode="Y") 
            except ValueError:
                print("Infeasible position: ", updown)
                forwardback = 0
                updown = 0
                leftright = 0
                offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=og_position, target_orientation = [0,0,1], orientation_mode="Y") 
                for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])  
                pass
            for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
                    
        elif(key == ord('U')):
            updown = updown + 0.07
            print("user moving arm up")
            offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
            try:
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position, target_orientation = [0,0,1], orientation_mode="Y") 
            except ValueError:
                print("Infeasible position: ", updown)
                forwardback = 0
                updown = 0
                leftright = 0
                offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=og_position, target_orientation = [0,0,1], orientation_mode="Y") 
                for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])  
                pass
            
            for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
                        
        elif key == keyboard.LEFT:
            leftright = leftright + 0.07
            print("user moving arm left")
            offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
            try:
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position, target_orientation = [0,0,1], orientation_mode="Y") 
            except ValueError:
                print("Infeasible position: ", leftright)
                forwardback = 0
                updown = 0
                leftright = 0
                offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=og_position, target_orientation = [0,0,1], orientation_mode="Y") 
                for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])  
                pass
            
            for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
                        
        elif key == keyboard.RIGHT:
            leftright = leftright - 0.07
            print("user moving arm right")
            offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
            try:
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position, target_orientation = [0,0,1], orientation_mode="Y") 
            except ValueError:
                print("Infeasible position: ", leftright)
                forwardback = 0
                updown = 0
                leftright = 0
                offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=og_position, target_orientation = [0,0,1], orientation_mode="Y") 
                for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])  
                pass
            
            for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
                     
        elif key == keyboard.UP:
            forwardback = forwardback + 0.07
            print("user moving arm forward", forwardback)
            offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
            try:
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position, target_orientation = [0,0,1], orientation_mode="Y") 
            except ValueError:
                print("Infeasible position: ", forwardback)
                forwardback = 0
                updown = 0
                leftright = 0
                offset_target = [depth_tracker+0.19+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=og_position, target_orientation = [0,0,1], orientation_mode="Y") 
                for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])  
                pass
            
            for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])

        elif key == keyboard.DOWN:
            forwardback = forwardback - 0.07
            print("user moving arm back", forwardback)
            offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 + leftright, new_y+1.2 + updown]
            initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
            try:
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position, target_orientation = [0,0,1], orientation_mode="Y") 
            except ValueError:
                print("Infeasible position: ", forwardback)
                forwardback = 0
                updown = 0
                leftright = 0
                offset_target = [depth_tracker+0.19+forwardback, new_x + 0.05 +leftright, new_y+1.2 + updown]
            
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=og_position, target_orientation = [0,0,1], orientation_mode="Y") 
                for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])  
                pass
            for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
        else:
            robot_parts["wheel_left_joint"].setVelocity(0)
            robot_parts["wheel_right_joint"].setVelocity(0)
    
    elif(mode == "initial grab"):
        if(iterator < 10000):
            if(iterator == 100):       
                ###################
                #
                # COMPUTER VISION
                #
                ###################

                # Initialize global variable for computer vision
                color_ranges = [] 

                # Initialize image dimensions
                img_height = camera.getHeight()
                img_width = camera.getWidth()
            
                def check_if_color_in_range(rgb_tuple):
                  '''
                  Function checks in given tubles are in target range
                  :param rgb_tuple: Tuple of RGB values
                  :returns Boolean: True if rgb_tuple is in any of the color ranges specified in color_ranges
                  '''
                  global color_ranges
                  # Grab bounds
                  for entry in color_ranges:
                    lower, upper = entry[0], entry[1]
                    in_range = True
                    # Check if pixel color is in range
                    for i in range(len(rgb_tuple)):
                      if rgb_tuple[i] < lower[i] or rgb_tuple[i] > upper[i]:
                        in_range = False
                        break
                    if in_range: return True
                  return False
                
                def do_color_filtering(img):
                  '''
                  Function takes an RGB image as input, and create a "mask image" to filter out irrelevant pixels
                  :param img: img from robot's camera
                  :return: image mask of only relevant pixels
                  '''
                  # Create a matrix of dimensions [height, width] using numpy
                  mask = np.zeros([img_height, img_width]) # Index mask as [height, width] (e.g.,: mask[y,x])
                  
                  # Iterate through pixels to check if color is in range
                  for x in range(0,(img_width-1)):
                    for y in range(0,(img_height-1)):
                      if check_if_color_in_range(img[x, y]):
                        mask[y, x] = 1 # If color is in range, mark mask as 1
                  return mask
                
                def expand_nr(img_mask, cur_coord, coordinates_in_blob):
                  '''
                  Function finds all of the non-zero pixels connected to a location non-recursively
                  :param img_mask: image mask 
                  :param cur_coord: current coordinate to start blob
                  :param coordinates_in_blob: list of coordinates that make up the blob
                  :return: list of coordinates that make up a blob
                  '''
                  coordinates_in_blob = [] # Initialize blob coordinate list
                  coordinate_list = [cur_coord] # List of all coordinates to try expanding to
                  while len(coordinate_list) > 0: # While the list is not empty
                    cur_coordinate = coordinate_list.pop() # Take the first coordinate in the list and perform 'expand' on it
                    # Check to make sure cur_coordinate is in bounds, otherwise 'continue'
                    if cur_coordinate[0] < 0 or cur_coordinate[1] < 0 or cur_coordinate[0] >= img_mask.shape[0] or cur_coordinate[1] >= img_mask.shape[1]: 
                      continue
                    # Check to see if the value is 0, if so, 'continue'
                    if img_mask[cur_coordinate[0], cur_coordinate[1]] == 0.0: 
                      continue
                    # Set image mask at this coordinate to 0 to avoid double counting
                    img_mask[cur_coordinate[0],cur_coordinate[1]] = 0
                    # Add this coordinate to 'coordinates_in_blob'
                    coordinates_in_blob.append(cur_coordinate)
                    # Add all neighboring coordinates (above, below, left, right) to coordinate_list to expand to them
                    above = [cur_coordinate[0]-1, cur_coordinate[1]]
                    below = [cur_coordinate[0]+1, cur_coordinate[1]]
                    left = [cur_coordinate[0], cur_coordinate[1]-1]
                    right = [cur_coordinate[0], cur_coordinate[1]+1]
                    coordinate_list.append(above)
                    coordinate_list.append(below)
                    coordinate_list.append(left)
                    coordinate_list.append(right)
                  return coordinates_in_blob
                
                def get_blobs(img_mask):
                  '''
                  Function takes a mask image as input and groups each blob of non-zero pixels as a detected object
                  :param img_mask: image mask 
                  :return: list of lists containing the coordinates of each pixel belonging to each blob
                  '''
                  # Copy image mask into local variable to make changes to
                  img_mask_copy = img_mask 
                  blobs_list = [] # List of all blobs, each element being a list of coordinates belonging to each blob
                  blob_coords = [] # Initialize empty list of blob coordinates for expansion
                  # Iterate through all coordinates in img_mask
                  for y in range(0,(img_height-1)):
                    for x in range(0,(img_width-1)):
                    # If mask value at [y,x] is 1, call expand_nr on copy of image mask and coordinate (y,x), giving a third argument of an empty list to populate with blob_coords.
                      if img_mask_copy[y,x] == 1:
                        blob_coords = expand_nr(img_mask_copy,(y,x), blob_coords)
                        # Add blob_coords to blobs_list
                        blobs_list.append(blob_coords)
                        blob_coords = [] # reintialize empty list
                  return blobs_list
                
                def get_blob_centroids(blobs_list):
                  '''
                  Function takes a list of blobs' coordinate lists and calculates each blob's centroid
                  :param blobs_list: lists of blobs in image
                  :return: list of coordinates of each blob's center
                  '''
                  object_positions_list = [] # Intialize object position list
                  # Blob centroid calculation
                  for l in blobs_list:
                    if len(l) >= 10:
                      n = len(l)
                      lx = []
                      ly = []
                      for i in range(0,(n-1)): # Seperating x and y coordinates
                        lx.append(l[i][0])
                        ly.append(l[i][1])
                      # Manually finding centroid of each blob
                      centroidx = np.min(lx)+((np.max(lx) - np.min(lx))/2)
                      centroidy = np.min(ly)+((np.max(ly) - np.min(ly))/2)
                      centroid = [centroidx,centroidy]
                      object_positions_list.append(centroid)
                  return object_positions_list
             
                # Grab image from camera & convert to appropriate array shape
                imgRaw = np.array(camera.getImageArray())
                img = imgRaw.view([(f'f{i}',imgRaw.dtype) for i in range(imgRaw.shape[-1])])[...,0].astype('O')
                
                # Create img_mask of all foreground pixels, where foreground is defined as passing the color filter
                color_ranges.append([[204,204,0], [255,255,153]]) # Detect darkest & lightest yellow 
                img_mask = do_color_filtering(img) # mask for detecting yellow objects
       
                # Find all the blobs in the img_mask
                blobs = get_blobs(img_mask)
                
                # Get the centroids of the img_mask blobs
                object_positions_list = get_blob_centroids(blobs) 
                if len(object_positions_list) != 0:
                    for obj_pos in object_positions_list:
                        obj_pos_vector = np.array(obj_pos).astype(np.int32) 

                #We can hard-code the up/down of the arm which can be either the top or bottom shelf.
                #For the left/right, use the range given by the recognized objects
                #This range is 1 if the object is on the far left and -1 for far right
                
                #To get the left/right we can make a linspace array with numpy with 140 evenly spaced entries between -1 and 1 and 
                #use the ratio between the x value given by the color blob (x / 140)
                
                # Grab x and y coordinates from computer vision
                x = int(obj_pos_vector[1] * 2) 
                y = int(obj_pos_vector[0] * 2)
                
                # Adjust coordinates for depth
                y_for_depth = int(round((obj_pos_vector[1] * 64) / 240)) 
                x_for_depth = int(round((obj_pos_vector[0] * 64) / 135)) 
                
                # Intialize values for range finder
                image = kinect.getRangeImage()
                width = kinect.getWidth()
                height = kinect.getHeight()

                # Depth Tracking Block
                depth_tracker = -1
                for i in range(x_for_depth - 3, x_for_depth + 3):
                    for z in range(y_for_depth - 3, y_for_depth + 3):
                        if(1 <= i <= 64 and 1 <= z <= 64):
                            depth = kinect.rangeImageGetDepth(image, width, i, z)
                            if(depth > depth_tracker and depth < 10000000):
                                depth_tracker = depth                
                if(depth_tracker == -1):
                    print("missed object on depth scanner so guessing")
                    depth_tracker = 0.6
                spacingx = np.linspace(-1, 1, num = 480)
                spacingy = np.linspace(-0.5, 0.5, num = 270)
                
                if(x > 480):
                    x = 480
                if(x < 1):
                    x = 1
                new_x = spacingx[480 - x]
                
                if(y > 270):
                    y = 270
                if(y < 1):
                    y = 1
                new_y = spacingy[270 - y]  
                 
                # Locating object
                if(-0.5 < new_y < 0): 
                    print("Object is on middle shelf") 
                    shelf = "middle"
                    new_y = -0.46
                elif(0 < new_y < 0.5):
                    print("Object is on top shelf")
                    shelf = "top"
                    new_y = 0.037
                
                print("x calculated by color blobbing: ", new_x)
                print("y calculated by color blobbing: ", new_y)
                print("depth computed by rangefinder: ", depth_tracker)                            
                recognized_objects = camera.getRecognitionObjects()
                target = recognized_objects[0].getPosition() # This is the position of the target in camera coordinates
                print("x, y, z", target[0], target[1], target[2])
                """
                target[0] is the distance
                target[1] is the left/right
                target[2] is the up/down
                
                """
                offset_target = [depth_tracker+0.19, new_x + 0.05, new_y+1.2]
                print("target: ", target)
                initial_position = [0,0,0,0] + [m.getPositionSensor().getValue() for m in motors] + [0,0,0,0]
                og_position = initial_position
                ikResults = my_chain.inverse_kinematics(offset_target, initial_position=initial_position, target_orientation = [0,0,5], orientation_mode="Y") 

            # Continuation of manual arm controller 
            key = keyboard.getKey()
            if(300 < iterator < 450 and key == ord('A')):
                print("user aborted initial guess")
                mode = "user control"
                
            # Continuation of auto arm controller
            if(iterator == 300):
                print("Getting into position")
                if(shelf == "top"):
                    robot.getDevice("arm_2_joint").setPosition(1)
                elif(shelf == "middle"):
                    robot.getDevice("arm_2_joint").setPosition(-0.1)
            
            if(iterator == 450):
                print("Reaching for item!")
                for res in range(len(ikResults)):
                    # This if check will ignore anything that isn't controllable
                    if my_chain.links[res].name in part_names:
                        robot.getDevice(my_chain.links[res].name).setPosition(ikResults[res])
                        if(my_chain.links[res].name == "arm_2_joint"):
                            save_height_joint = ikResults[res]
                        if(my_chain.links[res].name == "arm_6_joint"): 
                            save_wrist_joint = ikResults[res]
                mode = "user control"
                pass
            iterator = iterator + 1
    
    # Continuation of controller
    elif(mode == "back up"):
        print("backing up")

    elif(mode == "reset arm position"):
        iterate_reset = iterate_reset + 1
        if(iterate_reset == 3):   
            robot.getDevice("arm_1_joint").setPosition(0.07)
            robot.getDevice("arm_2_joint").setPosition(0)
            robot.getDevice("arm_3_joint").setPosition(0)
            robot.getDevice("arm_4_joint").setPosition(-0.25)
            robot.getDevice("arm_5_joint").setPosition(0)
            robot.getDevice("arm_6_joint").setPosition(0)
            robot.getDevice("arm_7_joint").setPosition(0)
        if(iterate_reset == 100):
            mode = "initial grab"
    
    elif(mode == "turn towards target first time"):        
        if(face_direction[currentPath - 1] == "south"):
            upperbound = -1.59
            lowerbound = -1.68
        else:
            upperbound = 1.59
            lowerbound = 1.55
       
        if(lowerbound < pose_theta < upperbound):
            print("facing target now")
            robot_parts["wheel_left_joint"].setVelocity(0)
            robot_parts["wheel_right_joint"].setVelocity(0)
            iterate_closer = 0
            iterate_reset = 0
            mode = "reset arm position"
        else:
            robot_parts["wheel_left_joint"].setVelocity(MAX_SPEED * 0.2)
            robot_parts["wheel_right_joint"].setVelocity(-MAX_SPEED * 0.2)
 
    elif(mode == "autonomous mapping"):
        # Intialize values
        rho = 0
        desired_x = mapping_array[currentTarget][0]
        desired_y = mapping_array[currentTarget][1]  
        pose_x = -pose_x
        pose_y = -pose_y
        
        # Calculate Euclid Error
        p = math.sqrt(math.pow(pose_x - desired_x, 2) + math.pow(pose_y - desired_y, 2))
            
        if(p <= 0.50 and currentTarget < (len(mapping_array) - 1)):
            print("reached target", currentTarget + 1)
            currentTarget = currentTarget + 1
        elif(p <= 0.05 and currentTarget == (len(mapping_array) - 1)):
            print("reached final destination")
            print("pose_theta", pose_theta)
            print("pose_x", pose_x)
            print("pose_y", pose_y)
            currentTarget = 0

        # Bearing Error
        a = math.atan2(desired_y-pose_y, desired_x-pose_x)-pose_theta
        # x dot
        dX = p * 0.01
        # theta dot
        dTheta = a

        # Compute wheelspeeds
        vL = (dX - ((dTheta*AXLE_LENGTH)/2))/(MAX_SPEED_MS/MAX_SPEED)
        vR = (dX + ((dTheta*AXLE_LENGTH)/2))/(MAX_SPEED_MS/MAX_SPEED)
        
        # Proportional velocities
        if(abs(vL / vR) >=  abs(vR / vL)):
            if(abs(vL) > 1):
                vR = vR / abs(vL)
                vL = vL / abs(vL)
            vL = vL * MAX_SPEED * 0.5
            vR = vR * MAX_SPEED * 0.5
        else: 
            if(abs(vR) > 1):
                vL = vL / abs(vR)
                vR = vR / abs(vR)
            vR = vR * MAX_SPEED * 0.5
            vL = vL * MAX_SPEED * 0.5

        if(vL > MAX_SPEED or vL < -MAX_SPEED or vR > MAX_SPEED or vR < -MAX_SPEED): 
            if(vL < 0): 
                vL = -MAX_SPEED * 0.5
            if(vR < 0):
                vR = -MAX_SPEED * 0.5
            if(vL > 0):
                vL = MAX_SPEED * 0.5
            if(vR > 0):
                vR = MAX_SPEED * 0.5

        # Actuator commands
        robot_parts["wheel_left_joint"].setVelocity(vL)
        robot_parts["wheel_right_joint"].setVelocity(vR)
    
        key = keyboard.getKey()
        if key == ord('S'):
            for x in range(0,360):
                for y in range(0,360):
                    if map[x][y] == 1: 
                        map[x][y] = True
                    else: 
                        map[x][y] = False
            map = np.multiply(map,1)
            np.save("map.npy",map)
            print("Map file saved")
            
    elif(mode == "autonomous"):
        key = keyboard.getKey()

        display.setColor(int(0xFF0000))
        display.drawPixel(180-int(pose_x*12),180-int(pose_y*12))
        
        if(driveMode != "at destination"):
            # Feedback controller
            # Intialize values
            rho = 0
            desired_x = waypoints[currentPath][currentTarget][0]
            desired_y = waypoints[currentPath][currentTarget][1]
            pose_x = -pose_x
            pose_y = -pose_y
               
            # Calculate Euclid Error
            p = math.sqrt(math.pow(pose_x - desired_x, 2) + math.pow(pose_y - desired_y, 2))
            
            if(p <= 0.5 and currentTarget < (len(waypoints[currentPath]) - 1)):
                print("reached target", currentTarget + 1)
                currentTarget = currentTarget + 1
            elif(p <= 0.07 and currentTarget == (len(waypoints[currentPath]) - 1) and currentPath != (len(waypoints) - 1)):
                print("reached final destination")
                print("pose_theta", pose_theta)
                print("pose_x", pose_x)
                print("pose_y", pose_y)
                old_a = -4000
                currentPath = currentPath + 1
                currentTarget = 1
                mode = "turn towards target first time"
            elif(p <= 0.07 and currentTarget == (len(waypoints[currentPath]) - 1)):
                print("reached final destination")
                print("pose_theta", pose_theta)
                print("pose_x", pose_x)
                print("pose_y", pose_y)
                currentPath = currentPath + 1
                old_a = -4000
                print("This should be the final destination")
                #driveMode = "at destination"
                mode = "turn towards target first time"
                
            # Bearing Error
            a = math.atan2(desired_y-pose_y, desired_x-pose_x)-pose_theta
            
            if(abs(old_a - a) > 5 and old_a != -4000):
                if(currentTarget < len(waypoints[currentPath]) - 1):
                    currentTarget = currentTarget + 1               
            
            # x dot
            dX = p * 0.05
    
            # theta dot
            dTheta = a
    
            # Compute wheelspeeds
            vL = (dX - ((dTheta*AXLE_LENGTH)/2))/(MAX_SPEED_MS/MAX_SPEED)
            vR = (dX + ((dTheta*AXLE_LENGTH)/2))/(MAX_SPEED_MS/MAX_SPEED)
            
            # Proportional velocities
            if(abs(vL / vR) >=  abs(vR / vL)):
                if(abs(vL) > 1):
                    vR = vR / abs(vL)
                    vL = vL / abs(vL)
                vL = vL * MAX_SPEED * 0.5
                vR = vR * MAX_SPEED * 0.5
            else: 
                if(abs(vR) > 1):
                    vL = vL / abs(vR)
                    vR = vR / abs(vR)
                vR = vR * MAX_SPEED * 0.5
                vL = vL * MAX_SPEED * 0.5

            old_a = a
            if(vL > MAX_SPEED or vL < -MAX_SPEED or vR > MAX_SPEED or vR < -MAX_SPEED): 
                if(vL < 0): 
                    vL = -MAX_SPEED * 0.5
                if(vR < 0):
                    vR = -MAX_SPEED * 0.5
                if(vL > 0):
                    vL = MAX_SPEED * 0.5
                if(vR > 0):
                    vR = MAX_SPEED * 0.5

            # Actuator commands
            robot_parts["wheel_left_joint"].setVelocity(vL)
            robot_parts["wheel_right_joint"].setVelocity(vR)

    if(driveMode == "at destination"):
        vL = 0
        vR = 0
    TIAGO_MAX_WHEEL_SPEED = MAX_SPEED_MS*timestep/1000.0
