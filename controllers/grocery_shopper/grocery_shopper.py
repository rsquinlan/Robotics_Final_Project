"""grocery controller."""

# Nov 2, 2022

from controller import Robot, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
import random

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
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint",
              "gripper_left_finger_joint","gripper_right_finger_joint")

# 

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.35, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf',0.045,0.045)

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

# Enable GPS and compass localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Enable LiDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Enable display
display = robot.getDevice("display")

# Enable Keyboard
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = np.zeros(shape=[900,480])
mode = 'planner'

# ------------------------------------------------------------------
# Helper Functions
if mode == 'planner':
    map = np.load("map.npy")
    #plt.imshow(map)
    
    config_space = np.zeros(map.shape)
   
    for i in range(map.shape[0]):
        for j in range(20):
            config_space[i, j] = 1
            
    for i in range(map.shape[0]):
        for j in range(map.shape[1]-20, map.shape[1]):
            config_space[i, j] = 1
            
    for j in range(map.shape[1]):
        for i in range(20):
            config_space[i, j] = 1
            
    for j in range(map.shape[1]):
        for i in range(map.shape[0]-20, map.shape[0]):
            config_space[i, j] = 1
    
    for i in range(20, map.shape[0]-20):
        for j in range(20, map.shape[1]-20):
            if(map[i, j] > 0):
                for r in range(-9, 10):
                    for c in range(-9, 10):
                        config_space[i + r, j + c] = 1
    plt.imshow(config_space)
    
    def state_is_valid(point):
        return config_space[int(point[1]), int(point[0])] == 0
            
    class Node:
        """
        Node for RRT Algorithm. This is what you'll make your graph with!
        """
        def __init__(self, pt, parent=None):
            self.point = pt # n-Dimensional point
            self.parent = parent # Parent node
            self.path_from_parent = [] # List of points along the way from the parent node (for edge's collision checking)
    
    def get_nearest_vertex(node_list, q_point):
        min_len = 1e100000 # functionally infinite
        min_node = None
    
        for node in node_list:
            dist = math.dist(node.point, q_point)
            if dist < min_len:
                min_len = dist
                min_node = node
        
        return min_node
    
    def steer(from_point, to_point, delta_q):
        from_point = np.asarray(from_point)
        to_point = np.asarray(to_point)
        
        dist = math.dist(from_point, to_point)
        if dist > delta_q:
            vector = to_point - from_point
            scaled_vector = (vector/dist)*delta_q
            to_point = from_point + scaled_vector
    
        path = np.linspace(from_point, to_point, 10)
        new_path = []
        for point in path:
            new_path.append([int(point[0]), int(point[1])])
        return path
        
    def check_path_valid(path, state_is_valid):
        valid = True
        for point in path:
            if not state_is_valid(point):
                valid = False
    
        return valid
        
    def rrt(state_bounds, starting_point, goal_point, k, delta_q):
        node_list = []
        node_list.append(Node(starting_point, parent=None)) # Add Node at starting point with no parent
    
        # TODO: Your code here
        # TODO: Make sure to add every node you create onto node_list, and to set node.parent and node.path_from_parent for each
    
        for i in range(k):
            # generate random point
            q_rand = []
            if goal_point is not None and np.random.rand() < 0.05:
                q_rand = goal_point
            else:
                for dim in state_bounds:
                    rand = (np.random.rand() * (dim[1] - dim[0])) + dim[0] # random float between dim[0] and dim[1]
                    rand = int(rand)
                    q_rand.append(rand)
    
            # find nearest node in node_list
            q_near = get_nearest_vertex(node_list, q_rand)
    
            # get path (scaled down if necessary)
            path = steer(q_near.point, q_rand, delta_q)
    
            q_new = path[-1] # next point is final point in path (differs from q_rand if scaling is done)
            print(q_near.point, q_rand)
            # create new node if the path is valid
            if check_path_valid(path, state_is_valid):
                new_node = Node(q_new, q_near)
                new_node.path_from_parent = path
    
                node_list.append(new_node)
    
                # return early if goal was found
                if goal_point is not None and abs(math.dist(q_new, goal_point)) < 1e-5:
                    return node_list
    
    
        return node_list
    print (config_space.shape)
    bounds = np.array([[0,479],[0,899]])
    start = [251, 585]
    
    nodes = rrt(bounds, start, None, 500, 50)
    
    waypoints = [node.point for node in nodes]
    paths = [node.path_from_parent for node in nodes]
    path_points = []
    for path in paths:
        for point in path:
            path_points.append([point[0], point[1]])
    
    plt.scatter([i[0] for i in path_points], [i[1] for i in path_points], c='blue')
    plt.scatter([i[0] for i in waypoints], [i[1] for i in waypoints], c='red')
    
    
    plt.show()
    
    
gripper_status="closed"

# Main Loop
while robot.step(timestep) != -1:
    vL = 0
    vR = 0
    
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
            # Part 1.4: Filter map and save to filesystem
            for row in range(900):
                for col in range(480):
                    if map[row, col] > .8:
                        map[row, col] = 1
                    else:
                        map[row, col] = 0
                        
            np.save("map.npy", map)
            plt.imshow(map)
            plt.show()
            print("Map file saved")
    
    # Localization
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    
    n = compass.getValues()
    rad = -((math.atan2(n[0], n[1]))-1.5708)
    pose_theta = rad
    
    #print("X: %f Y: %f Theta: %f" % (pose_x, pose_y, pose_theta))
    
    # Mapping
    lidar_sensor_readings = lidar.getRangeImage()
    lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]

    for i, rho in enumerate(lidar_sensor_readings):
        alpha = lidar_offsets[i]

        if rho > LIDAR_SENSOR_MAX_RANGE:
            continue

        # The Webots coordinate system doesn't match the robot-centric axes we're used to
        rx = math.cos(alpha)*rho
        ry = -math.sin(alpha)*rho

        t = pose_theta + np.pi/2.
        # Convert detection from robot coordinates into world coordinates
        wx =  math.cos(t)*rx - math.sin(t)*ry + pose_x
        wy =  math.sin(t)*rx + math.cos(t)*ry + pose_y

        ################ ^ [End] Do not modify ^ ##################

        #print("Rho: %f Alpha: %f rx: %f ry: %f wx: %f wy: %f" % (rho,alpha,rx,ry,wx,wy))
        if wx >= 15:
            wx = 14.999
        if wx <= -15:
            wx = -14.999
        if wy >= 8:
            wy = 7.999
        if wy <= -8:
            wy = -7.999
            
        wx = -wx + 15
        wy = -wy + 8
        
        if rho < LIDAR_SENSOR_MAX_RANGE:
            # Part 1.3: visualize map gray values.
 
            # You will eventually REPLACE the following lines with a more robust version of the map
            # with a grayscale drawing containing more levels than just 0 and 1.
            map[abs(int(wx*30)),abs(int(wy*30))] += .005
    
    robot_parts["wheel_left_joint"].setVelocity(vL)
    robot_parts["wheel_right_joint"].setVelocity(vR)
    
    if(gripper_status=="open"):
        # Close gripper, note that this takes multiple time steps...
        robot_parts["gripper_left_finger_joint"].setPosition(0)
        robot_parts["gripper_right_finger_joint"].setPosition(0)
        if right_gripper_enc.getValue()<=0.005:
            gripper_status="closed"
    else:
        # Open gripper
        robot_parts["gripper_left_finger_joint"].setPosition(0.045)
        robot_parts["gripper_right_finger_joint"].setPosition(0.045)
        if left_gripper_enc.getValue()>=0.044:
            gripper_status="open"
