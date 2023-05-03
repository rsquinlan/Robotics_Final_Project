"""grocery controller."""

# Nov 2, 2022

from controller import Robot, Keyboard
import math
import numpy as np
from matplotlib import pyplot as plt
import random
import cv2 

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
mode = 'move'



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
    
    nodes = rrt(bounds, start, None, 1000, 50)
    
    waypoints = [node.point for node in nodes]
    paths = [node.path_from_parent for node in nodes]
    path_points = []
    for path in paths:
        for point in path:
            path_points.append([point[0], point[1]])
    
    plt.scatter([i[0] for i in path_points], [i[1] for i in path_points], c='blue')
    plt.scatter([i[0] for i in waypoints], [i[1] for i in waypoints], c='red')
    
    np.save("path.npy", waypoints)
    
    plt.show()
    
    
gripper_status="closed"

if mode == 'autonomous':
      #####################################################
  #                 Computer Vision                   #
  #####################################################
  color_ranges = [] #initialize global variable for computer vision
  #initialize image dimensions
  img_height = camera.getHeight()
  img_width = camera.getWidth()

  def check_if_color_in_range(rgb_tuple):
    global color_ranges
    # grab bounds
    for entry in color_ranges:
      lower, upper = entry[0], entry[1]
      in_range = True
      # check if pixel color is in range
      for i in range(len(rgb_tuple)):
        if rgb_tuple[i] < lower[i] or rgb_tuple[i] > upper[i]:
          in_range = False
          break
      if in_range: return True
    return False

  def do_color_filtering(img):
    # Create a matrix of dimensions [height, width] using numpy
    mask = np.zeros([img_height, img_width]) # Index mask as [height, width] (e.g.,: mask[y,x])
    #iterate through pixels to check if color is in range
    for x in range(0,(img_width-1)):
      for y in range(0,(img_height-1)):
        if check_if_color_in_range(img[x, y]):
          mask[y, x] = 1 #if color is in range, mark mask as 1
    return mask

  def expand_nr(img_mask, cur_coord, coordinates_in_blob):
    coordinates_in_blob = [] #initialize blob coordinate list
    coordinate_list = [cur_coord] # List of all coordinates to try expanding to
    while len(coordinate_list) > 0: #while the list is not empty
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
    # Copy image mask into local variable to make changes to
    img_mask_copy = img_mask #copy.copy(img_mask)
    blobs_list = [] # List of all blobs, each element being a list of coordinates belonging to each blob
    blob_coords = [] # Initialize empty list of blob coordinates for expansion
    # Iterate through all coordinates in img_mask
    #print("img_height: ", img_height)
    #print("img_width: ", img_width)
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
    object_positions_list = [] #intialize object position list
    # blob centroid calculation
    for l in blobs_list:
      if len(l) >= 10:
        n = len(l)
        lx = []
        ly = []
        for i in range(0,(n-1)): # seperating x and y coordinates
          lx.append(l[i][0])
          ly.append(l[i][1])
        # manually finding centroid of each blob
        centroidx = np.min(lx)+((np.max(lx) - np.min(lx))/2)
        centroidy = np.min(ly)+((np.max(ly) - np.min(ly))/2)
        centroid = [centroidx,centroidy]
        object_positions_list.append(centroid)
    return object_positions_list

  imgRaw = np.array(camera.getImageArray())
  print(dir(camera))
  img = imgRaw.view([(f'f{i}',imgRaw.dtype) for i in range(imgRaw.shape[-1])])[...,0].astype('O')

  ########## PART 1 ############
  # Create img_mask of all foreground pixels, where foreground is defined as passing the color filter
  ## color ranges are initially seperated to create more precise masks for each block 
  color_ranges.append([[204,204,0], [255,255,153]]) # Detect darkest & lightest yellow

  img_mask = do_color_filtering(img) # mask for detecting yellow objects
  ########## PART 2 ############
  # Find all the blobs in the img_mask
  blobs = get_blobs(img_mask)
  ########## PART 3 ############
  # Get the centroids of the img_mask blobs
  object_positions_list = get_blob_centroids(blobs) 
  if len(object_positions_list) != 0:
      #print("object_positions_list: ", object_positions_list)
      for obj_pos in object_positions_list:
          obj_pos_vector = np.array(obj_pos).astype(np.int32) # In case your object positions weren't numpy arrays
      # TODO: translate position on image to position in world or relative to gripper
      #print("Object pos: " + str(obj_pos_vector))
    

# Main Loop
while robot.step(timestep) != -1:
   
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
            
    if mode == 'move':
        old_a = -4000
        driveMode = ""
        waypoints = [[205, 800]]
        
        key = keyboard.getKey()
    
        display.setColor(int(0xFF0000))
        display.drawPixel(180-int(pose_x*12),180-int(pose_y*12))
        
        if(driveMode != "at destination"):
            # Feedback controller
            # Intialize values
            rho = 0
            desired_x = waypoints[0][0]
            desired_y = waypoints[0][1]
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
            elif(p <= 0.07 and currentTarget == (len(waypoints[currentPath]) - 1)):
                print("reached final destination")
                print("pose_theta", pose_theta)
                print("pose_x", pose_x)
                print("pose_y", pose_y)
                currentPath = currentPath + 1
                old_a = -4000
                print("This should be the final destination")
                driveMode = "at destination"
                
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
