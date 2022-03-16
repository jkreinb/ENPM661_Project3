## Project 3 - Implementing A* algorithm
#  
#

import time
import numpy as np
import heapq as hq
import cv2 as cv
import math

start_time = time.time()

c2c_node =  np.full((250,400),np.inf)

# Initializes a numpy matrix full of np.inf values to represent the workspace
c2c_node =  np.full((250,400),np.inf)

# Initializes image file full of pixels with black color (0,0,0) RGB scale
image = np.zeros((250,400,3),np.uint8)

# Creating objects using half planes and semi-algebraic definitions, all object
# spaces have their cost set to -1
for x in range(0,400,1):
    for y in range(0,250,1):
        if x < 5 or x > 395 or y < 5 or y > 245:
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= 95 and y <= 180 and (x>math.floor(105-((37/45)*(y-95)))) and x < math.floor(110-(.3125*(y-95))):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= 180 and y <= 185 and x>math.floor(105-((37/45)*(y-95))) and x < math.floor(85+(1*(y-180))):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= 185 and y <= 215 and x>math.ceil(31+(2.9666*(y-185))) and x < math.floor(85+(1*(y-180))):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= 59 and y <= 82 and x > math.ceil(200 -(1.732*(y-59))) and x < math.floor(200+(1.732*(y-59))):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= 82 and y <= 117 and x > 160 and x < 240:
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif y >= 117 and y <= 141 and x > math.ceil(160 +(1.732*(y-117))) and x < math.floor(240 -(1.732*(y-117))):
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]
        elif math.floor(((x-300)**2)+((y-185)**2)) < (45**2): 
            c2c_node[y][x] = -1
            image[y,x]= [255,0,0]

# Creates variables for setting goal/start locations
goal_set = False
goal_angle_set = False
start_set = False
start_angle_set = False

# Goal setting loop
while goal_set == False:
    print("\nPlease enter a goal location on the workspace\n")
    print("-------------------------------------------------------\n")
    goal_x = int(input("Enter X position of goal node, must be an integer between 0-400\n"))
    goal_y = int(input("Enter Y position of goal node, must be an integer between 0-250\n"))
    if goal_x < 0 or goal_x > 400 or goal_y > 250 or goal_y < 0: # Checks if goal is outside of workspace
        print("\nGoal outside of workspace! Please try again\n")
        time.sleep(2)
        continue
    elif c2c_node[goal_y][goal_x] == np.inf:
        goal_set = True
    else: # If goal is not equal to np.inf it's an obstacle, prompts user again for new goal
        print("\nGoal location is ontop of an obstacle! Please enter new goal location\n")
        time.sleep(2)
while goal_angle_set == False:
    goal_theta = int(input("Enter orientation of robot at goal node, must be in degrees between 0-360, in steps of 30 degrees\n"))
    if (goal_theta > 360) or (goal_theta < 0):
        print("\n Goal angle out of range, please normalize within 0-360 degrees\n")
        time.sleep(2)
        continue
    elif (goal_theta % 30 != 0):
        print("\n Goal Angle must be in steps of 30 degrees \n")
        time.sleep(2)
    else:
        goal_angle_set = True
        
# Start setting loop

while start_set == False:
    print("\nPlease enter a starting location on the workspace\n")
    print("-------------------------------------------------------\n")
    start_x = int(input("Enter X position of start node, must be an integer between 0-400\n"))
    start_y = int(input("Enter Y position of start node, must be an integer between 0-250\n"))
    if start_x < 0 or start_x > 400 or start_y > 250 or start_y < 0: # Checks if start is outside workspace
        print("\nStarting location is outside of workspace! Please try again\n")
        time.sleep(2)
        continue
    elif c2c_node[start_y][start_x] == np.inf:
        start_set = True
    else: # If start is not equal to np.inf it's an obstacle, prompts user again for new start
        print("\nStarting location is ontop of an obstacle! Please enter new starting location\n")
        time.sleep(2)
while start_angle_set == False:
    start_theta = int(input("Enter orientation of robot at start node, must be in degrees between 0-360, in steps of 30 degrees\n"))
    if (start_theta > 360) or (start_theta < 0):
        print("\n Start angle out of range, please normalize within 0-360 degrees \n")
        time.sleep(2)
        continue
    elif (start_theta % 30 != 0):
        print("\n Start angle must be in steps of 30 degrees! Please try again! \n")
        time.sleep(2)
        continue
    else:
        start_angle_set = True

goal_node = (goal_y,goal_x,goal_theta)
start_node = (start_y,start_x,start_theta)

print(goal_node)
print(start_node)