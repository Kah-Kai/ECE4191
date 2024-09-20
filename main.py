import time
import numpy as np
import cv2

# Import classes
from detector import Detector



# TODO: Update values in the params folder
# Load parameters from callibration
wheel_radius = np.loadtxt("params/wheel_radius.txt")
wheel_sep = np.loadtxt("params/wheel_sep.txt")


capacity = 3   # Ball capacity of robot
num_balls = 0  # Number of balls collected

# Initialize classes
detector = Detector() # Initialise detector

def ball_search():
    pass

def align_with_ball():
    pass

def approach_ball():
    pass


# Infinite loop
# while True:
#     # While number of balls on robot < capacity:
#         # Detect balls
#         # Navigate to ball and collect
#     # Drive to box
#     # Deposit balls 
#     pass



