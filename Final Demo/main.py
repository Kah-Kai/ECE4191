import numpy as np
import cv2
import time
import math
import subprocess
from gpiozero import PWMOutputDevice, DigitalInputDevice
from classes import PIDClass, WHEELS, CONVEYOR

################################################################
################### MOVEMENT AND MOTORS ########################
################################################################
VERY_LOW_SPEED = 5 # use for fine adjustments
LOW_SPEED = 15 # use mainly for turning
MED_SPEED = 40 # use for more accurate odometry
HIGH_SPEED = 50 # only use in addition to camera information
MAX_SPEED = 60 # should never use
        
def move_dst_th_speed(movement, gd, gt, speed, W):
    # DO NOT CHANGE :save inital position 
    if(speed*gd<0 or speed*gt<0):
        raise Exception("Impossible movement command") 
    ix, iy, it = W.get_pose()
    gd*=1.02 # increase goal distance to account for slippage and encoder error
    gt*=1.05
    gx = gd*math.cos(it)# compute goal x and y positions from current theta
    gy = gd*math.sin(it)
    W.right_motor.reset_integral()
    W.left_motor.reset_integral()
    return gd, gt, gx, gy, ix, iy, it, movement, speed

def break_movement(movement, it, gt, ix, gx, iy, gy, speed, W):
    #compute odometry
    W.odometry_step()
    x, y, t = W.get_pose() # current odometry
    leftv = W.left_motor.return_velocity()
    rightv = W.right_motor.return_velocity()
    print(f"Pose: x={x:.3f}, y={y:.3f}, theta={t:.3f}, Lvel={leftv:.3f}, Rvel={rightv:.3f}")  # Print to 3 decimal places
    # compute break condition
    if(movement == 0):
        W.calculate_velocity()
        W.forward(speed)
        if( abs(x - ix) >= abs(gx) and abs(y - iy) >= abs(gy)): 
            W.stop()
            return True
    elif(movement == 1):
        W.calculate_velocity()
        W.turn(speed)
        if(abs(t - it) >= abs(gt) ): 
            W.stop()
            return True
    
    return False

"""    sample pid

gd, gt, gx, gy, ix, iy, it, movement, speed = move_dst_th_speed(0, 0, 0, MED_SPEED, W)
exterior_break = False
while(True):
    if(break_movement(movement, it, gt, ix, gx, iy, gy, speed, W) or exterior_break): 
        W.stop()
        W.integral = 0
        break
    ### START CODE ### 
    
    ## END OF CODE ###
  
"""      

################################################################
######################### CAMERA ###############################
################################################################

CAMERA_HEIGHT = 360
CAMERA_WIDTH = 640
CAMERA_FPS = 60
CAMERA_BRIGHTNESS = 200
CAMERA_SHARPNESS = 255
CAMERA_CONTRAST = 255
CAMERA_SATURATION = 255
CAMERA_AUTO_EXPOSURE = 1 # manual = 1
CAMERA_EXPOSURE = 500 # [3,2047]
CAMERA_FOV_DIAG = 78
CAMERA_FOV = 78
CAMERA_RADIANS = np.radians(CAMERA_FOV)

TENNIS_BALL_RADIUS = 0.0335
MAX_NUM_OBJECTS = 8
MIN_OBJECT_AREA = 4
MAX_OBJECT_AREA = CAMERA_HEIGHT * CAMERA_WIDTH / 1.2 # when object is too close ?
CENTERED_TOLERANCE = 0.1
WEAKLY_CENTERED_TOLERANCE = 0.30
H_MIN = 30 # colour range for ball detection
S_MIN = 80
V_MIN = 80

H_MAX = 80
S_MAX = 255
V_MAX = 255

WHITE_LINE_WINDOW = 10
LOWER_WHITE = np.array([0, 0, 150])
UPPER_WHITE = np.array([255, 20, 255])
ROBOT_CAMERA = (CAMERA_WIDTH*(2/3), CAMERA_HEIGHT/2)

BOX_WINDOW = 1
BOX_CAMERA_TOP = (CAMERA_WIDTH*(0), CAMERA_HEIGHT/2)
BOX_CAMERA_BOTTOM = (CAMERA_WIDTH*(4/5), CAMERA_HEIGHT/2)
LOWER_BROWN_BOX = np.array([17,57,34]) # np.array([22, 65, 30])
UPPER_BROWN_BOX = np.array([50,86,60])

ERODE = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
DIALATE = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
EARLY_DISTANCE = 0.6
STORAGE_CAPACITY = 4

def camera_setup():
    """
    Ensures that opencv and camera works
    Returns capture object
    """
    try:
        print(f"OpenCV version: {cv2.__version__}")
    except Exception as e:
        print(f"OpenCV not working: {e}")
        return False

    # Iterate through camera indices and try to find a working one
    for camera_index in range(10):  # Check first 10 camera devices
        cap = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        ret_val , cap_for_exposure = cap.read() # reference capture 
        if cap.isOpened():
            # Set camera parameters # Define the commands to run
            commands = [
                ['v4l2-ctl', '-d', '/dev/video0', f'--set-ctrl=auto_exposure={CAMERA_AUTO_EXPOSURE}'],
                ['v4l2-ctl', '-d', '/dev/video0', f'--set-ctrl=exposure_time_absolute={CAMERA_EXPOSURE}'],
                ['v4l2-ctl', '-d', '/dev/video0', f'--set-ctrl=brightness={CAMERA_BRIGHTNESS}'],
                ['v4l2-ctl', '-d', '/dev/video0', f'--set-ctrl=sharpness={CAMERA_SHARPNESS}'],
                ['v4l2-ctl', '-d', '/dev/video0', f'--set-ctrl=contrast={CAMERA_CONTRAST}'],
                ['v4l2-ctl', '-d', '/dev/video0', f'--set-ctrl=saturation={CAMERA_SATURATION}']
                ]
            for command in commands:
                result = subprocess.run(command, capture_output=True, text=True)
                if result.returncode == 0:
                    print(f"Success: {' '.join(command)}")
                else:
                    print(f"Error: {' '.join(command)}\n{result.stderr}")
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
            print("Camera found", camera_index)
            return cap, time.time()

    raise Exception("No camera found")

def morph_ops(threshold):
    """
    Perform morphological operations to reduce noise and enhance object detection.
    Erosion reduces the white noise, and dilation helps recover the shape of the objects.
    """
    threshold = cv2.erode(threshold, ERODE, iterations=2)
    threshold = cv2.dilate(threshold, DIALATE, iterations=2)
    return threshold

def locate_ball(threshold):
    '''
    Calculates the radius and centre of all balls with good contour definitions
    then returns the information of the largest ball detected
    Returns none if fails
    '''
    temp = threshold.copy()
    contours, _ = cv2.findContours(temp, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        num_objects = len(contours)
        if num_objects < MAX_NUM_OBJECTS:
            # Store Radii of Moments
            max_radius = 0
            max_centre = [None, None]
            for contour in contours:
                # Calculate moments for each contour
                moment = cv2.moments(contour)
                area = moment['m00']
                if MIN_OBJECT_AREA < area < MAX_OBJECT_AREA:
                    # Fit an ellipse to the contour if possible
                    if len(contour) >= 5:  # Need at least 5 points to fit an ellipse
                        centre, axes, _ = cv2.fitEllipse(contour)
                        radius = min(axes) / 2 # Use the major axis length as the radius
                        if radius > max_radius:
                            max_centre = centre
                            max_radius = radius
            if max_centre == [None, None]: return None
            dist = calc_dist_to_ball(max_radius)
            ball_centered = 0
            weakly_centered = 0
            if int(CAMERA_HEIGHT/2)*(1-CENTERED_TOLERANCE) < int(max_centre[1]) < int(CAMERA_HEIGHT/2)*(1+CENTERED_TOLERANCE): # within centre pixels
                ball_centered = 1
            if int(CAMERA_HEIGHT/2)*(1-WEAKLY_CENTERED_TOLERANCE) < int(max_centre[1]) < int(CAMERA_HEIGHT/2)*(1+WEAKLY_CENTERED_TOLERANCE): # within centre pixels
                weakly_centered = 1
            return max_centre[0], max_centre[1], ball_centered, dist, num_objects, weakly_centered
    return None

def calc_dist_to_ball(observed_radius_pixels):
    # Calculate the apparent angle of the ball
    apparent_angle = observed_radius_pixels*(CAMERA_RADIANS / CAMERA_WIDTH)
    # Calculate the distance to the ball using small angle approximation
    distance_to_ball = TENNIS_BALL_RADIUS / apparent_angle
    return distance_to_ball

def window_line_detection(image, start_point, end_point, lower_range, upper_range, window_size = 10, rectangle = False):
    '''
    image : HSV stream from camera to analyse
    start_point : pixel location of the robot
    end_point : pixel location of the ball 
    lower_range, upper_range : HSV range to detect a certain colour  .eg. np.array([0, 0, 150])
    window_size : the size of the window around single pixel to check
    
    returns,
        pixel location of the line from robot to ball
        false if there is no line
    '''
    # extract pixel locations of all pixels along line
    hsv_image_copy = image.copy()
    x1, y1 = end_point
    x2, y2 = start_point
    
    mask = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH), dtype=np.uint8)
    cv2.line(mask, (int(x1), int(y1)), (int(x2), int(y2)), color=255, thickness=1)
    y_coords, x_coords = np.where(mask > 0) # pixel coordinates along line
    
    # get average HSV value around pixel and check if it is within range
    for x, y in zip(x_coords, y_coords):
        # Extract the window around the pixel
        half_window = window_size // 2
        if(rectangle):
            window = image[max(0, y-half_window*2):min(image.shape[0], y+2*half_window+1), max(0, x-half_window):min(image.shape[1], x+half_window+1)]
        else:
            window = image[max(0, y-half_window):min(image.shape[0], y+half_window+1), max(0, x-half_window):min(image.shape[1], x+half_window+1)]
        #print(window)
        avg_hsv = np.mean(window, axis=(0, 1))
        #print(avg_hsv)
        cv2.circle(hsv_image_copy, (x,y), 1, (0, 255, 0), 1) # 
        # check if it is within range
        if( np.all(avg_hsv >= lower_range) and np.all(avg_hsv <= upper_range) ):
            cv2.imwrite('circle_output.jpg', cv2.circle(hsv_image_copy, (x,y), 20, (0, 255, 0), 2))
            return True
    cv2.imwrite('circle_output.jpg', hsv_image_copy)
    return False

################################################################
######################### NAVIGATION ###########################
################################################################
def wait_with_interrupts(end_time):
    start_time = time.time()
    while(time.time() - start_time < end_time):
        print("settling motors")
        continue

def camera_step(cap):
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    threshold = cv2.inRange(hsv, (H_MIN, S_MIN, V_MIN),(H_MAX, S_MAX, V_MAX))
    threshold = morph_ops(threshold)
    return hsv, threshold

def white_line_detection(ball_data, hsv):
    ball_x, ball_y, _, _, _, _ = ball_data
    return window_line_detection(hsv, ROBOT_CAMERA, (ball_x,ball_y), LOWER_WHITE, UPPER_WHITE, WHITE_LINE_WINDOW)

def box_detection(hsv):
    return window_line_detection(hsv, BOX_CAMERA_TOP,BOX_CAMERA_BOTTOM, LOWER_BROWN_BOX, UPPER_BROWN_BOX, BOX_WINDOW, True)

def camera_hair(cap):
    if not cap.isOpened():
        print("Error: Could not open video stream")
        exit()

    while(True):
        _, frame = cap.read()
        cv2.line(frame, (0, int(CAMERA_HEIGHT/2)), (CAMERA_WIDTH, int(CAMERA_HEIGHT/2)), (0,0,0), 1)
        cv2.imwrite('crosshair.jpg', frame)
        time.sleep(1/CAMERA_FPS)
        user = int(input("1 to retake, 0 to continue"))
        if(user == 1):
            continue
        else:
            break


try:
    # W = WHEELS([17,18, 27], [26, 20,21]) # left motor pins, riht motor pins
    W = WHEELS([18,17, 27], [20, 26,21]) # left motor pins, riht motor pins
    C = CONVEYOR([11,8,7]) # BCM Pins not physical board
    # Camera startup
    cap, last_frame_time = camera_setup()
    camera_hair(cap)
    
    state = "exploring for ball"
    while(True):
        if(state == "exploring for ball"):
            closest_ball_short_term = 69696969
            closest_ball_long_term = 98723984798798178
            _,_,last_theta = W.get_pose()
            explore_dir = 1
            gd, gt, gx, gy, ix, iy, it, movement, speed = move_dst_th_speed(1, 0, 69696969*explore_dir, LOW_SPEED*explore_dir, W)
            exterior_break = False
            while(True):
                if(break_movement(movement, it, gt, ix, gx, iy, gy, speed, W) or exterior_break): 
                    print("Exploration success")
                    state = "allign with centre of ball"
                    W.stop()
                    wait_with_interrupts(3)
                    break

                ### START CODE ### 
                """ UPDATE CAMERA DATA  """
                hsv, threshold = camera_step(cap)

                ball_data = locate_ball(threshold)

                if ball_data == None:
                    print("no balls detected")
                    state = "exploring for ball"
                    continue

                """ WHITE LINE DETECTION """
                if( white_line_detection(ball_data, hsv) == True):
                    print("ball out of bounds")
                    state = "exploring for ball"
                    continue
                
                #closest ball within current rotation
                ball_x, ball_y, ball_centered, ball_dist, num_object, weakly_centered = ball_data
                if(ball_dist < closest_ball_short_term):
                    print("short term value ", ball_dist)
                    closest_ball_short_term = ball_dist

                #closest ball within last rotation
                _,_,t = W.get_pose() 
                if(abs(t - last_theta) > 2*math.pi):
                    print("long term value ", closest_ball_short_term)
                    closest_ball_long_term = closest_ball_short_term
                    last_theta = t
                    closest_ball_short_term = 69696969696

                # see closest ball again within 15 % error
                exterior_break = (abs(ball_dist - closest_ball_long_term)/closest_ball_long_term < 0.05)
                ## END OF CODE ###
                
        elif(state == "allign with centre of ball"):
            ball_x, ball_y, ball_centered, ball_dist, num_objects, weakly_centered = ball_data
            turn_dir = -1 if (ball_y < CAMERA_HEIGHT/2) else 1
            gd, gt, gx, gy, ix, iy, it, movement, speed = move_dst_th_speed(1, 0, turn_dir*2*math.pi, turn_dir*LOW_SPEED, W)
            exterior_break = ball_centered
            while(True):
                if(break_movement(movement, it, gt, ix, gx, iy, gy, speed, W) or ball_centered): 
                    print("Ball now centered")
                    W.stop()
                    state = "approach closest ball"
                    wait_with_interrupts(3)
                    break
                ### START CODE ### 
                """" UPDATE CAMERA DATA  """
                hsv, threshold = camera_step(cap)

                ball_data = locate_ball(threshold)
                if ball_data == None:
                    print("no balls detected")
                    state = "exploring for ball"
                    break

                """ WHITE LINE DETECTION """
                if( white_line_detection(ball_data, hsv) == True):
                    print("ball out of bounds")
                    state = "exploring for ball"
                    break

                ball_x, ball_y, ball_centered, ball_dist, num_objects, weakly_centered = ball_data
                ## END OF CODE ###

        elif(state == "approach closest ball"):
            gd, gt, gx, gy, ix, iy, it, movement, speed = move_dst_th_speed(0, ball_dist, 0, MED_SPEED, W)
            exterior_break = False
            while(True):
                if(break_movement(movement, it, gt, ix, gx, iy, gy, speed, W) or exterior_break):
                    # break condition 1 : success
                    if(ball_dist < EARLY_DISTANCE and ball_centered):
                        print("Ball centred and ready to store")
                        #wait_with_interrupts(1)
                        state = "funnel ball"
                        W.stop()
                        break

                    elif( ball_dist < EARLY_DISTANCE and not ball_centered ): 
                        print("Current at ball, but not centred")
                        W.stop()
                        wait_with_interrupts(1)
                        state = "allign with centre of ball"
                        break
                
                ### START CODE ### 
                """ UPDATE CAMERA DATA  """
                hsv, threshold = camera_step(cap)

                ball_data = locate_ball(threshold)
                if ball_data == None:
                    print("no balls detected")
                    state = "exploring for ball"
                    break

                """ WHITE LINE DETECTION """
                if( white_line_detection(ball_data, hsv) == True):
                    print("ball out of bounds")
                    state = "exploring for ball"
                    break

                ball_x, ball_y, ball_centered, ball_dist, num_objects, weakly_centered = ball_data
                
                # deviation from centre 
                print("Approaching ball distance, ",ball_dist)
                exterior_break = not weakly_centered or (ball_dist < EARLY_DISTANCE)
                ### END OF CODE ###

        elif(state == "funnel ball"):
                temp_dist = ball_dist
                gd, gt, gx, gy, ix, iy, it, movement, speed = move_dst_th_speed(0, temp_dist + 0.05, 0, MED_SPEED, W)
                
                while(True):
                    if(break_movement(movement, it, gt, ix, gx, iy, gy, speed, W)):
                        W.stop()
                        state = "store ball"
                        break
                    
                    ### START CODE ### 
                    ### END OF CODE ###

        elif(state == "store ball"):
            C.motor.control(100)
            time.sleep(1.25)
            C.motor.control(0)
            C.stored_balls += 1
            if C.stored_balls == STORAGE_CAPACITY:
                state = "deposite"
                continue
            else:
                state = "exploring for ball"
                continue



except KeyboardInterrupt:
    W.close()

finally:
    print("End of program")