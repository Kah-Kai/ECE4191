import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import time

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
### Global Variables
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
## Raspberry Pi
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Define encoder pins
#enLA_pin = 36
enLB_pin = 38
#enRA_pin = 35
enRB_pin = 37


# Define motor control pins
in1_pin = 12
in2_pin = 13
in3_pin = 18
in4_pin = 19

# Set the GPIO mode to use physical pin numbers
GPIO.setmode(GPIO.BOARD)

# Set pins as input for encoders
#GPIO.setup(enLA_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(enLB_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
#GPIO.setup(enRA_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(enRB_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Set pins as output for motor control
GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)
GPIO.setup(in3_pin, GPIO.OUT)
GPIO.setup(in4_pin, GPIO.OUT)

# Set up PWM for motor control
frequency = 5000
pwm_IN1 = GPIO.PWM(in1_pin, frequency)
pwm_IN2 = GPIO.PWM(in2_pin, frequency)
pwm_IN3 = GPIO.PWM(in3_pin, frequency)
pwm_IN4 = GPIO.PWM(in4_pin, frequency)

# Start PWM with a duty cycle of 0% (off) for all pins
pwm_IN1.start(0)
pwm_IN2.start(0)
pwm_IN3.start(0)
pwm_IN4.start(0)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
## Motor
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# Initialise global variables
LFscale = 1  # Default scale is 1 (no scaling)
RFscale = 1  # Default scale is 1 (no scaling)
RBscale = 1 # Default reverse scale is 1 (no scaling)
RBscale = 1 # Default reverse scale is 1 (no scaling)

gear = 74.83
enc_pulse = 24

LA = 0
LB = 0
RA = 0
RB = 0

def enableEncoder():
    #GPIO.add_event_detect(enLA_pin, GPIO.RISING, callback=encoderLA_callback) # Add interrupt event listeners
    GPIO.add_event_detect(enLB_pin, GPIO.RISING, callback=encoderLB_callback)
    #GPIO.add_event_detect(enRA_pin, GPIO.RISING, callback=encoderRA_callback)
    GPIO.add_event_detect(enRB_pin, GPIO.RISING, callback=encoderRB_callback)

def disableEncoder():
    #GPIO.remove_event_detect(enLA_pin) # Disable further interrupts
    GPIO.remove_event_detect(enLB_pin)
    #GPIO.remove_event_detect(enRA_pin)
    GPIO.remove_event_detect(enRB_pin)
    
def clearEncoder():
    global LA,LB,RA,RB
    LA = 0
    LB = 0
    RA = 0
    RB = 0

# returns left encoder count , right encoder count
def getEncoder():
    global LA,LB,RA,RB
    return LA+LB, RA+RB
    
def motorControl(motorInput, left_encoder, right_encoder):
    robot_goal = False
    global LFscale, LBscale, RBscale, RFscale, LA, LB, RA, RB
    clearEncoder()
    left_velocity = motorInput[0]
    right_velocity = motorInput[1]
    if -1 <= left_velocity <= 0:  # Backward
        pwm_IN1.ChangeDutyCycle(0)
        pwm_IN2.ChangeDutyCycle(abs(left_velocity) * 100 * LBscale)
    elif 0 < left_velocity <= 1:  # Forward
        pwm_IN1.ChangeDutyCycle(left_velocity * 100 * LFscale)
        pwm_IN2.ChangeDutyCycle(0)
    else:
        print("Invalid left wheel velocity. Please enter a value between -1 and 1.")
        return

    if -1 <= right_velocity <= 0:  # Backward
        pwm_IN3.ChangeDutyCycle(0)
        pwm_IN4.ChangeDutyCycle(abs(right_velocity) * 100 * RBscale)
    elif 0 < right_velocity <= 1:  # Forward
        pwm_IN3.ChangeDutyCycle(right_velocity * 100 * RFscale)
        pwm_IN4.ChangeDutyCycle(0)
    else:
        print("Invalid right wheel velocity. Please enter a value between -1 and 1.")
        return
    # if either wheel reaches encoder goal then stop
    left_count, right_count =  getEncoder()
    while(left_count < left_encoder or right_count < right_encoder):
        time.sleep(1/1000) # 1ms reduce cpu usage
        left_count, right_count = getEncoder()
    robot_goal = True
    return left_count, right_count

def drive_dist(dist, power=1):
    # drive robot straigt for certain distance
    encoder_goal = dist/(2*np.pi*whl_r) * gear * enc_pulse
    if dist < 0:
        # backwards
        motor_input = power * [-1, -1]
        motorControl(motor_input, encoder_goal, encoder_goal)
    elif dist > 0:
        # forwards
        motor_input = power * [1, 1]
        motorControl(motor_input, encoder_goal, encoder_goal)
    elif dist == 0:
        return


def turn_robot(angle):
    # turn robot a certain angle
    encoder_goal = angle/360 * (2*np.pi*trk_w/2)/(2*np.pi*whl_r) * gear * enc_pulse
    if angle < 0: # turn right
        motor_input = [1, -1]
        motorControl(motor_input, encoder_goal, encoder_goal)
    elif angle > 0: # turn left
        motor_input = [-1, 1]
        motorControl(motor_input, encoder_goal, encoder_goal)
    elif angle == 0:
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
## Robot
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
## Camera
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
MAX_NUM_OBJECTS = 5
MIN_OBJECT_AREA = 4
MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.2

fov_degrees = 59  # Horiz Field of view in degrees

# Trackbar names and initial values
H_MIN = 20
H_MAX = 100
S_MIN = 50
S_MAX = 255
V_MIN = 65
V_MAX = 255

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
## Policy and Planning
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
prox_ball_dist = 0.1 # desired proximity to the ball
actual_radius = 0.0335  # Actual radius of the tennis ball in M
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
## Global Flags
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
robot_home = 1
robot_goal = 0
ball_centered = 0

# Define interrupt handlers
def encoderLA_callback(channel):
    global LA
    LA += 1

def encoderLB_callback(channel):
    global LB
    LB += 1

def encoderRA_callback(channel):
    global RA
    RA += 0

def encoderRB_callback(channel):
    global RB
    RB += 1

# Motor calibration function
# Returns: left motor duty cycle scaling factor, right motor duty cycle scaling factor 
def motor_calibration(calibration_interval):
    global LA, LB, RA, RB, LFscale, RFscale, LBscale, RBscale  # Declare global variables
    # Reset counters
    LFscale = 1
    LBscale = 1
    RFscale = 1
    RBscale = 1
    clearEncoder()
    
    ### activate motor forward ###
    pwm_IN1.ChangeDutyCycle(100)
    pwm_IN2.ChangeDutyCycle(0)
    pwm_IN3.ChangeDutyCycle(100)
    pwm_IN4.ChangeDutyCycle(0)
    
    time.sleep(2) # let motor get to full speed
    enableEncoder()
    time.sleep(calibration_interval)  # Wait and count
    disableEncoder()
    
    pwm_IN1.ChangeDutyCycle(0)
    pwm_IN2.ChangeDutyCycle(0)
    pwm_IN3.ChangeDutyCycle(0)
    pwm_IN4.ChangeDutyCycle(0)
    
    # forward calibration logic calibration logic
    forwardMin = min(LA + LB, RA + RB)
    LFscale = forwardMin/(LA + LB)
    RFscale = forwardMin/(RA + RB)
    # backwards calibration
    clearEncoder()
    ### activate motor backwards ###
    pwm_IN1.ChangeDutyCycle(0)
    pwm_IN2.ChangeDutyCycle(100)
    pwm_IN3.ChangeDutyCycle(0)
    pwm_IN4.ChangeDutyCycle(100)
    time.sleep(2) # let motor get to full speed
    enableEncoder()
    time.sleep(calibration_interval)  # Wait and count
    disableEncoder()
    pwm_IN1.ChangeDutyCycle(0)
    pwm_IN2.ChangeDutyCycle(0)
    pwm_IN3.ChangeDutyCycle(0)
    pwm_IN4.ChangeDutyCycle(0)
    # backward calibration logic calibration logic
    backwardMin = min(LA+LB,RA+RB) # minimum backwards encoder distance
    FBscale = backwardMin/forwardMin # forward-backward scale
    LBscale = LFscale * FBscale
    RBscale = RFscale * FBscale
    print(LFscale, RFscale, LFscale, LBscale)

def wheel_calib():
    # Calibrate Radius of Robot Wheels
    global whl_r
    # Assuming both wheels are the same Radius

    # physical measurement of wheel radius
    whl_r = 0.03
  
    # while user defined calibration not complete
    while True:
        print("Is Robot at Starting Marker?")
        if ans == "y":
            # Drive predicted 1 meter
            drive_dist(1)
            if not robot_goal:
                print("Driving 1 meter")
            if robot_goal:
                print("Done")
                print("Did the robot Drive 1 meter?")
            ans = # user input
            if ans == "y":
                print("Calibration Complete, Exiting Wheel Calibration")
                break
            elif ans == "n":
                # User input Actual distance
                print("What was the Travelled Distance in Meters?")
                trvl_d = # user input
                # scale wheel radius
                whl_r = 1/trvl_d * whl_r


def track_calib():
    # Calibrate Width of Robot
    global trk_w
    # physical measurement of robot track width
    trk_w = 0.2

    # while user defined calibration not complete
    while True:
        print("Is Robot at Starting Marker?")
        if ans == "y":
            # Turn predicted 360
            turn_robot(360)
            if not robot_goal:
                print("Turning 360")
            if robot_goal:
                print("Done")
                print("Did the robot Turn 360?")
            ans = # user input
            if ans == "y":
                print("Calibration Complete, Exiting Trackwidth Calibration")
                break
            elif ans == "n":
                # User input Actual distance
                print("What was the Turned Angle in Degrees?")
                trnd_a = # user input
                # scale track width
                trk_w = 360/trnd_a * trk_w


# Ball Detection

def draw_objects(contours, frame):
    # Store Radii of Moments
    radii = []
    for contour in contours:
        # Calculate moments for each contour
        moment = cv2.moments(contour)
        area = moment['m00']

        if MIN_OBJECT_AREA < area < MAX_OBJECT_AREA:
            # Calculate the center of the object
            x = int(moment['m10'] / area)
            y = int(moment['m01'] / area)

            # Fit an ellipse to the contour if possible
            if len(contour) >= 5:  # Need at least 5 points to fit an ellipse
                ellipse = cv2.fitEllipse(contour)
                center, axes, angle = ellipse
                radius = min(axes) / 2 # Use the major axis length as the radius
                radii.append(radius) # Append to list to find max
                
                # Display Derived Distance to ball
                dist = calc_dist_to_ball(fov_degrees, FRAME_WIDTH, actual_radius, radius)

                # Flag if Largest Ball is Centered
                if radius == max(radii):
                    
                    if int(center[0]) >= 154 and int(center[0]) <= 164:
                        ball_centered = 1
                    else:
                        ball_centered = 0
                    
                    return center[0], center[1], ball_centered, dist

def morph_ops(thresh):
    erode_element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
    thresh = cv2.erode(thresh, erode_element, iterations=2)
    thresh = cv2.dilate(thresh, dilate_element, iterations=2)
    return thresh

def track_filtered_objects(threshold, frame):
    temp = threshold.copy()
    contours, _ = cv2.findContours(temp, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        num_objects = len(contours)
        if num_objects < MAX_NUM_OBJECTS:
            lbcx, lbcy, lbc, lbd = draw_objects(contours, frame)
            return lbcx, lbcy, lbc, lbd, num_objects
        else:
            cv2.putText(frame, "TOO MUCH NOISE! ADJUST FILTER", (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

def calc_dist_to_ball(fov_degrees, resolution_width, actual_radius, observed_radius_pixels):
    # Convert fov from degrees to radians
    fov_radians = np.radians(fov_degrees)
    
    # Calculate the angular resolution per pixel
    angular_resolution_per_pixel = fov_radians / resolution_width
    
    # Calculate the apparent angle of the ball
    apparent_angle = observed_radius_pixels * angular_resolution_per_pixel
    
    # Calculate the distance to the ball using small angle approximation
    distance_to_ball = actual_radius / apparent_angle
    
    return distance_to_ball

def cam_cap():
    ret, frame = capture.read()
    if not ret:
        print("Error: Empty frame captured")
        return

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    threshold = cv2.inRange(hsv, (H_MIN, S_MIN, V_MIN),(H_MAX, S_MAX, V_MAX))

    threshold = morph_ops(threshold)

    return track_filtered_objects(threshold, frame)

def ball_search():
    # Wide Ball Search
    while True:
        print("wide Ball Search")
        # turn 30 clockwise
        turn_robot(-30)
        if not robot_goal:
            print("Turning 30* Clockwise")
        # stop
        if robot_goal:
            print("Done")

        # initialise ball location, centered flag and n_objects
        ball_data = []
        # start capture time
        t_s = time.time()

        while True:
            # capture
            ball_data.append(cam_cap())
            # store ball detection
            if time.time() - t_s >= 0.5:
                break
            
        # average ball detection
        av_ball_data = np.mean(ball_data)
        if av_ball_data[4] >= 0:
            break

def align_w_ball():
    # Narrow Ball Search / Ball Alignment
        while True:
            print("Aligning Robot with Ball")
            # initialise ball location, centered flag and n_objects
            ball_data = []
            # start capture time
            t_s = time.time()

            while True:
                # capture
                ball_data.append(cam_cap())
                # store ball detection
                if time.time() - t_s >= 0.5:
                    break
            # average ball detection
            av_ball_data = np.mean(ball_data)

            dist_2_cent = int(FRAME_WIDTH/2) - av_ball_data[0]
            f = FRAME_WIDTH/(2*np.tan(fov_degrees/2))
            theta = 2 * np.atan(dist_2_cent/2*f)

            # Drive robot theta degrees
            turn_robot(theta)

            if av_ball_data[2] >= 0.5:
                return av_ball_data[3]
                break

def approach_ball(dist_2_ball):
    # Drive Towards Ball and leave 10cm gap
    while True:
        # initialise ball location, centered flag and n_objects
        ball_data = []

        # capture
        lbcx, lbcy, lbc, lbd, n = cam_cap()

        # Drive towards ball for lbd
        drive_dist(lbd-prox_ball_dist, power)

        if lbd < 0.5:
            # reduce motor speed by half
            power = 0.5

        # if balls on screen and distance less than prox
        if n > 0 and lbd <= prox_ball_dist:
            break
        # if balls not on screen and travel distance > distance to ball - proximity
        if n < 1 & robot_goal:
            break
        
    left, right = getEncoder()
    mean = left + right / 2
    trav_d = (2 * np.pi * whl_r * mean)/(gear * enc_pulse)

    return trav_d
    
def go_home(dist):
    # Drive Back Home

    # move backward 10cm
    drive_dist(-0.1)
    # Turn 180
    turn_robot(180)
    # Drive forward trav_dist - 10cm
    drive_dist(dist - 0.1)

def main():
    global capture
    print("Welcome to Tennisbot")

    print("Initialising Camera")
    capture = cv2.VideoCapture(0)

    if capture.isOpened:
        print("Camera Capture Acquired")
    elif not capture.isOpened():
        print("Error: Could not open video capture")
        return
    
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # Calibrate Wheel Velocities
    motor_calibration(10)
    # Calibrate Distance/ Wheel Radius
    wheel_calib()
    # Calibrate Angle
    track_calib()

        
    ball_search()
    # original distance from ball to start point
    dist_2_ball = align_w_ball()
    trav_dist = approach_ball(dist_2_ball)
    go_home(trav_dist)

    if robot_goal:
        print("Milestone 1 Complete")

    if cv2.waitKey(30) & 0xFF == ord('q'):
        return
    
except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
