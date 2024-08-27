import RPi.GPIO as GPIO
import time

# Define encoder pins
enLA_pin = 36
enLB_pin = 38
enRA_pin = 35
enRB_pin = 37

# Define motor control pins
in1_pin = 12
in2_pin = 13
in3_pin = 18
in4_pin = 19

# Set the GPIO mode to use physical pin numbers
GPIO.setmode(GPIO.BOARD)

# Set pins as input for encoders
#GPIO.setup(enLA_pin, GPIO.IN)
#GPIO.setup(enLB_pin, GPIO.IN)
#GPIO.setup(enRA_pin, GPIO.IN)
#GPIO.setup(enRB_pin, GPIO.IN)
GPIO.setup(enLA_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(enLB_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(enRA_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(enRB_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


# Set pins as output for motor control
GPIO.setup(in1_pin, GPIO.OUT)
GPIO.setup(in2_pin, GPIO.OUT)
GPIO.setup(in3_pin, GPIO.OUT)
GPIO.setup(in4_pin, GPIO.OUT)

# Set up PWM for motor control
frequency = 15000
pwm_IN1 = GPIO.PWM(in1_pin, frequency)
pwm_IN2 = GPIO.PWM(in2_pin, frequency)
pwm_IN3 = GPIO.PWM(in3_pin, frequency)
pwm_IN4 = GPIO.PWM(in4_pin, frequency)

# Start PWM with a duty cycle of 0% (off) for all pins
pwm_IN1.start(0)
pwm_IN2.start(0)
pwm_IN3.start(0)
pwm_IN4.start(0)

# Initialise global variables
LFscale = 1  # Default scale is 1 (no scaling)
RFscale = 1  # Default scale is 1 (no scaling)
RBscale = 1 # Default reverse scale is 1 (no scaling)
RBscale = 1 # Default reverse scale is 1 (no scaling)

LA = 0
LB = 0
RA = 0
RB = 0

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

# input [leftmotor, right motor, enable] between -1,1
def motorControl(motorInput):
    global LFscale, LBscale, RBscale, RFscale
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
        
# Motor calibration function
# Returns: left motor duty cycle scaling factor, right motor duty cycle scaling factor 
def motor_calibration(calibration_interval):
    global LA, LB, RA, RB, LFscale, RFscale, LBscale, RBscale  # Declare global variables
    # Reset counters
    LA = 0  
    LB = 0
    RA = 0
    RB = 0
    RF = 0 # right forward count
    RB = 0 # right backwards count
    LF = 0 # left forward count
    LB = 0 # left backwards count

    ### activate motor forward ###
    motorControl([1,1])
    time.sleep(2) # let motor get to full speed
    GPIO.add_event_detect(enLA_pin, GPIO.RISING, callback=encoderLA_callback) # Add interrupt event listeners
    GPIO.add_event_detect(enLB_pin, GPIO.RISING, callback=encoderLB_callback)
    GPIO.add_event_detect(enRA_pin, GPIO.RISING, callback=encoderRA_callback)
    GPIO.add_event_detect(enRB_pin, GPIO.RISING, callback=encoderRB_callback)
    time.sleep(calibration_interval)  # Wait and count
    GPIO.remove_event_detect(enLA_pin) # Disable further interrupts 
    GPIO.remove_event_detect(enLB_pin)
    GPIO.remove_event_detect(enRA_pin)
    GPIO.remove_event_detect(enRB_pin)
    motorControl([0,0])
    # forward calibration logic calibration logic
    LF = LA + LB
    RF = RA + RB
    forwardMin = min(LF, RF)
    LFscale = forwardMin/LF
    RFscale = forwardMin/RF
    # backwards calibration
    LA = 0  
    LB = 0
    RA = 0
    RB = 0
    ### activate motor backwards ###
    motorControl([-1,-1])
    time.sleep(2) # let motor get to full speed
    GPIO.add_event_detect(enLA_pin, GPIO.RISING, callback=encoderLA_callback) # Add interrupt event listeners
    GPIO.add_event_detect(enLB_pin, GPIO.RISING, callback=encoderLB_callback)
    GPIO.add_event_detect(enRA_pin, GPIO.RISING, callback=encoderRA_callback)
    GPIO.add_event_detect(enRB_pin, GPIO.RISING, callback=encoderRB_callback)
    time.sleep(calibration_interval)  # Wait and count
    GPIO.remove_event_detect(enLA_pin) # Disable further interrupts
    GPIO.remove_event_detect(enLB_pin)
    GPIO.remove_event_detect(enRA_pin)
    GPIO.remove_event_detect(enRB_pin)
    motorControl([0,0])
    # backward calibration logic calibration logic
    backwardMin = min(LA+LB,RA+RB) # minimum backwards encoder distance
    FBscale = backwardMin/forwardMin # forward-backward scale
    LBscale = LFscale * FBscale
    RBscale = RFscale * FBscale
    

try:
    # Main loop or additional code can go here
    motor_calibration(10)
    while True:
        time.sleep(1)  # Just keep the program running

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
