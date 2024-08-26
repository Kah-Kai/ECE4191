import RPi.GPIO as GPIO
import time

# Define encoder pins
enLA_pin = 36
enLB_pin = 38
enRA_pin = 35
enRB_pin = 37
calibration_pin = 39

# Set the GPIO mode to use physical pin numbers
GPIO.setmode(GPIO.BOARD)

# Set pins as input for encoders
GPIO.setup(enLA_pin, GPIO.IN)
GPIO.setup(enLB_pin, GPIO.IN)
GPIO.setup(enRA_pin, GPIO.IN)
GPIO.setup(enRB_pin, GPIO.IN)
GPIO.setup(calibration_pin, GPIO.IN)

# Initialise global variables
Lscale = 1  # Default scale is 1 (no scaling)
Rscale = 1  # Default scale is 1 (no scaling)
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

# Motor calibration function
# Returns: left motor duty cycle scaling factor, right motor duty cycle scaling factor 
def motor_calibration(calibration_interval):
    global LA, LB, RA, RB, Lscale, Rscale  # Declare global variables

    # Reset counters
    LA = 0  
    LB = 0
    RA = 0
    RB = 0  
    
    # Add interrupt event listeners
    GPIO.add_event_detect(enLA_pin, GPIO.RISING, callback=encoderLA_callback)
    GPIO.add_event_detect(enLB_pin, GPIO.RISING, callback=encoderLB_callback)
    GPIO.add_event_detect(enRA_pin, GPIO.RISING, callback=encoderRA_callback)
    GPIO.add_event_detect(enRB_pin, GPIO.RISING, callback=encoderRB_callback)
    
    time.sleep(calibration_interval)  # Wait and count
    
    # Disable further interrupts 
    GPIO.remove_event_detect(enLA_pin)
    GPIO.remove_event_detect(enLB_pin)
    GPIO.remove_event_detect(enRA_pin)
    GPIO.remove_event_detect(enRB_pin)
    
    # Calibration logic
    if (LA + LB) > (RA + RB):  # Left motor stronger, scale down left motor
        Lscale = (RA + RB) / (LA + LB)
        Rscale = 1
    else:
        Lscale = 1
        Rscale = (LA + LB) / (RA + RB)


# Add event detection for calibration
GPIO.add_event_detect(calibration_pin, GPIO.RISING, callback=motor_calibration(10))

try:
    # Main loop or additional code can go here
    while True:
        time.sleep(1)  # Just keep the program running

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
