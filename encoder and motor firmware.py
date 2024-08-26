import RPi.GPIO as GPIO
import time

# Define encoder pins
enLA_pin = 36
enLB_pin = 38
enRA_pin = 35
enRB_pin = 37

# Set the GPIO mode to use physical pin numbers
GPIO.setmode(GPIO.BOARD)

# Set pins as input for encoders
GPIO.setup(enLA_pin, GPIO.IN)
GPIO.setup(enLB_pin, GPIO.IN)
GPIO.setup(enRA_pin, GPIO.IN)
GPIO.setup(enRB_pin, GPIO.IN)

# Initialise global variables
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

def disable_interrupts_for_pin(pin):
    GPIO.remove_event_detect(pin)

# Motor calibration function
# returns :  left motor duty cycle scalling factor, right motor duty cycle scalling factor 
def motor_calibration(calibration_interval):
    global LA, LB, RA, RB  # Declare global variables

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
    disable_interrupts_for_pin(enLA_pin)
    disable_interrupts_for_pin(enLB_pin)
    disable_interrupts_for_pin(enRA_pin)
    disable_interrupts_for_pin(enRB_pin)
    
    # Calibration logic
    if (LA + LB) > (RA + RB):  # Left motor stronger, scale down left motor
        return (RA + RB) / (LA + LB), 1
    return 1, (LA + LB) / (RA + RB)

# Example usage
try:
    left_scale, right_scale = motor_calibration(5)  # Calibrate for 5 seconds
    print(f"Left Scale: {left_scale}, Right Scale: {right_scale}")

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
