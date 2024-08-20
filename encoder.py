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

try:
    while True:
        # Variables to store the count of high transitions
        left_count = 0
        right_count = 0
        
        # Initial states of the encoders
        prev_enLA = GPIO.input(enLA_pin)
        prev_enLB = GPIO.input(enLB_pin)
        prev_enRA = GPIO.input(enRA_pin)
        prev_enRB = GPIO.input(enRB_pin)
        
        # Timer for 10ms
        start_time = time.time()
        
        while (time.time() - start_time) < 0.01:  # 10ms loop
            # Check left encoder transitions
            enLA = GPIO.input(enLA_pin)
            enLB = GPIO.input(enLB_pin)
            
            if enLA != prev_enLA and enLA == GPIO.HIGH:
                left_count += 1
            prev_enLA = enLA
            
            if enLB != prev_enLB and enLB == GPIO.HIGH:
                left_count += 1
            prev_enLB = enLB
            
            # Check right encoder transitions
            enRA = GPIO.input(enRA_pin)
            enRB = GPIO.input(enRB_pin)
            
            if enRA != prev_enRA and enRA == GPIO.HIGH:
                right_count += 1
            prev_enRA = enRA
            
            if enRB != prev_enRB and enRB == GPIO.HIGH:
                right_count += 1
            prev_enRB = enRB
            
            time.sleep(0.0001)  # Short delay to avoid excessive CPU usage
        
        # Determine left wheel direction
        left_speed = left_count
        if prev_enLA == GPIO.HIGH and prev_enLB == GPIO.LOW:
            left_direction = "Forward"
        else:
            left_direction = "Backward"
        
        # Determine right wheel direction
        right_speed = right_count
        if prev_enRA == GPIO.HIGH and prev_enRB == GPIO.LOW:
            right_direction = "Forward"
        else:
            right_direction = "Backward"
        
        # Print out the speed and direction
        print(f"Left Wheel: Speed = {left_speed}, Direction = {left_direction}")
        print(f"Right Wheel: Speed = {right_speed}, Direction = {right_direction}")
        print("")

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
