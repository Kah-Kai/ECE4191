import RPi.GPIO as GPIO
import time

# Setup Pins
# Encoder a + b : L + R
eaR = 8
ebR = 10

eaL = 16
ebL = 18

# Enable + PWM :  Right Side
enaR = 11
enbR = 13

paR = 15
pbR = 37

# Enable + PWM :  Right Side
enaL = 32
enbL = 36

paL = 38
pbL = 40

# Setup the pin numbering mode
GPIO.setmode(GPIO.BCM)
# or GPIO.setmode(GPIO.BOARD)


# Setup GPIO pins as Input
#   Encoder A + B Right Side
GPIO.setup(eaR, GPIO.IN)
GPIO.setup(ebR, GPIO.IN)
#   Encodeer A + B Left Side
GPIO.setup(eaL, GPIO.IN)
GPIO.setup(ebL, GPIO.IN)

# Setup a GPIO pin as Output
#   Motor Control xxxx Side
#       Enable
GPIO.setup(enaR, GPIO.OUT)
GPIO.setup(enbR, GPIO.OUT)
#       PWM
GPIO.setup(paR, GPIO.OUT)
GPIO.setup(pbR, GPIO.OUT)

#   Motor Control xxxx Side
#       Enable
GPIO.setup(32, GPIO.OUT)
GPIO.setup(36, GPIO.OUT)
#       PWM
GPIO.setup(38, GPIO.OUT)
GPIO.setup(40, GPIO.OUT)

# PWM Initialization
pwm_paR = GPIO.PWM(paR, 1000)  # 1 kHz PWM frequency
pwm_pbR = GPIO.PWM(pbR, 1000)
pwm_paL = GPIO.PWM(paL, 1000)
pwm_pbL = GPIO.PWM(pbL, 1000)

# Start PWM with 0 duty cycle (off)
pwm_paR.start(0)
pwm_pbR.start(0)
pwm_paL.start(0)
pwm_pbL.start(0)


# Count Encoder Pulses 
try:
    while True:
        # Rotate Motor A clockwise
        GPIO.output(paL, GPIO.HIGH)
        GPIO.output(pbL, GPIO.LOW)
        time.sleep(2)

        # Stop Motor A
        GPIO.output(paL, GPIO.LOW)
        GPIO.output(pbL, GPIO.LOW)
        time.sleep(0.5)

        # Rotate Motor B clockwise
        GPIO.output(paR, GPIO.HIGH)
        GPIO.output(pbR, GPIO.LOW)
        time.sleep(2)

        # Stop Motor B
        GPIO.output(paR, GPIO.LOW)
        GPIO.output(pbR, GPIO.LOW)
        time.sleep(0.5)

        # Rotate Motor A counter-clockwise
        GPIO.output(paL, GPIO.LOW)
        GPIO.output(pbL, GPIO.HIGH)
        time.sleep(2)

        # Stop Motor A
        GPIO.output(paL, GPIO.LOW)
        GPIO.output(pbL, GPIO.LOW)
        time.sleep(0.5)

        # Rotate Motor B counter-clockwise
        GPIO.output(paR, GPIO.LOW)
        GPIO.output(pbR, GPIO.HIGH)
        time.sleep(2)

        # Stop Motor B
        GPIO.output(paR, GPIO.LOW)
        GPIO.output(pbR, GPIO.LOW)

        time.sleep(0.5)

except KeyboardInterrupt:
    # Clean up GPIO settings on Ctrl+C
    pass


finally:
    # Stop PWM and clean up GPIO
    pwm_paR.stop()
    pwm_pbR.stop()
    pwm_paL.stop()
    pwm_pbL.stop()
    GPIO.cleanup()
