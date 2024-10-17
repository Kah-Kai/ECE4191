import numpy as np
import cv2
import time
import math
import subprocess
from gpiozero import PWMOutputDevice, DigitalInputDevice

class PIDClass:
    def __init__(self, in1_pin, in2_pin, encoder_pin, encoder_ticks_per_rev):
        # Pin setup
        self._in1_pin = PWMOutputDevice(in1_pin)
        self._in2_pin = PWMOutputDevice(in2_pin)
        self._encoder_pin = DigitalInputDevice(encoder_pin, pull_up=True)
        self._encoder_pin.when_activated = self.encoder_ISR

        self._current_time = time.time()
        self._previous_time = self._current_time
        self._dt = 0
        self._current_velocity = 0.0
        self._previous_velocity = 0.0
        self._current_velocity_filtered = 0.0
        self._previous_velocity_filtered = 0.0
        self._position_encoder = 0
        self._position_temp = 0
        self._pulseToRPM = 60.0 / encoder_ticks_per_rev
        self._direction = 1
        self._kp = 2.4#2.8
        self._ki = 0.5#1.2
        self._integral = 0.0

    def encoder_ISR(self):
        self._position_encoder += self._direction

    def control(self, u):
        self._u = u
        self._u = max(-100.0, min(100.0, self._u))

        if self._u >= 0:
            self._direction = 1
            self._in1_pin.value = self._u / 100
            self._in2_pin.value = 0
        else:
            self._direction = -1
            self._in1_pin.value = 0
            self._in2_pin.value = abs(self._u) / 100

    def return_velocity(self):
        return self._current_velocity_filtered

    def calculate_velocity(self):
        if time.time() - self._current_time >= 1/1000:
            self._current_time = time.time()
            self._dt = self._current_time - self._previous_time
            self._previous_time = self._current_time

            self._current_velocity = (self._position_encoder - self._position_temp) / self._dt
            self._current_velocity = self._current_velocity * self._pulseToRPM
            self._position_temp = self._position_encoder
            # difference equation fil[t] = 0.83fil[t-1] + 0.086raw[t] + 0.086raw[t-1]
            self._current_velocity_filtered = 0.8277396 * self._previous_velocity_filtered + 0.0861302 * (self._current_velocity + self._previous_velocity)
            self._previous_velocity = self._current_velocity
            self._previous_velocity_filtered = self._current_velocity_filtered

    def pi(self, setpoint):
        error = setpoint - self._current_velocity_filtered
        self._integral += error * self._dt
        self._integral = max(-100.0 / self._ki, min(100.0 / self._ki, self._integral))
        self._u = self._kp * error + self._ki * self._integral
        self.control(self._u)
        
    def close(self):
        self._in1_pin.close()
        self._in2_pin.close()
        self._encoder_pin.close()
        
    def reset_integral(self):
        self._integral = 0
        
class WHEELS:
    def __init__(self, left_pins, right_pins):
        self.encoder_ticks_per_rev = 12 * 74.83
        self.left_motor = PIDClass(left_pins[0], left_pins[1], left_pins[2], self.encoder_ticks_per_rev)
        self.right_motor = PIDClass(right_pins[0], right_pins[1], right_pins[2], self.encoder_ticks_per_rev)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        self.last_time = time.time_ns()/1000000
        self.trk_w = 0.223
        self.whl_r = 0.027
        self.ticks_per_meter = self.encoder_ticks_per_rev/(2*math.pi*self.whl_r)

    def odometry_step(self):
        delta_left_encoder = self.left_motor._position_encoder - self.last_encoder_left
        delta_right_encoder = self.right_motor._position_encoder - self.last_encoder_right
        self.last_encoder_left = self.left_motor._position_encoder
        self.last_encoder_right = self.right_motor._position_encoder

        delta_left = delta_left_encoder/self.ticks_per_meter
        delta_right = delta_right_encoder/self.ticks_per_meter
        translation = (delta_left + delta_right) / 2
        
        dx = translation * math.cos(self.theta)
        dy = translation * math.sin(self.theta)
        dtheta = (delta_right - delta_left) / self.trk_w
        
        self.x += dx
        self.y += dy
        self.theta += dtheta

    def get_pose(self):
        return self.x, self.y, self.theta

    def forward(self, speed):
        self.left_motor.pi(speed)
        self.right_motor.pi(speed)
        
    def calculate_velocity(self): # keep running to keep up to date velocity measurements
        self.left_motor.calculate_velocity()
        self.right_motor.calculate_velocity()

    def turn(self, speed):
        self.left_motor.pi(-speed)
        self.right_motor.pi(speed)

    def stop(self):
        self.left_motor.control(0)
        self.right_motor.control(0)
    
    def close(self):
        self.left_motor.close()
        self.right_motor.close()
        
class CONVEYOR:
    def __init__(self, motor_pins):
        self.encoder_ticks_per_rev = 12 * 74.83
        self.motor = PIDClass(motor_pins[0], motor_pins[1], motor_pins[2], self.encoder_ticks_per_rev)
        self.stored_balls = 0