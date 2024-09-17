#include "Arduino.h"
#include "pidClass.h"

pidClass::pidClass(int in1_pin, int in2_pin, int encoder_pin, float encoder_ticks_per_rev){
  Serial.begin(115200);
  // pin setup
  pinMode(encoder_pin, INPUT); // reads encoder and triggers interrupt -> estimate velocity
  //attachInterrupt(digitalPinToInterrupt(encoder_pin),encoder_handler,RISING);
  // the ISR will be implemented externally
  pinMode(in1_pin, OUTPUT); // pwm the motor signal pins
  pinMode(in2_pin, OUTPUT);

  // initalise class variables
  _current_time = micros();
  _previous_time = _current_time;
  _current_velocity = 0;
  _previous_velocity = 0;
  _current_velocity_filtered = 0;
  _previous_velocity_filtered = 0;
  _position_encoder = 0;
  _position_temp = 0;
  _pulseToRPM = 60/encoder_ticks_per_rev; // eg 1200 ticks.per.second * (60 sec.per.min / 900 ticks.per.rev) = 80 rev.per.min 
  _direction = 1;
  _in1_pin = in1_pin;
  _in2_pin = in2_pin;
  _encoder_pin = encoder_pin;
  _kp = 20;
  _ki = 10;
  _integral = 0;
}

void pidClass::encoder_ISR(){
  _position_encoder += _direction;
}

void pidClass::control(float u){
  _u = u;
  if(u != max(min(u, -100), 100)){
    Serial.println("u is not in range");
    u = max(min(u, -100), 100);
  }

  // set direction
  if(u >= 0){
    _direction = 1;
  } else{
    _direction = -1;
  }

  // motor control
  if(_direction == 1){ //forward
    analogWrite(_in1_pin, u*2.55);  // Set motor speed (0-255)
    analogWrite(_in2_pin, 0);

  } else { // backwards
    analogWrite(_in1_pin, 0);
    analogWrite(_in2_pin, abs(u)*2.55);
  }
}

float pidClass::return_velocity(){
  return _current_velocity_filtered;
}

void pidClass:calculate_velocity(){
  // one iteration of velocity measurement, must repeatedly call function every 1ms 
  // calculate raw velocity
  _current_time = micros();
  _dt = (_current_time - _previous_time)/1.0e6;
  _previous_time = _current_time;
  _current_velocity = ((_position_encoder - _position_temp)/dt) * _pulseToRPM; 
  _position_temp = _position_encoder;

  // low pass filter (assumes 1khz sampling frequency)[https://www.youtube.com/watch?v=HRaZLCBFVDE&t=298s]
  _current_velocity_filtered = 0.854*_previous_velocity_filtered + 0.0728*(_current_velocity + _previous_velocity);
  _previous_velocity = _current_velocity;  
}

void pidClass::PID(float setpoint){
  calculate_velocity(); // undecided whether i should run this here
  float error = setpoint - _current_velocity;  // calculate error
  _integral += error*_dt;
  _integral = min(max(_integral, 100/_ki), -100/_ki) // integral windup measures
  _u = min(max( _kp * error + _ki*_integral , 100), -100);
  control(_u);  // update motor control
}