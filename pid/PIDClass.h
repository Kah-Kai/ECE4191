
#ifndef PIDClass_h
#define PIDClass_h
#include "Arduino.h"

class PIDClass{
  public:
    PIDClass(int in1_pin, int in2_pin, int encoder_pin, float encoder_ticks_per_rev);
    void encoder_ISR(); // called on rising edge of encoder to update the position variable
    void calculate_velocity(); // calculate the difference equation to estimate velocity of encoder motor
    void control(float u); // -100 <= input <= 100 for the power of the motor
    void pi(float setpoint); // PI controller to match the motor power to a specified velocity
    float return_velocity(); // returns rpm of motor estimate
  private:
    int _in1_pin;
    int _in2_pin;
    int _encoder_pin;
    float _ki;
    float _kp;
    float _u;
    float _integral;
    unsigned long _current_time; // micro seconds
    unsigned long _previous_time;
    unsigned long _dt;
    float _current_velocity; // RPM 
    float _previous_velocity;
    float _current_velocity_filtered;
    float _previous_velocity_filtered;
    int _position_encoder;
    int _position_temp;
    float _pulseToRPM; 
    int _direction; // specify current direction required because only one encoder pin
};

#endif