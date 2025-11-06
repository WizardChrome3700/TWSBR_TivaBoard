#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

// PID Controller Structure
typedef struct {
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    float integral;     // Integral accumulator
    float prev_error;   // Previous error (optional)
    float integral_limit; // Integral windup limit
} PID_Controller;

float calculate_pid_control(PID_Controller* pid, float filteredPitch, float gyroRate);

#endif
