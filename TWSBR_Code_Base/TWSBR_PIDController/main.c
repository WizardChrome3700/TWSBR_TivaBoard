#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "TWSBR_PIDController.h"

// Calculate control input using PID struct
float calculate_pid_control(PID_Controller* pid, float filteredPitch, float gyroRate)
{
    float setpoint = 0.0f;  // Maintain upright position
    float error = setpoint - filteredPitch;

    // Proportional term
    float proportional = pid->Kp * error;

    // Integral term with windup protection
    pid->integral += error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float integral = pid->Ki * pid->integral;

    // Derivative term (using direct gyro rate)
    float derivative = pid->Kd * (-gyroRate);

    // Combine all terms
    float output = proportional + integral + derivative;

    return output;
}
