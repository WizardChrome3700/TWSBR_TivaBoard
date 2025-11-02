/*
 * TWSBR_Wheel.h
 *
 *  Created on: Oct 19, 2025
 *      Author: akshat
 */

#ifndef TWSBR_WHEEL_H_
#define TWSBR_WHEEL_H_

#define PWM_FREQUENCY 2000

void wheels_init();
void wheel_setSpeed(float control_input, int8_t wheel_id);

#endif /* TWSBR_WHEEL_H_ */
