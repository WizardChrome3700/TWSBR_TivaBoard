#ifndef TWSBR_MPU9250_H
#define TWSBR_MPU9250_H

#include <stdbool.h>
#include <stdint.h>

// I2C register base addresses for MPU9250 data reads
#define ACCEL_ADDR 0x3B
#define GYRO_ADDR  0x43
#define devAddr    0x68

// Public functions
void mpu9250_init(void);
bool mpu9250_calculate_bias(void);         // accel bias (call when level & stationary)
bool mpu9250_calibrate_gyro_bias(void);    // gyro bias (call when stationary)
void mpu9250_readAcceleration(uint8_t* accelArray, float* accelXYZ); // returns accel in g
void mpu9250_get_gyro(float* gyroXYZ);     // returns gyro in dps (bias corrected)
void mpu9250_kalmanFilter(float* pitchFiltered, float* gyroXYZ, float* accelXYZ, float dt);
void delay_ms(uint16_t delay);
void delay_us(uint16_t delay);

#endif // TWSBR_MPU9250_H
