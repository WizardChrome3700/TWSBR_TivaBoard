#ifndef MPU9250_H
#define MPU9250_H

#include <stdbool.h>
#include <stdint.h>

#define ACCEL_ADDR 0x3B
#define GYRO_ADDR 0x43
#define devAddr 0x68

// Function declarations
void mpu9250_init(void);
void recover_i2c_bus(void);
void check_i2c_bus_status(void);
void delay_ms(uint16_t delay);
void delay_us(uint16_t delay);

// Accelerometer functions
void mpu9250_readAcceleration(uint8_t* accelArray, float* accelXYZ);
bool mpu9250_readBytes(uint8_t registerAddr, uint8_t bytes, uint8_t* data);
bool mpu9250_calculate_bias(void);
void mpu9250_apply_bias_correction(float* accelXYZ);
float mpu9250_calculate_pitch_degrees_robust(float accelX, float accelY, float accelZ);

// Gyroscope functions
bool mpu9250_read_gyro_raw(int16_t* gyroXYZ);
bool mpu9250_read_gyro_dps(float* gyroXYZ);
bool mpu9250_read_gyro_dps_corrected(float* gyroXYZ);
bool mpu9250_calibrate_gyro_bias(void);
void mpu9250_get_gyro(float* gyroXYZ);

// Bias status functions
bool mpu9250_is_accel_bias_calibrated(void);
bool mpu9250_is_gyro_bias_calibrated(void);

void mpu9250_kalmanFilter(float* pitchFiltered, float* gyroXYZ, float* accelXYZ);

#endif
