#ifndef MPU9250_H
#define MPU9250_H

#include <stdbool.h>
#include <stdint.h>


#define ACCEL_ADDR 0x3B
#define devAddr 0x68

// Function declarations
void mpu9250_init(void);
void recover_i2c_bus(void);
void mpu9250_readAcceleration(uint8_t* accelArray, float* accelXYZ);

// Add other function declarations as needed

#endif
