#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#include "TWSBR_MPU9250.h"

#define ACCEL_ADDR 0x3B
#define GYRO_ADDR 0x43
#define devAddr 0x68

#define GYRO_FULL_SCALE 500.0f  // ±500 dps - adjust based on your configuration
#define I2C_TIMEOUT_ATTEMPTS 3
#define I2C_BUSY_TIMEOUT 4000

void delay_ms(uint16_t delay)
{
    SysCtlDelay((SysCtlClockGet()/3000)*delay);
}

void delay_us(uint16_t delay)
{
    uint32_t clockFreq = SysCtlClockGet();
    uint32_t loops = (uint32_t)(((uint64_t)clockFreq * delay) / 3000000);
    SysCtlDelay(loops);
}

void check_i2c_bus_status(void)
{
    // Temporarily configure SCL and SDA as GPIO inputs to check their state
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    // Add small delay to let pins stabilize
    delay_us(10);

    uint32_t pins = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    // Check if either line is stuck low
    if((pins & GPIO_PIN_2) == 0) {
        // SCL is stuck low - need recovery
        recover_i2c_bus();
    }
    else if((pins & GPIO_PIN_3) == 0) {
        // SDA is stuck low - need recovery
        recover_i2c_bus();
    }
    else {
        // Both lines are high - bus is likely fine
        // Restore I2C function
        GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        GPIOPinConfigure(GPIO_PB3_I2C0SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
        GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    }
}

void recover_i2c_bus(void)
{
    // Temporarily configure SCL and SDA as GPIO outputs
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    // Drive both lines high initially
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2 | GPIO_PIN_3);
    delay_ms(1);

    // Generate clock pulses until SDA is released
    int i = 0;
    while(i < 16) {
        // Drive SCL low
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
        delay_us(5);

        // Check if SDA is high (released)
        if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3)) {
            // SDA is released, do one more clock and stop
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
            delay_us(5);
            break;
        }

        // Drive SCL high
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
        delay_us(5);
        i = i + 1;
    }

    // Send STOP condition (SDA low→high while SCL high)
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
    delay_us(5);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
    delay_us(5);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
    delay_us(5);

    // Restore I2C function
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
}

bool mpu9250_writeByte(uint8_t registerAddr, uint8_t data)
{
    uint32_t timeout_counter;
    uint8_t retry_count = 0;

    for(retry_count = 0; retry_count < I2C_TIMEOUT_ATTEMPTS; retry_count++) {
        check_i2c_bus_status();

        // Initialize I2C on each attempt
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

        // Set slave address for write
        I2CMasterSlaveAddrSet(I2C0_BASE, devAddr, false);

        // Send register address
        I2CMasterDataPut(I2C0_BASE, registerAddr);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        // Wait for completion
        timeout_counter = 0;
        while(I2CMasterBusy(I2C0_BASE) && (timeout_counter < I2C_BUSY_TIMEOUT)) {
            timeout_counter++;
        }

        if(timeout_counter >= I2C_BUSY_TIMEOUT ||
           I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) {
            SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
            delay_ms(1);
            continue;
        }

        // Send data byte
        I2CMasterDataPut(I2C0_BASE, data);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

        // Wait for completion
        timeout_counter = 0;
        while(I2CMasterBusy(I2C0_BASE) && (timeout_counter < I2C_BUSY_TIMEOUT)) {
            timeout_counter++;
        }

        if(I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE) {
            return true; // Success
        }

        // Reset and retry on failure
        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
        delay_ms(5);
    }

    return false; // All attempts failed
}

void mpu9250_init()
{
    // Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));

    // Reset I2C module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    delay_ms(10);

    // Check and recover I2C bus if needed
    check_i2c_bus_status();

    // Configure I2C pins
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    delay_ms(100);

    // *** ADD PROPER MPU9250 CONFIGURATION ***

    // Reset MPU9250
    mpu9250_writeByte(0x6B, 0x80);
    delay_ms(100);

    // Wake up MPU9250 and select best clock source
    mpu9250_writeByte(0x6B, 0x01);
    delay_ms(10);

    // Configure gyroscope range (±500 dps)
    mpu9250_writeByte(0x1B, 0x08);

    // Configure accelerometer range (±2g)
    mpu9250_writeByte(0x1C, 0x00);

    // Configure digital low pass filter (optional)
    mpu9250_writeByte(0x1A, 0x03); // 41Hz bandwidth

    // Disable I2C master mode (we're using direct I2C)
    mpu9250_writeByte(0x24, 0x00);

    delay_ms(10);

    // Optional: Add a simple I2C test here to verify communication
    // You could try reading the WHO_AM_I register (0x75) which should return 0x71 for MPU9250
}

bool mpu9250_readBytes(uint8_t registerAddr, uint8_t bytes, uint8_t* data)
{
    uint32_t timeout_counter;
    uint8_t retry_count = 0;

    for(retry_count = 0; retry_count < I2C_TIMEOUT_ATTEMPTS; retry_count++) {
        check_i2c_bus_status();

        // Initialize I2C on each attempt
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

        // First set register address (write operation)
        I2CMasterSlaveAddrSet(I2C0_BASE, devAddr, false);
        I2CMasterDataPut(I2C0_BASE, registerAddr);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        // Wait for write to complete with timeout
        timeout_counter = 0;
        while(I2CMasterBusy(I2C0_BASE) && (timeout_counter < I2C_BUSY_TIMEOUT)) {
            timeout_counter++;
        }

        if(timeout_counter >= I2C_BUSY_TIMEOUT) {
            // Timeout occurred - reset and retry
            SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
            delay_ms(1);
            continue; // Retry the entire operation
        }

        if(I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) {
            // Error - reset and retry
            SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
            delay_ms(1);
            continue; // Retry the entire operation
        }

        // Read multiple bytes
        I2CMasterSlaveAddrSet(I2C0_BASE, devAddr, true);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

        bool read_success = true;
        uint8_t bytes_remaining = bytes;
        uint8_t data_index = 0;

        while(bytes_remaining > 0 && read_success)
        {
            timeout_counter = 0;
            while(I2CMasterBusy(I2C0_BASE) && (timeout_counter < I2C_BUSY_TIMEOUT)) {
                timeout_counter++;
            }

            if(timeout_counter >= I2C_BUSY_TIMEOUT) {
                read_success = false;
                break;
            }

            if(I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE) {
                data[data_index++] = I2CMasterDataGet(I2C0_BASE);
            } else {
                read_success = false;
                break;
            }

            bytes_remaining--;

            if(bytes_remaining > 0) {
                if(bytes_remaining == 1) {
                    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                } else {
                    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
                }
            }
        }

        if(read_success) {
            return true; // Success
        }

        // If we get here, the read failed - reset and retry
        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
        delay_ms(5);
    }

    // All retry attempts failed
    return false;
}

#define NUM_CALIBRATION_SAMPLES 500
float accel_bias[3] = {0.0f, 0.0f, 0.0f};
bool bias_calibrated = false;

// Function to calculate bias (should be called when sensor is stationary and level)
bool mpu9250_calculate_bias(void)
{
    float samples[3] = {0.0f, 0.0f, 0.0f};
    float temp_accel[3];
    uint8_t accel_reg[6];
    uint32_t valid_samples = 0;

    // Take multiple samples for averaging
    uint16_t i;
    for(i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
        if(mpu9250_readBytes(ACCEL_ADDR, 6, accel_reg)) {
            // Convert raw data to acceleration (reusing your existing conversion logic)
            int16_t accel_raw[3];
            accel_raw[0] = (accel_reg[0] << 8) | accel_reg[1];
            accel_raw[1] = (accel_reg[2] << 8) | accel_reg[3];
            accel_raw[2] = (accel_reg[4] << 8) | accel_reg[5];

            temp_accel[0] = (float)accel_raw[0] / 16384.0f;
            temp_accel[1] = (float)accel_raw[1] / 16384.0f;
            temp_accel[2] = (float)accel_raw[2] / 16384.0f;

            // Accumulate samples
            samples[0] += temp_accel[0];
            samples[1] += temp_accel[1];
            samples[2] += temp_accel[2];
            valid_samples++;
        }
        delay_ms(2); // Small delay between samples
    }

    if(valid_samples > 0) {
        // Calculate average (this is the bias)
        accel_bias[0] = samples[0] / valid_samples;
        accel_bias[1] = samples[1] / valid_samples;
        accel_bias[2] = samples[2] / valid_samples - 1.0f; // Subtract gravity from Z-axis

        bias_calibrated = true;
        return true;
    }

    return false;
}

// Function to apply bias correction to acceleration readings
void mpu9250_apply_bias_correction(float* accelXYZ)
{
    if(bias_calibrated) {
        accelXYZ[0] -= accel_bias[0];
        accelXYZ[1] -= accel_bias[1];
        accelXYZ[2] -= accel_bias[2];
    }
}

// Updated read acceleration function with bias correction
void mpu9250_readAcceleration(uint8_t* accelArray, float* accelXYZ)
{
    uint8_t accel_reg[6];
    int16_t accelXYZ_int[3];
    uint8_t count = 3;

    if(mpu9250_readBytes(ACCEL_ADDR, 6, accel_reg)) {
        while(count > 0)
        {
            accelXYZ_int[3-count] = accel_reg[2*(3-count)] << 8;
            accelXYZ_int[3-count] = accelXYZ_int[3-count] | accel_reg[2*(3-count) + 1];
            accelXYZ[3-count] = (float)accelXYZ_int[3-count] / 16384.0f;
            count--;
        }

        // Apply bias correction
        mpu9250_apply_bias_correction(accelXYZ);
    } else {
        // Handle read failure
        accelXYZ[0] = 0.0f;
        accelXYZ[1] = 0.0f;
        accelXYZ[2] = 0.0f;
    }
}

float mpu9250_calculate_pitch_degrees_robust(float accelX, float accelY, float accelZ)
{
    // Pitch = atan2(-Z, sqrt(X² + Y²))
    return atan2(-accelY, sqrt(accelX*accelX + accelZ*accelZ)) * (180.0 / M_PI);
}

float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
bool gyro_bias_calibrated = false;

// Read raw gyro data (16-bit integers)
bool mpu9250_read_gyro_raw(int16_t* gyroXYZ)
{
    uint8_t gyro_reg[6];

    if(mpu9250_readBytes(GYRO_ADDR, 6, gyro_reg)) {
        gyroXYZ[0] = (int16_t)((gyro_reg[0] << 8) | gyro_reg[1]);
        gyroXYZ[1] = (int16_t)((gyro_reg[2] << 8) | gyro_reg[3]);
        gyroXYZ[2] = (int16_t)((gyro_reg[4] << 8) | gyro_reg[5]);
        return true;
    }

    gyroXYZ[0] = 0;
    gyroXYZ[1] = 0;
    gyroXYZ[2] = 0;
    return false;
}

// Read gyro data in degrees per second (uses the raw function)
bool mpu9250_read_gyro_dps(float* gyroXYZ)
{
    int16_t raw_gyro[3];

    if(mpu9250_read_gyro_raw(raw_gyro)) {
        // Convert to degrees per second (±500 dps full scale)
        // Sensitivity: 65.5 LSB/(dps) for ±500dps
        float scale_factor = 1.0f / 65.5f;

        gyroXYZ[0] = (float)raw_gyro[0] * scale_factor;
        gyroXYZ[1] = (float)raw_gyro[1] * scale_factor;
        gyroXYZ[2] = (float)raw_gyro[2] * scale_factor;
        return true;
    }

    gyroXYZ[0] = 0.0f;
    gyroXYZ[1] = 0.0f;
    gyroXYZ[2] = 0.0f;
    return false;
}

// Calibrate gyro bias (call when sensor is stationary)
bool mpu9250_calibrate_gyro_bias(void)
{
    float samples[3] = {0.0f, 0.0f, 0.0f};
    float temp_gyro[3];
    uint32_t valid_samples = 0;

    // Take multiple samples for averaging
    uint16_t i;
    for(i = 0; i < 500; i++) {
        if(mpu9250_read_gyro_dps(temp_gyro)) {  // Read in dps directly
            samples[0] += temp_gyro[0];
            samples[1] += temp_gyro[1];
            samples[2] += temp_gyro[2];
            valid_samples++;
        }
        delay_ms(2);
    }

    if(valid_samples > 0) {
        // Calculate average bias (already in dps)
        gyro_bias[0] = samples[0] / valid_samples;
        gyro_bias[1] = samples[1] / valid_samples;
        gyro_bias[2] = samples[2] / valid_samples;

        gyro_bias_calibrated = true;
        return true;
    }

    return false;
}

// Read gyro with bias correction
bool mpu9250_read_gyro_dps_corrected(float* gyroXYZ)
{
    if(mpu9250_read_gyro_dps(gyroXYZ)) {
        if(gyro_bias_calibrated) {
            gyroXYZ[0] -= gyro_bias[0];
            gyroXYZ[1] -= gyro_bias[1];
            gyroXYZ[2] -= gyro_bias[2];
        }
        return true;
    }
    return false;
}

// Simple all-in-one function to get corrected gyro readings
void mpu9250_get_gyro(float* gyroXYZ)
{
    mpu9250_read_gyro_dps_corrected(gyroXYZ);
}

void mpu9250_kalmanFilter(float* pitchFiltered, float* gyroXYZ, float* accelXYZ)
{
    static float pitchAngle = 0.0f;           // Estimated pitch angle
    static float pitchBias = 0.0f;            // Gyro bias estimate
    static float P[2][2] = {{0.0f, 0.0f},     // Error covariance matrix
                           {0.0f, 0.0f}};

    // Kalman filter parameters (tune these for your application)
    const float Q_angle = 0.1f;   // Process noise - much lower
    const float Q_bias = 0.003f;    // Bias noise
    const float R_measure = 0.01f;  // Measurement noise - higher
    const float dt = 0.01f;          // Sample time in seconds (10ms)

    // Calculate pitch angle from accelerometer
    float pitchAccel = mpu9250_calculate_pitch_degrees_robust(accelXYZ[0], accelXYZ[1], accelXYZ[2]);

    // Prediction step (time update)
    // 1. Predict state
    float pitchRate = -gyroXYZ[0] - pitchBias;  // Gyro rate minus estimated bias
    pitchAngle += pitchRate * dt;

    // 2. Predict error covariance
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update step (measurement update)
    // 1. Calculate Kalman gain
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // 2. Update estimate with measurement
    float y = pitchAccel - pitchAngle;  // Innovation (measurement residual)
    pitchAngle += K[0] * y;
    pitchBias += K[1] * y;

    // 3. Update error covariance
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    // Return the filtered pitch angle
    *pitchFiltered = pitchAngle;
}
