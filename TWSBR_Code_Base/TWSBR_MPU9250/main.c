#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "TWSBR_MPU9250.h"

// I2C & MPU constants
#define ACCEL_ADDR 0x3B
#define GYRO_ADDR  0x43
#define devAddr    0x68

#define I2C_TIMEOUT_ATTEMPTS 3
#define I2C_BUSY_TIMEOUT 4000

// Filtering & Kalman parameters (tune as needed)
#define ACCEL_LPF_TAU        0.02f   // seconds
#define ACCEL_MEDIAN_SAMPLES 3
#define KAL_Q_ANGLE          0.001f
#define KAL_Q_BIAS           0.003f
#define KAL_R_MEAS           0.03f
#define ACCEL_MAG_THRESHOLD  0.25f   // g units

// PID parameters (tune to your motors & robot)
#define PID_KP  30.0f
#define PID_KI  0.5f
#define PID_KD  1.0f
#define PID_I_CLAMP  100.0f
#define PID_DEADZONE_DEG 0.7f   // degrees

// ---------- simple helpers ----------
static float median3(float a, float b, float c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

static float lpf_update(float prev, float in, float dt, float tau)
{
    if (tau <= 0.0f) return in;
    float alpha = dt / (tau + dt);
    return prev + alpha * (in - prev);
}

// simple blocking delays using SysCtlDelay
void delay_ms(uint16_t delay)
{
    // SysCtlDelay counts 3 cycles per loop
    uint32_t loops = (SysCtlClockGet() / 3000) * delay;
    SysCtlDelay(loops);
}
void delay_us(uint16_t delay)
{
    uint32_t clockFreq = SysCtlClockGet();
    uint32_t loops = (uint32_t)(((uint64_t)clockFreq * delay) / 3000000);
    SysCtlDelay(loops);
}

// ---------- I2C bus recovery / checks (PB2: SCL, PB3: SDA) ----------
void recover_i2c_bus(void)
{
    // Make pins GPIO outputs to toggle SCL until SDA releases
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    // Drive both high first
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2 | GPIO_PIN_3);
    delay_ms(1);

    int i = 0;
    while(i < 16) {
        // clock low
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
        delay_us(5);

        // check SDA
        if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_3) & GPIO_PIN_3) {
            // SDA released; pulse one more time and break
            GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
            delay_us(5);
            break;
        }

        // drive clock high
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
        delay_us(5);
        i++;
    }

    // send STOP: SDA low -> high while SCL high
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
    delay_us(5);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
    delay_us(5);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
    delay_us(5);

    // restore I2C pin functions
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
}

void check_i2c_bus_status(void)
{
    // configure as inputs to read
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    delay_us(10);
    uint32_t pins = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    if((pins & GPIO_PIN_2) == 0 || (pins & GPIO_PIN_3) == 0) {
        recover_i2c_bus();
    } else {
        // restore I2C config
        GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        GPIOPinConfigure(GPIO_PB3_I2C0SDA);
        GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
        GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    }
}

// ---------- basic I2C write/read ----------
bool mpu9250_writeByte(uint8_t registerAddr, uint8_t data)
{
    uint32_t timeout_counter;
    uint8_t retry_count;

    for(retry_count = 0; retry_count < I2C_TIMEOUT_ATTEMPTS; retry_count++) {
        check_i2c_bus_status();
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
        I2CMasterSlaveAddrSet(I2C0_BASE, devAddr, false);

        I2CMasterDataPut(I2C0_BASE, registerAddr);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        timeout_counter = 0;
        while(I2CMasterBusy(I2C0_BASE) && (timeout_counter < I2C_BUSY_TIMEOUT)) timeout_counter++;

        if(timeout_counter >= I2C_BUSY_TIMEOUT || I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) {
            SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
            delay_ms(1);
            continue;
        }

        I2CMasterDataPut(I2C0_BASE, data);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

        timeout_counter = 0;
        while(I2CMasterBusy(I2C0_BASE) && (timeout_counter < I2C_BUSY_TIMEOUT)) timeout_counter++;

        if(I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE) return true;

        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
        delay_ms(5);
    }
    return false;
}

bool mpu9250_readBytes(uint8_t registerAddr, uint8_t bytes, uint8_t* data)
{
    uint32_t timeout_counter;
    uint8_t retry_count;

    for(retry_count = 0; retry_count < I2C_TIMEOUT_ATTEMPTS; retry_count++) {
        check_i2c_bus_status();
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

        I2CMasterSlaveAddrSet(I2C0_BASE, devAddr, false);
        I2CMasterDataPut(I2C0_BASE, registerAddr);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

        timeout_counter = 0;
        while(I2CMasterBusy(I2C0_BASE) && (timeout_counter < I2C_BUSY_TIMEOUT)) timeout_counter++;

        if(timeout_counter >= I2C_BUSY_TIMEOUT) { SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0); delay_ms(1); continue; }
        if(I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) { SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0); delay_ms(1); continue; }

        I2CMasterSlaveAddrSet(I2C0_BASE, devAddr, true);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

        bool read_success = true;
        uint8_t bytes_remaining = bytes;
        uint8_t data_index = 0;

        while(bytes_remaining > 0 && read_success) {
            timeout_counter = 0;
            while(I2CMasterBusy(I2C0_BASE) && (timeout_counter < I2C_BUSY_TIMEOUT)) timeout_counter++;

            if(timeout_counter >= I2C_BUSY_TIMEOUT) { read_success = false; break; }

            if(I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE) {
                data[data_index++] = I2CMasterDataGet(I2C0_BASE);
            } else {
                read_success = false;
                break;
            }

            bytes_remaining--;
            if(bytes_remaining > 0) {
                if(bytes_remaining == 1) I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                else I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            }
        }

        if(read_success) return true;

        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
        delay_ms(5);
    }

    return false;
}

// ---------- MPU init ----------
void mpu9250_init()
{
    // ensure peripherals enabled
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));

    // reset I2C
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
    delay_ms(10);

    // check bus and recover if necessary
    check_i2c_bus_status();

    // configure I2C pins PB2 (SCL), PB3 (SDA)
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    delay_ms(100);

    // reset MPU
    mpu9250_writeByte(0x6B, 0x80);
    delay_ms(100);

    // Wake and set clock source
    mpu9250_writeByte(0x6B, 0x01);
    delay_ms(10);

    // Gyro ±500 dps (0x08), Accel ±2g (0x00)
    mpu9250_writeByte(0x1B, 0x08);
    mpu9250_writeByte(0x1C, 0x00);

    // DLPF ~41Hz
    mpu9250_writeByte(0x1A, 0x03);

    // disable I2C master
    mpu9250_writeByte(0x24, 0x00);
    delay_ms(10);
}

// ---------- bias calibration (accel + gyro) ----------
#define NUM_CALIBRATION_SAMPLES 500
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
static bool accel_bias_calibrated = false;

bool mpu9250_calculate_bias(void)
{
    float sum[3] = {0.0f,0.0f,0.0f};
    uint8_t buf[6];
    uint32_t valid = 0;

    uint16_t i;
    for(i=0;i<NUM_CALIBRATION_SAMPLES;i++){
        if(mpu9250_readBytes(ACCEL_ADDR, 6, buf)){
            int16_t raw0 = (int16_t)((buf[0]<<8) | buf[1]);
            int16_t raw1 = (int16_t)((buf[2]<<8) | buf[3]);
            int16_t raw2 = (int16_t)((buf[4]<<8) | buf[5]);
            float ax = raw0 / 16384.0f;
            float ay = raw1 / 16384.0f;
            float az = raw2 / 16384.0f;
            sum[0]+=ax; sum[1]+=ay; sum[2]+=az;
            valid++;
        }
        delay_ms(2);
    }

    if(valid>0){
        accel_bias[0]=sum[0]/valid;
        accel_bias[1]=sum[1]/valid;
        accel_bias[2]=sum[2]/valid - 1.0f; // subtract gravity
        accel_bias_calibrated = true;
        return true;
    }
    return false;
}

void mpu9250_apply_bias_correction(float* accelXYZ)
{
    if(accel_bias_calibrated){
        accelXYZ[0] -= accel_bias[0];
        accelXYZ[1] -= accel_bias[1];
        accelXYZ[2] -= accel_bias[2];
    }
}

// ---------- Accel read with median + LPF ----------
void mpu9250_readAcceleration(uint8_t* accelArray, float* accelXYZ)
{
    uint8_t accel_reg[6];
    int16_t accel_raw[3];

    if(!mpu9250_readBytes(ACCEL_ADDR, 6, accel_reg)){
        accelXYZ[0]=accelXYZ[1]=accelXYZ[2]=0.0f;
        return;
    }

    accel_raw[0] = (int16_t)((accel_reg[0]<<8) | accel_reg[1]);
    accel_raw[1] = (int16_t)((accel_reg[2]<<8) | accel_reg[3]);
    accel_raw[2] = (int16_t)((accel_reg[4]<<8) | accel_reg[5]);

    float ax = accel_raw[0] / 16384.0f;
    float ay = accel_raw[1] / 16384.0f;
    float az = accel_raw[2] / 16384.0f;

    // ring buffers for median
    static float ax_buf[ACCEL_MEDIAN_SAMPLES] = {0}, ay_buf[ACCEL_MEDIAN_SAMPLES] = {0}, az_buf[ACCEL_MEDIAN_SAMPLES] = {0};
    static uint8_t buf_index = 0;

    ax_buf[buf_index] = ax;
    ay_buf[buf_index] = ay;
    az_buf[buf_index] = az;
    buf_index = (buf_index + 1) % ACCEL_MEDIAN_SAMPLES;

    float ax_med = median3(ax_buf[0], ax_buf[1], ax_buf[2]);
    float ay_med = median3(ay_buf[0], ay_buf[1], ay_buf[2]);
    float az_med = median3(az_buf[0], az_buf[1], az_buf[2]);

    static float ax_lpf=0.0f, ay_lpf=0.0f, az_lpf=0.0f;
    const float dt_default = 0.01f; // 10ms default
    ax_lpf = lpf_update(ax_lpf, ax_med, dt_default, ACCEL_LPF_TAU);
    ay_lpf = lpf_update(ay_lpf, ay_med, dt_default, ACCEL_LPF_TAU);
    az_lpf = lpf_update(az_lpf, az_med, dt_default, ACCEL_LPF_TAU);

    accelXYZ[0]=ax_lpf;
    accelXYZ[1]=ay_lpf;
    accelXYZ[2]=az_lpf;

    mpu9250_apply_bias_correction(accelXYZ);
}

// ---------- Gyro raw & bias corrected ----------
static float gyro_bias[3] = {0};
static bool gyro_bias_calibrated = false;

bool mpu9250_read_gyro_raw(int16_t* gyroXYZ)
{
    uint8_t buf[6];
    if(mpu9250_readBytes(GYRO_ADDR,6,buf)){
        gyroXYZ[0] = (int16_t)((buf[0]<<8)|buf[1]);
        gyroXYZ[1] = (int16_t)((buf[2]<<8)|buf[3]);
        gyroXYZ[2] = (int16_t)((buf[4]<<8)|buf[5]);
        return true;
    }
    gyroXYZ[0]=gyroXYZ[1]=gyroXYZ[2]=0;
    return false;
}

bool mpu9250_read_gyro_dps(float* gyroXYZ)
{
    int16_t raw[3];
    if(!mpu9250_read_gyro_raw(raw)){
        gyroXYZ[0]=gyroXYZ[1]=gyroXYZ[2]=0.0f;
        return false;
    }
    // ±500 dps => sensitivity ~65.5 LSB/dps
    const float scale = 1.0f / 65.5f;
    gyroXYZ[0] = raw[0] * scale;
    gyroXYZ[1] = raw[1] * scale;
    gyroXYZ[2] = raw[2] * scale;
    return true;
}

bool mpu9250_calibrate_gyro_bias(void)
{
    float sum[3]={0};
    float tmp[3];
    uint32_t valid=0;
    uint16_t i;
    for(i=0;i<500;i++){
        if(mpu9250_read_gyro_dps(tmp)){
            sum[0]+=tmp[0]; sum[1]+=tmp[1]; sum[2]+=tmp[2];
            valid++;
        }
        delay_ms(2);
    }
    if(valid>0){
        gyro_bias[0]=sum[0]/valid;
        gyro_bias[1]=sum[1]/valid;
        gyro_bias[2]=sum[2]/valid;
        gyro_bias_calibrated = true;
        return true;
    }
    return false;
}

bool mpu9250_read_gyro_dps_corrected(float* gyroXYZ)
{
    if(!mpu9250_read_gyro_dps(gyroXYZ)) return false;
    if(gyro_bias_calibrated){
        gyroXYZ[0] -= gyro_bias[0];
        gyroXYZ[1] -= gyro_bias[1];
        gyroXYZ[2] -= gyro_bias[2];
    }
    return true;
}

void mpu9250_get_gyro(float* gyroXYZ)
{
    mpu9250_read_gyro_dps_corrected(gyroXYZ);
}

// ---------- robust Kalman filter (angle in degrees) ----------
float mpu9250_calculate_pitch_degrees_robust(float accelX, float accelY, float accelZ)
{
    // note: depending on your IMU orientation, you might need to swap axes
    // I've used pitch = atan2(-Y, sqrt(X^2 + Z^2)) (same as your prior code)
    return atan2f(-accelY, sqrtf(accelX*accelX + accelZ*accelZ)) * (180.0f / M_PI);
}

void mpu9250_kalmanFilter(float* pitchFiltered, float* gyroXYZ, float* accelXYZ, float dt)
{
    static float pitchAngle = 0.0f;
    static float pitchBias = 0.0f;
    static float P[2][2] = {{0,0},{0,0}};
    static float accel_angle_lpf = 0.0f;
    static int initialized = 0;

    if(!initialized){
        pitchAngle = mpu9250_calculate_pitch_degrees_robust(accelXYZ[0], accelXYZ[1], accelXYZ[2]);
        accel_angle_lpf = pitchAngle;
        initialized = 1;
    }

    // gyro rate -> pitch rate (dps)
    float gyro_rate_dps = -gyroXYZ[0]; // preserve previous sign convention; test and invert if necessary

    // predict
    float pitchRate = gyro_rate_dps - pitchBias;
    pitchAngle += pitchRate * dt;

    // covariance prediction
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + KAL_Q_ANGLE);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += KAL_Q_BIAS * dt;

    // measurement
    float pitchAccel = mpu9250_calculate_pitch_degrees_robust(accelXYZ[0], accelXYZ[1], accelXYZ[2]);
    const float accel_angle_tau = 0.03f;
    accel_angle_lpf = lpf_update(accel_angle_lpf, pitchAccel, dt, accel_angle_tau);

    // magnitude check
    float mag = sqrtf(accelXYZ[0]*accelXYZ[0] + accelXYZ[1]*accelXYZ[1] + accelXYZ[2]*accelXYZ[2]);
    float mag_err = fabsf(mag - 1.0f);
    float R_measure_local = KAL_R_MEAS;
    if(mag_err > ACCEL_MAG_THRESHOLD) R_measure_local *= 10.0f; // downweight accel

    // Kalman gain
    float S = P[0][0] + R_measure_local;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    float y = accel_angle_lpf - pitchAngle;

    // update
    pitchAngle += K0 * y;
    pitchBias += K1 * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K0 * P00_temp;
    P[0][1] -= K0 * P01_temp;
    P[1][0] -= K1 * P00_temp;
    P[1][1] -= K1 * P01_temp;

    *pitchFiltered = pitchAngle;
}
