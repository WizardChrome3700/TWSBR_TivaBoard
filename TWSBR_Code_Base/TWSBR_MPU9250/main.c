#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"

#include "TWSBR_MPU9250.h"

#define ACCEL_ADDR 0x3B
#define devAddr 0x68

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

void mpu9250_init()
{
    // Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));

    // Check I2C bus state before initialization
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    if(!GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_2)) { // SCL is LOW
        // Bus is hung - recover it
        recover_i2c_bus();
    }

    // Reset I2C module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    // Configure I2C pins
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
}

void mpu9250_readBytes(uint8_t registerAddr, uint8_t bytes, uint8_t* data)
{
    uint8_t i2c_busy;
    // Initialize I2C
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
    // Read multiple bytes from consecutive registers
    // First set register address (write operation)
    I2CMasterSlaveAddrSet(I2C0_BASE, devAddr, false);  // false = write
    I2CMasterDataPut(I2C0_BASE, registerAddr);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait for write to complete and check for errors
    while(I2CMasterBusy(I2C0_BASE));
    if(I2CMasterErr(I2C0_BASE) != I2C_MASTER_ERR_NONE) {
        // Handle error - reset and retry
        SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
        delay_ms(10);
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
        delay_ms(100);
        return;
    }

    // Then read multiple bytes (read operation)
    I2CMasterSlaveAddrSet(I2C0_BASE, devAddr, true);  // true = read
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    // Wait for completion, then read first byte
    while(bytes--)
    {
        while(I2CMasterBusy(I2C0_BASE));
        if(I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE) {
            data[5-bytes] = I2CMasterDataGet(I2C0_BASE);
        }

        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        if(bytes == 1)
        {
            break;
        }
    }

    // Finish burst read
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
    if(I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE) {
        data[5] = I2CMasterDataGet(I2C0_BASE);
    }

    // Success - process data here
    i2c_busy = false;
}

void mpu9250_readAcceleration(uint8_t* accelArray, float* accelXYZ)
{
    uint8_t accel_reg[6];
    int16_t accelXYZ_int[3];
    uint8_t count = 3;
    mpu9250_readBytes(ACCEL_ADDR, 6, accel_reg);
    while(count > 0)
    {
        accelXYZ_int[3-count] = accel_reg[2*(3-count)] << 8;
        accelXYZ_int[3-count] = accelXYZ_int[3-count] + accel_reg[2*(3-count) + 1];
        accelXYZ[3-count] = (float)accelXYZ_int[3-count] / 16384.0f;
        count--;
    }
}
