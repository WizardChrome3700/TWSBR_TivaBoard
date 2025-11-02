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
#include "TWSBR_Wheel.h"

uint8_t accel_reg[6];
int16_t accelXYZ_int[3];
float accelXYZ[3];

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui32SystemClock;
volatile uint32_t ui8Adjust1, ui8Adjust2, ui8Adjust3, ui8Adjust4;

// Main application for I2C hardware
int main(void)
{
    // Initialize system clock
    SysCtlClockSet(SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_SYSDIV_5);

    mpu9250_init();
    delay_ms(100);
    wheels_init();
    while(true)
    {
//        mpu9250_readAcceleration(accel_reg, accelXYZ);
//         Delay before next read
//        delay_ms(10);
//        wheel_setSpeed(0, 0);
//        SysCtlDelay(40000000);
//        wheel_setSpeed(320, 0);
//        SysCtlDelay(40000000);
//        wheel_setSpeed(-320, 0);
//        SysCtlDelay(40000000);
    }

    return 0;
}

//int main(void)
//{
//    SysCtlClockSet(SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_SYSDIV_5);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
//    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
//    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
//    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
//    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, ((!GPIO_PIN_0) | (!GPIO_PIN_1) | (!GPIO_PIN_2) | (!GPIO_PIN_3) | (!GPIO_PIN_4) | (!GPIO_PIN_5) | (!GPIO_PIN_6) | (!GPIO_PIN_7)));
//    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, ((!GPIO_PIN_0) | (!GPIO_PIN_1) | (!GPIO_PIN_2) | (!GPIO_PIN_3) | (!GPIO_PIN_4) | (!GPIO_PIN_5) | (!GPIO_PIN_6) | (!GPIO_PIN_7)));
//    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, ((!GPIO_PIN_0) | (!GPIO_PIN_1) |  (GPIO_PIN_2) |  (GPIO_PIN_3) | (!GPIO_PIN_4) | (!GPIO_PIN_5) | (!GPIO_PIN_6) | (!GPIO_PIN_7)));
//    while(true)
//    {
//        delay_us(4);
//        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, ((!GPIO_PIN_0) | (!GPIO_PIN_1) | (!GPIO_PIN_2) |  (GPIO_PIN_3) | (!GPIO_PIN_4) | (!GPIO_PIN_5) | (!GPIO_PIN_6) | (!GPIO_PIN_7)));
//        delay_us(4);
//        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, ((!GPIO_PIN_0) | (!GPIO_PIN_1) | (!GPIO_PIN_2) | (!GPIO_PIN_3) | (!GPIO_PIN_4) | (!GPIO_PIN_5) | (!GPIO_PIN_6) | (!GPIO_PIN_7)));
//        delay_us(4);
//        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, ((!GPIO_PIN_0) | (!GPIO_PIN_1) |  (GPIO_PIN_2) | (!GPIO_PIN_3) | (!GPIO_PIN_4) | (!GPIO_PIN_5) | (!GPIO_PIN_6) | (!GPIO_PIN_7)));
//        delay_us(4);
//        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, ((!GPIO_PIN_0) | (!GPIO_PIN_1) |  (GPIO_PIN_2) |  (GPIO_PIN_3) | (!GPIO_PIN_4) | (!GPIO_PIN_5) | (!GPIO_PIN_6) | (!GPIO_PIN_7)));
//    }
//    return 0;
//}
