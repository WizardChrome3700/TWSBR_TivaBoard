#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"

#include "TWSBR_Wheel.h"

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui32SystemClock;
volatile uint32_t ui8Adjust1, ui8Adjust2, ui8Adjust3, ui8Adjust4;

void wheels_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // PORTB is used to generate PWM signals
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Used to generate ENA signal
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // Used to generate ENB signal
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
    ui32PWMClock = SysCtlPWMClockGet();
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4); // Used to set PE4 as EnableA
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6); // Used to set PD6 as EnableB

    // PB4 and PB5 as PWM outputs for Motor1
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);

    // PB6 and PB7 as PWM outputs for Motor2
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);

    // PB4, PB5, PB6, PB7 get their PWM from Module0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));

    // (PB6 and PB7) get PWM from generator 0
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load);

    // (PB4 and PB5) get PWM from generator 1
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);

    // Set initial duty cycles (motors stopped)
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32Load/2); // PB4 - Motor 1 IN1
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, ui32Load/2); // PB5 - Motor 1 IN2
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ui32Load/2); // PB6 - Motor 2 IN1
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ui32Load/2); // PB7 - Motor 2 IN2

    // Set Output State of PWM_outputs to true
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);

    // Enable PWM Generator0 and Generator1
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    // Enable Pins HIGH to drive the motors
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // Set EnableA HIGH to run motor1
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_PIN_6); // Set EnableB HIGH to run motor2
}

void wheel_setSpeed(float control_input, int8_t wheel_id)
{
    float max_input = (float)(ui32Load/2);
    if(control_input > max_input) { control_input = max_input; }
    else if(control_input < -max_input) { control_input = -max_input; }

    int32_t adjusted_input = (int32_t)control_input;
    uint32_t halfLoad = ui32Load / 2;

    if(wheel_id == 1)
    {
        // Motor 1: PB4 (M0PWM2) and PB5 (M0PWM3)
        ui8Adjust1 = halfLoad + adjusted_input;  // PB4
        ui8Adjust2 = halfLoad - adjusted_input;  // PB5
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui8Adjust1); // CORRECT: PB4
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, ui8Adjust2); // CORRECT: PB5
    }
    else if(wheel_id == 2)  // Use 2 for second motor
    {
        // Motor 2: PB6 (M0PWM0) and PB7 (M0PWM1)
        ui8Adjust3 = halfLoad - adjusted_input;  // PB6
        ui8Adjust4 = halfLoad + adjusted_input;  // PB7
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ui8Adjust3); // CORRECT: PB6
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ui8Adjust4); // CORRECT: PB7
    }
    else
    {
        // Both motors (wheel_id = 0)
        ui8Adjust1 = halfLoad + adjusted_input;
        ui8Adjust2 = halfLoad - adjusted_input;
        ui8Adjust3 = halfLoad - adjusted_input;
        ui8Adjust4 = halfLoad + adjusted_input;

        // Motor 1
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui8Adjust1); // PB4
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, ui8Adjust2); // PB5

        // Motor 2
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ui8Adjust3); // PB6
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ui8Adjust4); // PB7
    }
}
