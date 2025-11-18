#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#define ACCEL_ADDR 0x3B
#define devAddr 0x68
#define PWM_FREQUENCY 2000
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint32_t ui32SystemClock;
volatile uint32_t ui8Adjust1, ui8Adjust2, ui8Adjust3, ui8Adjust4;

uint8_t accel_reg[6];
int16_t accelXYZ_int[3];
float accelXYZ[3];

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

void wheels_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // PORTB is used to generate PWM signals
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Used to generate ENA signal
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // Used to read Encoder1 at PD6 and PD7
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Used to generate ENB signal at PF0
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Used to read Encoder2 at PC5 and PC6
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0); // Used to read motor2 Encoder
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1));
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
    ui32PWMClock = SysCtlPWMClockGet();
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4); // Used to set PE4 as EnableA
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0); // Used to set PF0 as EnableB

    // PB4 and PB5 as PWM outputs for Motor1
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinConfigure(GPIO_PB5_M0PWM3);

    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);
    QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_NO_SWAP, 7000);
    QEIFilterEnable(QEI1_BASE);
    QEIFilterConfigure(QEI1_BASE, QEI_FILTCNT_3);
    QEIPositionSet(QEI1_BASE, 0);
    QEIVelocityEnable(QEI1_BASE);
    QEIEnable(QEI1_BASE);

    // PB6 and PB7 as PWM outputs for Motor2
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_6 | GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET | QEI_CONFIG_SWAP, 7000);
    QEIFilterEnable(QEI0_BASE);
    QEIFilterConfigure(QEI0_BASE, QEI_FILTCNT_3);
    QEIPositionSet(QEI0_BASE, 0);
    QEIVelocityEnable(QEI0_BASE);
    QEIEnable(QEI0_BASE);

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
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_PIN_0); // Set EnableB HIGH to run motor2
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
    else if(wheel_id == 2)  // Use 2 instead of -1
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

#define BUFFER_SIZE 250
char responseESP[BUFFER_SIZE] = {0};  // Buffer to store the response
volatile bool timerExpired = false;    // Timer interrupt flag
char gateway[16] = {0};
uint16_t adcVals[3] = {0};

// UART0 Initialization (for printing to USB/UART)
void UART0_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTEnable(UART0_BASE);
}

// Print string over UART0 (USB to PC terminal)
void UART0_PrintString(char *str) {
    while (*str) {
        UARTCharPut(UART0_BASE, *str);
        str++;
    }
}

// Send AT command to ESP01
void ESP01_sendATCommand(char* command, uint16_t length) {
    uint16_t i = 0;
    for (i = 0; i < length; i++) {
        UARTCharPut(UART1_BASE, command[i]);
    }
}

// Timer Interrupt Handler
void Timer0A_Handler(void) {
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Set the flag to indicate timer expired
    timerExpired = true;
}

// Initialize Timer0 for timeout
void Timer0_Init(uint32_t timeoutMs) {
    // Enable Timer0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}

    // Configure Timer0 as 32-bit periodic timer
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Calculate timer load value for the desired timeout
    // System clock is 50MHz, so each tick is 20ns
//    uint32_t timerLoadValue = (timeoutMs * 50000) - 1; // timeoutMs * (50MHz / 1000)
    uint32_t timerLoadValue = timeoutMs;
    // Set the timer load value
    TimerLoadSet(TIMER0_BASE, TIMER_A, timerLoadValue);

    // Register the interrupt handler
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0A_Handler);

    // Enable the timer interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timer (but don't start it yet)
    TimerEnable(TIMER0_BASE, TIMER_A);
}

// Read response using timer for timeout
void ESP01_readResponse(char* responseESP, uint32_t timeoutMs) {
    uint32_t count = 0;
    timerExpired = false;

    // Clear buffer
    int i;
    for (i = 0; i < BUFFER_SIZE; i++) {
        responseESP[i] = 0;
    }

    // Initialize and start timer
    Timer0_Init(timeoutMs);

    // Start the timer
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Keep reading until timer expires or buffer is full
    while (!timerExpired && count < BUFFER_SIZE - 1) {
        if (UARTCharsAvail(UART1_BASE)) {
            char receivedChar = UARTCharGetNonBlocking(UART1_BASE);
            responseESP[count] = receivedChar;
            count++;
        }
    }

    // Stop the timer
    TimerDisable(TIMER0_BASE, TIMER_A);

    responseESP[count] = '\0';  // Null terminate
}

uint8_t ESP01_checkResponse(char* response, char* benchmark) {
    uint8_t status;
    while(*response != '\0') {
        if(*response == 'O', *(response + 1) == 'K') {
            status = 1;
        }
        else {
            status == 0;
        }
        response++;
    }
    return status;
}

void ESP01_extractADCVals(char* json) {
    int adc0 = -1, adc1 = -1, adc2 = -1;

    // Extract ADC0
    const char* adc0_start = strstr(json, "\"adc0\":");
    if (adc0_start) {
        adc0_start += 7; // Move past "\"adc0\":"
        adc0 = atoi(adc0_start);
    }

    // Extract ADC1
    const char* adc1_start = strstr(json, "\"adc1\":");
    if (adc1_start) {
        adc1_start += 7; // Move past "\"adc1\":"
        adc1 = atoi(adc1_start);
    }

    // Extract ADC2
    const char* adc2_start = strstr(json, "\"adc2\":");
    if (adc2_start) {
        adc2_start += 7; // Move past "\"adc2\":"
        adc2 = atoi(adc2_start);
    }

    adcVals[0] = adc0;
    adcVals[1] = adc1;
    adcVals[2] = adc2;
}

// ESP01 Setup (UART1 for communicating with ESP01)
void ESP01_Setup(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTEnable(UART1_BASE);
    // Clear any existing data in UART buffer
    while (UARTCharsAvail(UART1_BASE)) {
        UARTCharGetNonBlocking(UART1_BASE);
    }
//    ESP01_sendATCommand("AT+RST\r\n", 8);  // Send the AT command to ESP01
//    ESP01_readResponse(responseESP, 5000000000); // 75000 for OK
    // AT Test
    ESP01_sendATCommand("AT\r\n", 4);  // Send the AT command to ESP01
    ESP01_readResponse(responseESP, 75000000); // 75000 for OK
    if(ESP01_checkResponse(responseESP, "OK")) {
        UART0_PrintString("ESP01 connected\r\n");
    }
    else {
        UART0_PrintString("ESP01 not connected\r\n");
        UART0_PrintString(responseESP);
        UART0_PrintString("\r\n");
        return;
    }

    // AT+CWMODE=1 for STA mode
    ESP01_sendATCommand("AT+CWMODE=1\r\n", 15);
    ESP01_readResponse(responseESP, 75000);
    if(ESP01_checkResponse(responseESP, "OK")) {
        UART0_PrintString("ESP01 in STA mode\r\n");
    }
    else {
        UART0_PrintString("ESP01 not in STA mode\r\n");
        UART0_PrintString(responseESP);
        UART0_PrintString("\r\n");
        return;
    }

    // AT+CWJAP="ADC_Server","password123" for connecting to Access Point
    ESP01_sendATCommand("AT+CWJAP=\"ADC_Server\",\"password123\"\r\n", 41);
    ESP01_readResponse(responseESP, 50000000);
    if(ESP01_checkResponse(responseESP, "OK")) {
        UART0_PrintString("ESP01 connected to Access Point\r\n");
    }
    else {
        UART0_PrintString("Debug: ESP01 not connected to Access Point\r\n");
        UART0_PrintString(responseESP);
        UART0_PrintString("\r\n");
        return;
    }

    // AT+CIPSTA?
    ESP01_sendATCommand("AT+CIPSTA?\r\n", 14);
    ESP01_readResponse(responseESP, 400000);
    if(ESP01_checkResponse(responseESP, "OK")) {
        const char* gateway_start = strstr(responseESP, "gateway:\"");
        if (gateway_start) {
            gateway_start += 9; // Move past "gateway:\""
            const char* gateway_end = strchr(gateway_start, '"');
            if (gateway_end) {
                size_t len = gateway_end - gateway_start;
                strncpy(gateway, gateway_start, len);
                UART0_PrintString("Gateway: ");
                UART0_PrintString(gateway);
                UART0_PrintString("\r\n");
            }
        }
    }
    else {
        UART0_PrintString("Debug: ESP01 Gateway not obtained.\r\n");
        UART0_PrintString(responseESP);
        UART0_PrintString("\r\n");
        return;
    }
}

void ESP01_fetchData() {
    // AT+CIPSTART="TCP","192.168.4.1",80
    ESP01_sendATCommand("AT+CIPSTART=\"TCP\",\"192.168.4.1\",80\r\n", 36);
    ESP01_readResponse(responseESP, 150000);
    UART0_PrintString(responseESP);

    // AT+CIPSEND=18
    ESP01_sendATCommand("AT+CIPSEND=18\r\n", 31);
    ESP01_readResponse(responseESP, 75000);
    if(ESP01_checkResponse(responseESP, "OK")) {
        UART0_PrintString("ESP01 ready to receive GET request\r\n");
    }
    else {
        UART0_PrintString("Debug: ESP01 not ready to receive GET request\r\n");
        UART0_PrintString(responseESP);
        UART0_PrintString("\r\n");
        return;
    }

    ESP01_sendATCommand("GET /json HTTP/1.1\r\n", 21);
    ESP01_readResponse(responseESP, 5000000);
    if(ESP01_checkResponse(responseESP, "OK")) {
        UART0_PrintString("ESP01 sent data successfully\r\n");
        UART0_PrintString(responseESP);
        UART0_PrintString("\r\n");
        ESP01_extractADCVals(responseESP);
    }
    else {
        UART0_PrintString("Debug: ESP01 didn't sent data successfully\r\n");
        UART0_PrintString(responseESP);
        UART0_PrintString("\r\n");
        return;
    }

    ESP01_sendATCommand("AT+CIPCLOSE\r\n", 13);
    ESP01_readResponse(responseESP, 5000000);
    if(ESP01_checkResponse(responseESP, "OK")) {
        UART0_PrintString("ESP01 connection closed\r\n");
    }
    else {
        UART0_PrintString("Debug: ESP01 connection wasn't closed\r\n");
        UART0_PrintString(responseESP);
        UART0_PrintString("\r\n");
        return;
    }
}

// Main function
int main(void) {
    // Set system clock to 50 MHz
    SysCtlClockSet(SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_SYSDIV_1);

    // Enable processor interrupts
    IntMasterEnable();

    // Initialize UART0 for printing to USB
    UART0_Init();

    // Give ESP01 time to boot up
    SysCtlDelay(SysCtlClockGet()/3);  // Delay ~1 second

    // Send AT command to ESP01
    UART0_PrintString("Sending AT command to ESP01...\r\n");

    // Initialize UART1 for ESP01 communication
    ESP01_Setup();

    // Print the response
//    UART0_PrintString("Response from ESP01: ");
//    UART0_PrintString(responseESP);
//    UART0_PrintString("\r\n");
      // Send the AT command to ESP01
    // Main loop
    while (1) {
        // Main loop can handle other tasks
        ESP01_fetchData();
        SysCtlDelay(SysCtlClockGet()/600);
    }

    return 0;
}
