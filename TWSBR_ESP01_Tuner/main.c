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
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#include "TWSBR_ESP01_Tuner.h"

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
    //UART0_PrintString(responseESP);

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
