#ifndef TWSBR_ESP01_TUNER_H
#define TWSBR_ESP01_TUNER_H

#include <stdbool.h>
#include <stdint.h>

void UART0_Init(void);
void UART0_PrintString(char *str);
void ESP01_sendATCommand(char* command, uint16_t length);
void Timer0A_Handler(void);
void Timer0_Init(uint32_t timeoutMs);
void ESP01_readResponse(char* responseESP, uint32_t timeoutMs);
uint8_t ESP01_checkResponse(char* response, char* benchmark);
void ESP01_extractADCVals(char* json, uint16_t* adcVals);
void ESP01_Setup();
void ESP01_fetchData(uint16_t* adcVals);

#endif
