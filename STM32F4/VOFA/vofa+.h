#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>

void VOFA_Receive_Init(void);
void VOFA_ProcessCommands(void);
uint8_t VOFA_SendText(const char *text);
void VOFA_Send_6Float_DMA(float f1, float f2, float f3, float f4, float f5, float f6);
void VOFA_Send_8Float_DMA(float f1, float f2, float f3, float f4, float f5, float f6, float f7, float f8);
void VOFA_CommandReceivedCallback(const char *command, float value);

#endif
