#ifndef MSPM0G3507_UART_DMA_UART_PC_H
#define MSPM0G3507_UART_DMA_UART_PC_H

#include <stdbool.h>
#include <stdint.h>

void UART_PC_Init(void);
void UART_PC_SendByte(uint8_t data);
void UART_PC_SendString(const char *text);
void UART_PC_SendHex8(uint8_t data);
bool UART_PC_TryReceiveByte(uint8_t *data);
uint16_t UART_PC_GetRxAvailable(void);
bool UART_PC_HasRxOverflowed(void);
bool UART_PC_StartDmaTransmit(const uint8_t *data, uint16_t length);
bool UART_PC_IsDmaTransmitBusy(void);

#endif //MSPM0G3507_UART_DMA_UART_PC_H
