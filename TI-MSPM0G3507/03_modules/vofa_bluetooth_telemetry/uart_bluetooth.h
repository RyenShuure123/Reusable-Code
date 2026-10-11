#ifndef CAR3519NEW_UART_BLUETOOTH_H
#define CAR3519NEW_UART_BLUETOOTH_H

#include <stdbool.h>
#include <stdint.h>

void UART_BLUETOOTH_Init(void);
void UART_BLUETOOTH_SendByte(uint8_t data);
void UART_BLUETOOTH_SendString(const char *text);
void UART_BLUETOOTH_SendHex8(uint8_t data);
bool UART_BLUETOOTH_TryReceiveByte(uint8_t *data);
uint16_t UART_BLUETOOTH_GetRxAvailable(void);
bool UART_BLUETOOTH_HasRxOverflowed(void);
bool UART_BLUETOOTH_StartDmaTransmit(const uint8_t *data, uint16_t length);
bool UART_BLUETOOTH_IsDmaTransmitBusy(void);
void UART_BLUETOOTH_OnTransmitComplete(void);

#endif //CAR3519NEW_UART_BLUETOOTH_H
