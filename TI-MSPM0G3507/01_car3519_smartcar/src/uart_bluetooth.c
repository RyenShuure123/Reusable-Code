#include "uart_bluetooth.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define UART_BLUETOOTH_RX_BUFFER_SIZE 128U
#define UART_BLUETOOTH_RX_BUFFER_MASK (UART_BLUETOOTH_RX_BUFFER_SIZE - 1U)

static volatile bool g_dmaTransmitBusy;
static volatile uint8_t g_rxBuffer[UART_BLUETOOTH_RX_BUFFER_SIZE];
static volatile uint16_t g_rxReadIndex;
static volatile uint16_t g_rxWriteIndex;
static volatile bool g_rxOverflowed;
static volatile uint8_t g_rxDmaByte;

void UART_BLUETOOTH_OnTransmitComplete(void) __attribute__((weak));

void UART_BLUETOOTH_OnTransmitComplete(void)
{
}

static void UART_BLUETOOTH_StoreReceivedByte(uint8_t data)
{
    uint16_t nextWriteIndex =
        (g_rxWriteIndex + 1U) & UART_BLUETOOTH_RX_BUFFER_MASK;

    if (nextWriteIndex == g_rxReadIndex) {
        g_rxOverflowed = true;
        return;
    }

    g_rxBuffer[g_rxWriteIndex] = data;
    g_rxWriteIndex = nextWriteIndex;
}

static void UART_BLUETOOTH_ArmDmaReceive(void)
{
    DL_UART_Main_clearInterruptStatus(
        UART_BLUETOOTH_INST,
        DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
    DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);
    DL_DMA_setSrcAddr(
        DMA,
        DMA_CH1_CHAN_ID,
        (uint32_t) (uintptr_t) &UART_BLUETOOTH_INST->RXDATA);
    DL_DMA_setDestAddr(
        DMA,
        DMA_CH1_CHAN_ID,
        (uint32_t) (uintptr_t) &g_rxDmaByte);
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, 1U);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
}

void UART_BLUETOOTH_Init(void)
{
    g_dmaTransmitBusy = false;
    g_rxReadIndex = 0U;
    g_rxWriteIndex = 0U;
    g_rxOverflowed = false;
    g_rxDmaByte = 0U;

    UART_BLUETOOTH_ArmDmaReceive();
    NVIC_EnableIRQ(UART_BLUETOOTH_INST_INT_IRQN);
}

void UART_BLUETOOTH_SendByte(uint8_t data)
{
    if (g_dmaTransmitBusy) {
        return;
    }

    while (DL_UART_Main_isTXFIFOFull(UART_BLUETOOTH_INST)) {
    }

    DL_UART_Main_transmitData(UART_BLUETOOTH_INST, data);
}

void UART_BLUETOOTH_SendString(const char *text)
{
    if ((text == NULL) || g_dmaTransmitBusy) {
        return;
    }

    while (*text != '\0') {
        UART_BLUETOOTH_SendByte((uint8_t) *text);
        text++;
    }
}

void UART_BLUETOOTH_SendHex8(uint8_t data)
{
    static const char hexCharacters[] = "0123456789ABCDEF";

    UART_BLUETOOTH_SendByte((uint8_t) hexCharacters[data >> 4U]);
    UART_BLUETOOTH_SendByte((uint8_t) hexCharacters[data & 0x0FU]);
}

bool UART_BLUETOOTH_TryReceiveByte(uint8_t *data)
{
    uint16_t readIndex;

    if (data == NULL) {
        return false;
    }

    readIndex = g_rxReadIndex;
    if (readIndex == g_rxWriteIndex) {
        return false;
    }

    *data = g_rxBuffer[readIndex];
    g_rxReadIndex = (readIndex + 1U) & UART_BLUETOOTH_RX_BUFFER_MASK;
    return true;
}

uint16_t UART_BLUETOOTH_GetRxAvailable(void)
{
    return (g_rxWriteIndex - g_rxReadIndex) & UART_BLUETOOTH_RX_BUFFER_MASK;
}

bool UART_BLUETOOTH_HasRxOverflowed(void)
{
    return g_rxOverflowed;
}

bool UART_BLUETOOTH_StartDmaTransmit(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U) || g_dmaTransmitBusy) {
        return false;
    }

    g_dmaTransmitBusy = true;
    DL_UART_Main_clearInterruptStatus(
        UART_BLUETOOTH_INST,
        DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
            DL_UART_MAIN_INTERRUPT_EOT_DONE);
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_setSrcAddr(
        DMA, DMA_CH0_CHAN_ID, (uint32_t) (uintptr_t) data);
    DL_DMA_setDestAddr(
        DMA,
        DMA_CH0_CHAN_ID,
        (uint32_t) (uintptr_t) &UART_BLUETOOTH_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, length);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    return true;
}

bool UART_BLUETOOTH_IsDmaTransmitBusy(void)
{
    return g_dmaTransmitBusy;
}

void UART_BLUETOOTH_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_BLUETOOTH_INST)) {
    case DL_UART_MAIN_IIDX_DMA_DONE_RX:
        UART_BLUETOOTH_StoreReceivedByte(g_rxDmaByte);
        UART_BLUETOOTH_ArmDmaReceive();
        break;

    case DL_UART_MAIN_IIDX_DMA_DONE_TX:
        break;

    case DL_UART_MAIN_IIDX_EOT_DONE:
        g_dmaTransmitBusy = false;
        UART_BLUETOOTH_OnTransmitComplete();
        break;

    default:
        break;
    }
}
