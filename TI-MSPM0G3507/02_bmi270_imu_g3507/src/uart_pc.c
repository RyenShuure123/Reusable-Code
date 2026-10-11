#include "uart_pc.h"
#include "ti_msp_dl_config.h"

#define UART_PC_RX_BUFFER_SIZE 128U
#define UART_PC_RX_BUFFER_MASK (UART_PC_RX_BUFFER_SIZE - 1U)

static volatile bool g_dmaTransmitBusy;
static volatile uint8_t g_rxBuffer[UART_PC_RX_BUFFER_SIZE];
static volatile uint16_t g_rxReadIndex;
static volatile uint16_t g_rxWriteIndex;
static volatile bool g_rxOverflowed;
static volatile uint8_t g_rxDmaByte;

static void UART_PC_StoreReceivedByte(uint8_t data)
{
    uint16_t nextWriteIndex =
        (g_rxWriteIndex + 1U) & UART_PC_RX_BUFFER_MASK;

    if (nextWriteIndex == g_rxReadIndex) {
        g_rxOverflowed = true;
        return;
    }

    g_rxBuffer[g_rxWriteIndex] = data;
    g_rxWriteIndex = nextWriteIndex;
}

static void UART_PC_ArmDmaReceive(void)
{
    DL_UART_Main_clearInterruptStatus(
        UART_PC_INST,
        DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);

    DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);
    DL_DMA_setSrcAddr(
        DMA,
        DMA_CH1_CHAN_ID,
        (uint32_t)(uintptr_t)&UART_PC_INST->RXDATA);
    DL_DMA_setDestAddr(
        DMA,
        DMA_CH1_CHAN_ID,
        (uint32_t)(uintptr_t)&g_rxDmaByte);
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, 1U);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
}

void UART_PC_Init(void)
{
    g_dmaTransmitBusy = false;
    g_rxReadIndex = 0U;
    g_rxWriteIndex = 0U;
    g_rxOverflowed = false;
    g_rxDmaByte = 0U;
    UART_PC_ArmDmaReceive();
    NVIC_EnableIRQ(UART_PC_INST_INT_IRQN);
}

void UART_PC_SendByte(uint8_t data)
{
    while (DL_UART_Main_isTXFIFOFull(UART_PC_INST)) {
    }

    DL_UART_Main_transmitData(UART_PC_INST, data);
}

void UART_PC_SendString(const char *text)
{
    if (text == NULL) {
        return;
    }

    while (*text != '\0') {
        UART_PC_SendByte((uint8_t)*text);
        text++;
    }
}

void UART_PC_SendHex8(uint8_t data)
{
    static const char hexCharacters[] = "0123456789ABCDEF";

    UART_PC_SendByte((uint8_t)hexCharacters[data >> 4U]);
    UART_PC_SendByte((uint8_t)hexCharacters[data & 0x0FU]);
}

bool UART_PC_TryReceiveByte(uint8_t *data)
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
    g_rxReadIndex = (readIndex + 1U) & UART_PC_RX_BUFFER_MASK;
    return true;
}

uint16_t UART_PC_GetRxAvailable(void)
{
    return (g_rxWriteIndex - g_rxReadIndex) & UART_PC_RX_BUFFER_MASK;
}

bool UART_PC_HasRxOverflowed(void)
{
    return g_rxOverflowed;
}

bool UART_PC_StartDmaTransmit(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U) || g_dmaTransmitBusy) {
        return false;
    }

    g_dmaTransmitBusy = true;

    DL_UART_Main_clearInterruptStatus(
        UART_PC_INST,
        DL_UART_MAIN_INTERRUPT_DMA_DONE_TX |
            DL_UART_MAIN_INTERRUPT_EOT_DONE);

    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(uintptr_t)data);
    DL_DMA_setDestAddr(
        DMA,
        DMA_CH0_CHAN_ID,
        (uint32_t)(uintptr_t)&UART_PC_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, length);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    return true;
}

bool UART_PC_IsDmaTransmitBusy(void)
{
    return g_dmaTransmitBusy;
}

void UART_PC_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_PC_INST)) {
        case DL_UART_MAIN_IIDX_DMA_DONE_TX:
            break;

        case DL_UART_MAIN_IIDX_EOT_DONE:
            g_dmaTransmitBusy = false;
            break;

        case DL_UART_MAIN_IIDX_DMA_DONE_RX:
            UART_PC_StoreReceivedByte(g_rxDmaByte);
            UART_PC_ArmDmaReceive();
            break;

        default:
            break;
    }
}
