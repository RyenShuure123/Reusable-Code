#include "ti_msp_dl_config.h"
#include "oled.h"
#include "servo.h"
#include "uart_pc.h"

#define DELAY_1S_CYCLES 32000000U

static const uint8_t kWelcomeMessage[] =
    "\r\nMSPM0 UART RX buffer + DMA TX echo ready\r\n";

int main(void)
{
    SYSCFG_DL_init();
    UART_PC_Init();
    //Servo_Init();

    UART_PC_StartDmaTransmit(
        kWelcomeMessage,
        (uint16_t)(sizeof(kWelcomeMessage) - 1U));

    delay_cycles(DELAY_1S_CYCLES);


    OLED_Init();
    OLED_NewFrame();
    OLED_ShowString(1, 1, "MSPM0G3507");
    OLED_ShowString(2, 1, "OLED OK");
    OLED_ShowFrame();

    while (1) {
        static uint8_t echoByte;

        if (!UART_PC_IsDmaTransmitBusy() &&
            UART_PC_TryReceiveByte(&echoByte)) {
            UART_PC_StartDmaTransmit(&echoByte, 1U);
            DL_GPIO_togglePins(GPIO_GRP_LED_PORT, GPIO_GRP_LED_PIN_LED_0_PIN);
        }
    }
}
