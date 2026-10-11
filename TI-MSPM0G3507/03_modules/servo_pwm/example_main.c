#include "ti_msp_dl_config.h"
#include "oled.h"
#include "servo.h"

#define DELAY_1S_CYCLES 32000000U

int main(void)
{
    SYSCFG_DL_init();
    Servo_Init();

    delay_cycles(DELAY_1S_CYCLES);

    Servo_SetAngle(45U);

    OLED_Init();
    OLED_NewFrame();
    OLED_ShowString(1, 1, "MSPM0G3507");
    OLED_ShowString(2, 1, "OLED OK");
    OLED_ShowFrame();

    while (1) {
        DL_GPIO_togglePins(GPIO_GRP_LED_PORT, GPIO_GRP_LED_PIN_LED_0_PIN);
        delay_cycles(DELAY_1S_CYCLES);
    }
}
