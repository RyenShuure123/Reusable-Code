#include "LED.h"
#include "ti_msp_dl_config.h"

void LED_Green_On(void)
{
    DL_GPIO_clearPins(GPIO_LED_GREEN_PORT, GPIO_LED_GREEN_PIN);
}

void LED_Green_Off(void)
{
    DL_GPIO_setPins(GPIO_LED_GREEN_PORT, GPIO_LED_GREEN_PIN);
}

void LED_Red_On(void)
{
    DL_GPIO_clearPins(GPIO_LED_RED_PORT, GPIO_LED_RED_PIN);
}

void LED_Red_Off(void)
{
    DL_GPIO_setPins(GPIO_LED_RED_PORT, GPIO_LED_RED_PIN);
}

void LED_Blue_On(void)
{
    DL_GPIO_clearPins(GPIO_LED_BLUE_PORT, GPIO_LED_BLUE_PIN);
}

void LED_Blue_Off(void)
{
    DL_GPIO_setPins(GPIO_LED_BLUE_PORT, GPIO_LED_BLUE_PIN);
}

void LED_Red_Toggle(void)
{
    DL_GPIO_togglePins(GPIO_LED_RED_PORT, GPIO_LED_RED_PIN);
}

void LED_GREEN_Toggle(void)
{
    DL_GPIO_togglePins(GPIO_LED_GREEN_PORT, GPIO_LED_GREEN_PIN);
}

void LED_BLUE_Toggle(void)
{
    DL_GPIO_togglePins(GPIO_LED_BLUE_PORT, GPIO_LED_BLUE_PIN);
}
