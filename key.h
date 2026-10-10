#ifndef KEY_H
#define KEY_H

#include "stm32f4xx_hal.h"

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SINGLE_CLICK,
    KEY_EVENT_DOUBLE_CLICK,
    KEY_EVENT_LONG_PRESS
}KeyEvent_t;

typedef enum {
    KEY_STATE_IDLE = 0,
    KEY_STATE_PRESS_DEBOUNCE,
    KEY_STATE_PRESSED,
    KEY_STATE_RELEASE_DEBOUNCE,
    KEY_STATE_WAIT_NEXT_CLICK,
    KEY_STATE_LONG_RELEASE_DEBOUNCE,
    KEY_STATE_WAIT_RELEASE
}KeyState_t;

typedef struct {
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    GPIO_PinState ActiveLevel;

    KeyState_t State;
    uint32_t Timer;             // 状态机内部计时器 (单位：扫描周期)
    uint8_t ClickCount;         // 记录连续按下的次数

    KeyEvent_t Event;
} Key_t;

void Key_StateMachine_Scan(Key_t *key);
uint8_t Choose_Title();

#endif
