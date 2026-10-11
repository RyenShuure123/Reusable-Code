#include "stm32f4xx_hal.h"
#include "Key.h"


#define LONG_PRESS_TICKS    100  // 100 * 10ms = 1000ms (1秒长按)
#define DOUBLE_CLICK_TICKS  25   // 25 * 10ms = 250ms (双击间隔窗口)


// 初始化实例：GPIOA, PIN0, 低电平有效

void Key_StateMachine_Scan(Key_t *key) {
    GPIO_PinState currentLevel = HAL_GPIO_ReadPin(key->GPIOx, key->GPIO_Pin);

    switch (key->State) {
        case KEY_STATE_IDLE:
            if (currentLevel == key->ActiveLevel) {
                key->State = KEY_STATE_PRESS_DEBOUNCE;
            }
            break;

        case KEY_STATE_PRESS_DEBOUNCE:
            if (currentLevel == key->ActiveLevel) {
                key->State = KEY_STATE_PRESSED;
                key->Timer = 0; // 进入按下状态，清零计时器，准备长按计时
            } else {
                // 如果之前已经有单击记录，但这次是抖动，退回到等待下一次点击状态继续倒计时
                key->State = (key->ClickCount > 0) ? KEY_STATE_WAIT_NEXT_CLICK : KEY_STATE_IDLE;
            }
            break;

        case KEY_STATE_PRESSED:
            if (currentLevel != key->ActiveLevel) {
                // 按键松开了
                key->State = KEY_STATE_RELEASE_DEBOUNCE;
            } else {
                // 按键一直按着，计时器累加
                key->Timer++;
                // 判断是否达到长按阈值 (并且要求是第一次按下，防止双击的第二次被判定为长按)
                if (key->Timer >= LONG_PRESS_TICKS && key->ClickCount == 0) {
                    key->Event = KEY_EVENT_LONG_PRESS; // 触发长按事件
                    key->State = KEY_STATE_WAIT_RELEASE; // 跳转到等待释放状态
                }
            }
            break;

        case KEY_STATE_WAIT_RELEASE: // 长按触发后，死等松手
            if (currentLevel != key->ActiveLevel) {
                // 检测到松手，进入【长按专属】的松开消抖
                key->State = KEY_STATE_LONG_RELEASE_DEBOUNCE;
            }
            break;

        case KEY_STATE_LONG_RELEASE_DEBOUNCE: // 长按专属消抖
            if (currentLevel != key->ActiveLevel) {
                // 确实松开了，直接回空闲态，绝不增加 ClickCount
                key->State = KEY_STATE_IDLE;
                key->ClickCount = 0; // 保险起见清零
            } else {
                // 是抖动，退回死等松手状态
                key->State = KEY_STATE_WAIT_RELEASE;
            }
            break;

        case KEY_STATE_RELEASE_DEBOUNCE: // 短按专属消抖
            if (currentLevel != key->ActiveLevel) {
                // 纯粹的短按松开，正常进入双击等待窗口
                key->ClickCount++;
                key->Timer = 0;
                key->State = KEY_STATE_WAIT_NEXT_CLICK;
            } else {
                key->State = KEY_STATE_PRESSED; // 抖动，退回按下态
            }
            break;

        case KEY_STATE_WAIT_NEXT_CLICK:
            if (currentLevel == key->ActiveLevel) {
                // 在窗口期内又按下了！
                key->State = KEY_STATE_PRESS_DEBOUNCE;
            } else {
                // 没按下，计时器累加
                key->Timer++;
                // 判断窗口期是否超时
                if (key->Timer >= DOUBLE_CLICK_TICKS) {
                    // 超时了，结算连击次数
                    if (key->ClickCount == 1) {
                        key->Event = KEY_EVENT_SINGLE_CLICK; // 触发单击
                    } else if (key->ClickCount >= 2) {
                        key->Event = KEY_EVENT_DOUBLE_CLICK; // 触发双击
                    }

                    // 状态重置
                    key->State = KEY_STATE_IDLE;
                    key->ClickCount = 0;
                }
            }
            break;

        default:
            key->State = KEY_STATE_IDLE;
            key->ClickCount = 0;
            break;
    }
}

uint8_t Choose_Title(void) {
    uint8_t bits[4];
    uint8_t result = 0;

    bits[0] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET) ? 1 : 0;
    bits[1] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET) ? 1 : 0;
    bits[2] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) ? 1 : 0;
    bits[3] = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_SET) ? 1 : 0;

    //  逆序遍历：从 bits[3] 到 bits[0]
    for (int8_t i = 3; i >= 0; i--) {  // 必须用 int8_t，避免 uint8_t 死循环
        result = (result << 1) | bits[i];
    }
    return result & 0x0F;
}
