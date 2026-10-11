#ifndef CAR3519NEW_BUTTON_H
#define CAR3519NEW_BUTTON_H

#include <ti/driverlib/driverlib.h>

#define BUTTON_SCAN_PERIOD_MS 10U
#define BUTTON_LONG_PRESS_TICKS 100U
#define BUTTON_DOUBLE_CLICK_TICKS 25U

typedef enum {
    BUTTON_EVENT_NONE = 0U,
    BUTTON_EVENT_KEY0_PRESSED = (1U << 0),
    BUTTON_EVENT_KEY1_PRESSED = (1U << 1),
    BUTTON_EVENT_KEY2_PRESSED = (1U << 2)
} Button_Event;

typedef enum {
    BUTTON_GESTURE_NONE = 0,
    BUTTON_GESTURE_SINGLE_CLICK,
    BUTTON_GESTURE_DOUBLE_CLICK,
    BUTTON_GESTURE_LONG_PRESS
} Button_Gesture;

typedef enum {
    BUTTON_STATE_IDLE = 0,
    BUTTON_STATE_PRESS_DEBOUNCE,
    BUTTON_STATE_PRESSED,
    BUTTON_STATE_RELEASE_DEBOUNCE,
    BUTTON_STATE_WAIT_NEXT_CLICK,
    BUTTON_STATE_LONG_RELEASE_DEBOUNCE,
    BUTTON_STATE_WAIT_RELEASE
} Button_State;

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    bool activeLow;
    Button_State state;
    uint16_t ticks;
    uint8_t clickCount;
    Button_Gesture gesture;
} Button;

void Button_Init(void);
void Button_Scan(Button *button);
void Button_Tick10ms(void);
uint32_t Button_PollEvents(void);

#endif //CAR3519NEW_BUTTON_H
