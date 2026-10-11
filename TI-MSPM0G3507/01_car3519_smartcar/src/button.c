#include "button.h"
#include "ti_msp_dl_config.h"

typedef enum {
    BUTTON_KEY0 = 0,
    BUTTON_KEY1,
    BUTTON_KEY2,
    BUTTON_COUNT
} Button_Id;

static Button buttons[BUTTON_COUNT] = {
    {
        .port = GPIO_Key_Key0_PORT,
        .pin = GPIO_Key_Key0_PIN,
        .activeLow = true
    },
    {
        .port = GPIO_Key_Key1_PORT,
        .pin = GPIO_Key_Key1_PIN,
        .activeLow = true
    },
    {
        .port = GPIO_Key_Key2_PORT,
        .pin = GPIO_Key_Key2_PIN,
        .activeLow = true
    }
};

static volatile uint32_t buttonPendingEvents;

static bool Button_IsPressed(const Button *button)
{
    bool pinHigh = (DL_GPIO_readPins(button->port, button->pin) != 0U);

    return button->activeLow ? !pinHigh : pinHigh;
}

static uint32_t Button_EventMask(Button_Id button)
{
    return 1U << (uint32_t) button;
}

void Button_Init(void)
{
    Button_Id button;

    buttonPendingEvents = BUTTON_EVENT_NONE;

    for (button = BUTTON_KEY0; button < BUTTON_COUNT; button++) {
        Button *currentButton = &buttons[button];

        currentButton->state = Button_IsPressed(currentButton) ?
            BUTTON_STATE_PRESS_DEBOUNCE : BUTTON_STATE_IDLE;
        currentButton->ticks = 0U;
        currentButton->clickCount = 0U;
        currentButton->gesture = BUTTON_GESTURE_NONE;
    }
}

void Button_Scan(Button *button)
{
    bool pressed = Button_IsPressed(button);

    switch (button->state) {
    case BUTTON_STATE_IDLE:
        if (pressed) {
            button->state = BUTTON_STATE_PRESS_DEBOUNCE;
        }
        break;

    case BUTTON_STATE_PRESS_DEBOUNCE:
        button->state = pressed ? BUTTON_STATE_PRESSED :
            (button->clickCount > 0U ? BUTTON_STATE_WAIT_NEXT_CLICK : BUTTON_STATE_IDLE);
        if (pressed) {
            button->ticks = 0U;
        }
        break;

    case BUTTON_STATE_PRESSED:
        if (!pressed) {
            button->state = BUTTON_STATE_RELEASE_DEBOUNCE;
        } else if (++button->ticks >= BUTTON_LONG_PRESS_TICKS &&
            button->clickCount == 0U) {
            button->gesture = BUTTON_GESTURE_LONG_PRESS;
            button->state = BUTTON_STATE_WAIT_RELEASE;
        }
        break;

    case BUTTON_STATE_RELEASE_DEBOUNCE:
        if (pressed) {
            button->state = BUTTON_STATE_PRESSED;
        } else {
            button->clickCount++;
            button->ticks = 0U;
            button->state = BUTTON_STATE_WAIT_NEXT_CLICK;
        }
        break;

    case BUTTON_STATE_WAIT_NEXT_CLICK:
        if (pressed) {
            button->state = BUTTON_STATE_PRESS_DEBOUNCE;
        } else if (++button->ticks >= BUTTON_DOUBLE_CLICK_TICKS) {
            button->gesture = (button->clickCount == 1U) ?
                BUTTON_GESTURE_SINGLE_CLICK : BUTTON_GESTURE_DOUBLE_CLICK;
            button->clickCount = 0U;
            button->state = BUTTON_STATE_IDLE;
        }
        break;

    case BUTTON_STATE_WAIT_RELEASE:
        if (!pressed) {
            button->state = BUTTON_STATE_LONG_RELEASE_DEBOUNCE;
        }
        break;

    case BUTTON_STATE_LONG_RELEASE_DEBOUNCE:
        if (pressed) {
            button->state = BUTTON_STATE_WAIT_RELEASE;
        } else {
            button->clickCount = 0U;
            button->state = BUTTON_STATE_IDLE;
        }
        break;

    default:
        button->state = BUTTON_STATE_IDLE;
        button->ticks = 0U;
        button->clickCount = 0U;
        button->gesture = BUTTON_GESTURE_NONE;
        break;
    }
}

void Button_Tick10ms(void)
{
    Button_Id button;

    for (button = BUTTON_KEY0; button < BUTTON_COUNT; button++) {
        Button *currentButton = &buttons[button];

        Button_Scan(currentButton);
        if (currentButton->gesture != BUTTON_GESTURE_NONE) {
            buttonPendingEvents |= Button_EventMask(button);
            currentButton->gesture = BUTTON_GESTURE_NONE;
        }
    }
}

uint32_t Button_PollEvents(void)
{
    uint32_t interruptState = __get_PRIMASK();
    uint32_t events;

    __disable_irq();
    events = buttonPendingEvents;
    buttonPendingEvents = BUTTON_EVENT_NONE;
    __set_PRIMASK(interruptState);

    return events;
}
