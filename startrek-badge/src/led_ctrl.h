#ifndef LED_CTRL_H
#define LED_CTRL_H

enum led_state {
    LED_STATE_IDLE,
    LED_STATE_ADVERTISING,
    LED_STATE_CONNECTED,
    LED_STATE_STREAMING,
    LED_STATE_LOW_BATTERY
};

void led_ctrl_init(void);
void led_ctrl_set_state(enum led_state state);
enum led_state led_ctrl_get_state(void);
void led_ctrl_flash_feedback(void);
void led_ctrl_process(void);

#endif
