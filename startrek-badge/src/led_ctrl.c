#include <zephyr/kernel.h>
#include <dk_buttons_and_leds.h>
#include "led_ctrl.h"

static enum led_state current_state = LED_STATE_IDLE;
static int64_t next_toggle_time = 0;
static bool led_on = false;

/*
 * LED Mapping:
 * LED1 (Green): Connection Status
 * LED2 (Green): Streaming Status
 */

void led_ctrl_init(void)
{
    // DK Leds already init in main
    led_ctrl_set_state(LED_STATE_IDLE);
}

void led_ctrl_set_state(enum led_state state)
{
    current_state = state;
    // Reset LEDs on state change
    dk_set_leds(0);
}

enum led_state led_ctrl_get_state(void)
{
    return current_state;
}

void led_ctrl_process(void)
{
    int64_t now = k_uptime_get();

    if (now < next_toggle_time) {
        return;
    }

    switch (current_state) {
        case LED_STATE_IDLE:
            dk_set_led(DK_LED1, 0);
            break;

        case LED_STATE_ADVERTISING:
            // Blink LED1 fast (200ms)
            led_on = !led_on;
            dk_set_led(DK_LED1, led_on);
            next_toggle_time = now + 200;
            break;

        case LED_STATE_CONNECTED:
            // LED1 Solid On
            dk_set_led(DK_LED1, 1);
            break;

        case LED_STATE_STREAMING:
             // LED1 Solid, LED2 Breathing or Blinking
             dk_set_led(DK_LED1, 1);
             led_on = !led_on;
             dk_set_led(DK_LED2, led_on);
             next_toggle_time = now + 500;
             break;

        case LED_STATE_LOW_BATTERY:
            // Blink all LEDs slow
            led_on = !led_on;
            dk_set_leds(led_on ? DK_ALL_LEDS_MSK : 0);
            next_toggle_time = now + 1000;
            break;
    }
}
