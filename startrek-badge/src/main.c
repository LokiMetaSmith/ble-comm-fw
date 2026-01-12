#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/random/random.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <stdio.h>

#include "le_audio.h"
#include "battery.h"
#include "led_ctrl.h"

LOG_MODULE_REGISTER(main);

#define DEVICE_NAME_PREFIX "ComBadge-"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME_PREFIX) + 4)

static const struct device *i2s_dev = DEVICE_DT_GET(DT_ALIAS(i2s_dev));
static struct k_poll_signal i2s_rx_sig;
static struct k_poll_signal i2s_tx_sig;

/* Audio Buffers */
#define AUDIO_BLOCK_SIZE_BYTES 128
#define AUDIO_NUM_BLOCKS 4
K_MEM_SLAB_DEFINE_STATIC(audio_mem_slab, AUDIO_BLOCK_SIZE_BYTES, AUDIO_NUM_BLOCKS, 4);

/* I2S Config */
struct i2s_config i2s_cfg;

/* State */
static bool is_connected = false;

static void get_random_name(char *name, size_t len)
{
    uint16_t rand_val;
    sys_rand_get(&rand_val, sizeof(rand_val));
    snprintf(name, len, "%s%04X", DEVICE_NAME_PREFIX, rand_val);
}

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ASCS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
    } else {
        LOG_INF("Connected");
        is_connected = true;
        led_ctrl_set_state(LED_STATE_CONNECTED);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);
    is_connected = false;
    led_ctrl_set_state(LED_STATE_ADVERTISING);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    if (has_changed & DK_BTN3_MSK) {
        if (button_state & DK_BTN3_MSK) {
            LOG_INF("Touch Button Pressed - Toggle Play/Pause");
            // Here we would toggle streaming state or send a command
        }
    }
}

static int i2s_setup(void)
{
    if (!device_is_ready(i2s_dev)) {
        LOG_ERR("I2S device not ready");
        return -ENODEV;
    }

    i2s_cfg.word_size = 16;
    i2s_cfg.channels = 2;
    i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
    i2s_cfg.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
    i2s_cfg.frame_clk_freq = 48000;
    i2s_cfg.mem_slab = &audio_mem_slab;
    i2s_cfg.block_size = AUDIO_BLOCK_SIZE_BYTES;
    i2s_cfg.timeout = 2000;

    int ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure I2S TX: %d", ret);
        return ret;
    }

    ret = i2s_configure(i2s_dev, I2S_DIR_RX, &i2s_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure I2S RX: %d", ret);
        return ret;
    }

    return 0;
}

void main(void)
{
    int err;
    char device_name[DEVICE_NAME_LEN + 1];

    LOG_INF("Starting Star Trek Com Badge Firmware");

    // Init Modules
    led_ctrl_init();

    err = dk_buttons_init(button_handler);
    if (err) LOG_ERR("Failed to init buttons (err %d)", err);

    err = battery_init();
    if (err) LOG_ERR("Failed to init battery (err %d)", err);

    // Initialize I2S
    err = i2s_setup();
    if (err) {
        LOG_ERR("Failed to init I2S (err %d)", err);
    } else {
        LOG_INF("I2S Initialized");
    }

    // Bluetooth Setup
    get_random_name(device_name, sizeof(device_name));
    bt_set_name(device_name);

    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    le_audio_init();

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    led_ctrl_set_state(LED_STATE_ADVERTISING);
    LOG_INF("Advertising started as %s", device_name);

    // Main loop
    while (1) {
        // Battery Monitor (every 10s)
        static int64_t next_batt_check = 0;
        int64_t now = k_uptime_get();

        if (now >= next_batt_check) {
            int level = battery_sample();
            if (level < 20) {
                led_ctrl_set_state(LED_STATE_LOW_BATTERY);
            } else if (is_connected) {
                 // Restore state if we were low battery but now okay (e.g. charging?)
                 // Simplification: just reset to connected
                 if (led_ctrl_get_state() == LED_STATE_LOW_BATTERY) {
                     led_ctrl_set_state(LED_STATE_CONNECTED);
                 }
            }
            next_batt_check = now + 10000;
        }

        led_ctrl_process();
        k_sleep(K_MSEC(50));
    }
}
